//###########################################################################
// This file is part of LImA, a Library for Image Acquisition
//
// Copyright (C) : 2009-2022
// European Synchrotron Radiation Facility
// CS40220 38043 Grenoble Cedex 9 
// FRANCE
//
// Contact: lima@esrf.fr
//
// This is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 3 of the License, or
// (at your option) any later version.
//
// This software is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, see <http://www.gnu.org/licenses/>.
//###########################################################################
#include "lz4.h"
#include "bitshuffle.h"

#include "EigerDecompress.h"
#include "EigerStream.h"

#include <eigerapi/Requests.h>

#include "processlib/LinkTask.h"
#include "processlib/ProcessExceptions.h"

#include "lima/SidebandData.h"

using namespace lima;
using namespace lima::Eiger;
using namespace eigerapi;

class _DecompressTask : public LinkTask
{
  DEB_CLASS_NAMESPC(DebModCamera,"_DecompressTask","Eiger");
public:
  virtual Data process(Data&);

private:
  typedef Stream::ImageData ImageData;
  typedef std::shared_ptr<ImageData> ImageDataPtr;
};

template <typename S, typename D>
void _expand(void *src, void *dst, int nbItems)
{
  S *src_data = (S *) src;
  D *dst_data = (D *) dst;
  while(nbItems) {
    *dst_data = unsigned(*src_data);
    ++dst_data,++src_data,--nbItems;
  }
}

typedef unsigned char aligned16_uint8 __attribute__ ((aligned (16)));
typedef unsigned short aligned16_uint16 __attribute__ ((aligned (16)));
typedef unsigned int aligned16_uint32 __attribute__ ((aligned (16)));

inline void _expand_8_to_16(void *src, void *dst, int nbItems)
{ _expand<aligned16_uint8, aligned16_uint16>(src, dst, nbItems); }
inline void _expand_8_to_32(void *src, void *dst, int nbItems)
{ _expand<aligned16_uint8, aligned16_uint32>(src, dst, nbItems); }
inline void _expand_16_to_32(void *src, void *dst, int nbItems)
{ _expand<aligned16_uint16, aligned16_uint32>(src, dst, nbItems); }

Data _DecompressTask::process(Data& out)
{
  DEB_MEMBER_FUNCT();
  DEB_PARAM() << DEB_VAR1(out.frameNumber);
  ImageDataPtr img_data = Sideband::GetData<ImageData>("eiger_data", out);
  if (!bool(img_data))
    throw ProcessException("Cannot get plugin_data");
  Sideband::RemoveData("eiger_data", out);
  void *msg_data;
  size_t msg_size;
  img_data->getMsgDataNSize(msg_data, msg_size);
  const int& depth = img_data->depth;
  const Camera::CompressionType& type = img_data->comp_type;
  bool expand = (out.depth() != depth);
  int nb_pixels = out.size() / out.depth();
  int size = nb_pixels * depth;
  bool decompress = (type != Camera::NoCompression);
  HeapPtr<void> aux_buffer;
  if(expand && decompress) {
    void *ptr;
    if(posix_memalign(&ptr,16,size))
      throw ProcessException("Can't allocate temporary memory");
    aux_buffer.reset(ptr);
  }
  void *lima_buffer = out.data();
  void *decompress_out = aux_buffer ? aux_buffer.get() : lima_buffer;
  int return_code = 0;
  if(type == Camera::LZ4) {
    return_code = LZ4_decompress_fast((const char*)msg_data,
				      (char*)decompress_out,size);
  } else if(type == Camera::BSLZ4) {
    struct bslz4_data {
      uint64_t data_size_be;
      uint32_t block_size_be;
      char data[1];
    } *d = (bslz4_data *) msg_data;
    if(be64toh(d->data_size_be) != size)
      throw ProcessException("Data size mismatch");
    size_t block_size = be32toh(d->block_size_be) / depth;
    return_code = bshuf_decompress_lz4(d->data, decompress_out, nb_pixels,
				       depth, block_size);
  }
  if(return_code < 0) {
    char ErrorBuff[1024];
    snprintf(ErrorBuff,sizeof(ErrorBuff),
	     "_DecompressTask: decompression failed, (error code: %d) (data size %d)",
	     return_code,out.size());
    throw ProcessException(ErrorBuff);
  }

  if(expand) {
    void *expand_src = decompress ? decompress_out : msg_data;
    if(out.depth() == 2)
      _expand_8_to_16(expand_src, lima_buffer, nb_pixels);
    else if(depth == 1) 
      _expand_8_to_32(expand_src, lima_buffer, nb_pixels);
    else
      _expand_16_to_32(expand_src, lima_buffer, nb_pixels);
  } else if(!decompress)
    memcpy(lima_buffer, msg_data, size);
  
  return out;
}

Decompress::Decompress() :
  m_decompress_task(new _DecompressTask())
{
}

Decompress::~Decompress()
{
  m_decompress_task->unref();
}

LinkTask* Decompress::getReconstructionTask()
{
  return m_decompress_task;
}

void Decompress::setActive(bool active)
{
  reconstructionChange(active ? m_decompress_task : NULL);
}
