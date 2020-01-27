//###########################################################################
// This file is part of LImA, a Library for Image Acquisition
//
// Copyright (C) : 2009-2015
// European Synchrotron Radiation Facility
// BP 220, Grenoble 38043
// FRANCE
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

#include "processlib/LinkTask.h"
#include "processlib/ProcessExceptions.h"

using namespace lima;
using namespace lima::Eiger;

class _DecompressTask : public LinkTask
{
  DEB_CLASS_NAMESPC(DebModCamera,"_DecompressTask","Eiger");
public:
  _DecompressTask(Stream& stream) : m_stream(stream) {}
  virtual Data process(Data&);

private:
  Stream& m_stream;
};

void _expand(void *src,Data& dst)
{
  int nbItems = dst.size() / dst.depth();
  unsigned short* src_data = (unsigned short*)src;
  unsigned int* dst_data = (unsigned int*)dst.data();
  while(nbItems)
    {
      *dst_data = unsigned(*src_data);
      ++dst_data,++src_data,--nbItems;
    }
  dst.type = Data::UINT32;
}

Data _DecompressTask::process(Data& out)
{
  void *msg_data;
  size_t msg_size;
  int depth;
  void *lima_buffer = out.data();
  if(!m_stream.get_msg(lima_buffer,msg_data,msg_size,depth))
    throw ProcessException("_DecompressTask: can't find compressed message");
  int size;
  bool expand_16_to_32bit = ((out.depth() == 4) && (depth == 2));
  size = out.size() / (expand_16_to_32bit ? 2 : 1);
  Camera::CompressionType type;
  m_stream.getCompressionType(type);
  bool decompress = (type != Camera::NoCompression);
  void *aux_buffer = NULL;
  if(expand_16_to_32bit && decompress)
    if(posix_memalign(&aux_buffer,16,size))
      throw ProcessException("Can't allocate temporary memory");

  void *decompress_out = expand_16_to_32bit ? aux_buffer : lima_buffer;
  int return_code = 0;
  if (type == Camera::LZ4) {
    return_code = LZ4_decompress_fast((const char*)msg_data,(char*)decompress_out,size);
  } else if (type == Camera::BSLZ4) {
    size_t nb_elements = size / depth;
    return_code = bshuf_decompress_lz4(msg_data, decompress_out, nb_elements, depth, 0);
  }
  if(return_code < 0)
    {
      if(aux_buffer) free(aux_buffer);

      char ErrorBuff[1024];
      snprintf(ErrorBuff,sizeof(ErrorBuff),
	       "_DecompressTask: decompression failed, (error code: %d) (data size %d)",
	       return_code,out.size());
      throw ProcessException(ErrorBuff);
    }

  if(expand_16_to_32bit) {
    void *expand_src = decompress ? decompress_out : msg_data;
    _expand(expand_src,out);
  } else if(!decompress)
    memcpy(lima_buffer, msg_data, size);
  
  if(aux_buffer)
    free(aux_buffer);

  m_stream.release_msg(lima_buffer);

  return out;
}

Decompress::Decompress(Stream& stream) :
  m_decompress_task(new _DecompressTask(stream))
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
