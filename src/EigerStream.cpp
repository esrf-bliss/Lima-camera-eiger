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
#include <fcntl.h>
#include <unistd.h>

#include <map>
#include <set>

#include <zmq.h>

#include <json/json.h>

#include <eigerapi/Requests.h>
#include <eigerapi/EigerDefines.h>

#include "lima/Exceptions.h"
#include "EigerStream.h"

#define _BSD_SOURCE
#include <endian.h>

using namespace lima;
using namespace lima::Eiger;
using namespace eigerapi;
//			--- Message struct ---
struct Stream::Message
{
  Message()
  {
    zmq_msg_init(&msg);
  }
  ~Message()
  {
    zmq_msg_close(&msg);
  }
  zmq_msg_t* get_msg() {return &msg;}

  zmq_msg_t msg;
};
//		      --- buffer management ---
class Stream::_BufferCtrlObj : public SoftBufferCtrlObj
{
  DEB_CLASS_NAMESPC(DebModCamera,"Stream","_BufferCtrlObj");
public:
  _BufferCtrlObj(Stream& stream) : 
    m_stream(stream)
  {
  }
private:
  Stream&	m_stream;
};

//			 --- Stream class ---
Stream::Stream(Camera& cam) : 
  m_cam(cam),
  m_active(false),
  m_header_detail(OFF),
  m_dirty_flag(true),
  m_wait(true),
  m_running(false),
  m_stop(false),
  m_buffer_ctrl_obj(new Stream::_BufferCtrlObj(*this))
{
  DEB_CONSTRUCTOR();

  bool is_le = (htole16(0x1234) == 0x1234);
  m_endianess = (is_le ? '<' : '>');

  m_zmq_context = zmq_ctx_new();
  if(pipe(m_pipes))
    THROW_HW_ERROR(Error) << "Can't open pipe";

  pthread_create(&m_thread_id,NULL,_runFunc,this);
}

Stream::~Stream()
{
  AutoMutex aLock(m_cond.mutex());
  m_stop = true;
  m_cond.broadcast();
  aLock.unlock();
  _send_synchro();

  if(m_thread_id > 0)
    pthread_join(m_thread_id,NULL);

  close(m_pipes[0]),close(m_pipes[1]);
  zmq_ctx_destroy(m_zmq_context);

  delete m_buffer_ctrl_obj;
}

void Stream::start()
{
  AutoMutex aLock(m_cond.mutex());
  m_buffer_ctrl_obj->getBuffer().setStartTimestamp(Timestamp::now());
}

void Stream::stop()
{
  setActive(false);
}

void Stream::_send_synchro()
{
  DEB_MEMBER_FUNCT();

  if(write(m_pipes[1],"|",1) == -1)
    DEB_ERROR() << "Something wrong happened!";
}

bool Stream::isRunning() const
{
  AutoMutex aLock(m_cond.mutex());
  return m_running;
}

void Stream::getCompressionType(Camera::CompressionType& type) const
{
  m_cam.getCompressionType(type);
}

void Stream::getHeaderDetail(Stream::HeaderDetail& detail) const
{
  AutoMutex lock(m_cond.mutex());
  detail = m_header_detail;
}

void Stream::setHeaderDetail(Stream::HeaderDetail detail)
{
  AutoMutex lock(m_cond.mutex());
  m_header_detail = detail,m_dirty_flag = true;
}

void Stream::setActive(bool active)
{
  DEB_MEMBER_FUNCT();
  DEB_PARAM() << DEB_VAR1(active);

  AutoMutex lock(m_cond.mutex());
  //Don't resend parameters if not changed
  if(active != m_active || m_dirty_flag)
    {
      const char* header_detail_str;
      switch(m_header_detail)
	{
	case ALL:
	  header_detail_str = "all";break;
	case BASIC:
	  header_detail_str = "basic";break;
	default:
	  header_detail_str = "none";break;
	}

      DEB_TRACE() << "STREAM_HEADER_DETAIL:" << DEB_VAR1(header_detail_str);
      std::shared_ptr<Requests::Param> header_detail_req = 
	m_cam.m_requests->set_param(Requests::STREAM_HEADER_DETAIL,header_detail_str);
      try {
	header_detail_req->wait();
      } catch (const eigerapi::EigerException &e) {
	m_cam.m_requests->cancel(header_detail_req);
	THROW_HW_ERROR(Error) << e.what();
      }

      const char* active_str = active ? "enabled" : "disabled";
      DEB_TRACE() << "STREAM_MODE:" << DEB_VAR1(active_str);
      std::shared_ptr<Requests::Param> active_req = 
	m_cam.m_requests->set_param(Requests::STREAM_MODE,active_str);
      try {
	active_req->wait();
      } catch (const eigerapi::EigerException &e) {
	m_cam.m_requests->cancel(active_req);
	THROW_HW_ERROR(Error) << e.what();
      }
    }
  m_dirty_flag = false;
  if(active == m_active)
    return;
  
  m_active = active;
  m_wait = !m_active;
  if(m_active)
    m_activate_tstamp = Timestamp::now();
  else
    _send_synchro();

  m_cond.broadcast();
  while(m_running != m_active)
    m_cond.wait();
}

HwBufferCtrlObj* Stream::getBufferCtrlObj()
{
  DEB_MEMBER_FUNCT();
  return m_buffer_ctrl_obj;
}

void* Stream::_runFunc(void *streamPt)
{
  ((Stream*)streamPt)->_run();
  return NULL;
}

#define _CHECK_RETURN(funct)			\
  if(funct == -1)					\
    {						\
      if(errno == EAGAIN)				\
	{								\
	  DEB_TRACE() << "zmq EAGAIN";					\
	  break;							\
	}								\
									\
      continue_flag = false;						\
      char errno_buffer[256];						\
      char* errno_msg = strerror_r(errno,errno_buffer,sizeof(errno_buffer)); \
      DEB_ERROR() << "Something bad appends stream reading will stop (errno: " \
		  << errno_msg << ")";					\
      DEB_ERROR() << "After rx " << pending_messages.size() << " message(s)"; \
      break;								\
    }


static inline bool _get_json_header(std::shared_ptr<Stream::Message> &msg,
				    Json::Value& header)
{
  DEB_GLOBAL_FUNCT();
  void* data = zmq_msg_data(msg->get_msg());
  size_t data_size = zmq_msg_size(msg->get_msg());
  DEB_TRACE() << "json_header=" << std::string((char *) data, data_size);
  
  const char* begin = (const char*)data;
  const char* end = begin + data_size;
  Json::Reader reader;
  return reader.parse(begin,end,header);
}

#ifdef READ_HEADER
static bool _get_header(const Json::Value& stream_header,
			int nb_messages,std::vector<zmq_msg_t> &pending_messages,
			Json::Value& header)
{
  std::string header_detail = stream_header.get("header_detail","").asString();
  int message_id;
  if(nb_messages > 1 && header_detail == "none")
    message_id = 1;
  else if(nb_messages > 2 && header_detail == "basic")
    message_id = 2;
  else if(nb_messages > 8 && header_detail == "all")
    message_id = 8;
  else				// Unknown header detail
    return false;

  return _get_json_header(pending_messages[message_id],header);
}
#endif

void Stream::_run()
{
  DEB_MEMBER_FUNCT();
  
  AutoMutex aLock(m_cond.mutex());
  StdBufferCbMgr& buffer_mgr = m_buffer_ctrl_obj->getBuffer();

  while(1)
    {
      void* stream_socket = NULL;
      while(m_wait && !m_stop)
	{
	  DEB_TRACE() << "Wait";
	  m_running = false;
	  m_cond.broadcast();
	  m_cond.wait();
	  m_running = true;
	  DEB_TRACE() << "Running";
	}
      if(m_stop) break;
      int nb_frames;
      m_cam.getNbFrames(nb_frames);
      TrigMode trigger_mode;
      m_cam.getTrigMode(trigger_mode);

      bool continue_flag = true;
      //open stream socket
      char stream_endpoint[256];
      snprintf(stream_endpoint,sizeof(stream_endpoint),
	       "tcp://%s:9999",m_cam.getDetectorIp().c_str());
      stream_socket = zmq_socket(m_zmq_context,ZMQ_PULL);

      if(!zmq_connect(stream_socket,stream_endpoint))
	{
	  m_cond.broadcast();
	  aLock.unlock();

	  DEB_TRACE() << "connected to " << stream_endpoint;
	  //  Initialize poll set
	  zmq_pollitem_t items [] = {
	    { NULL, m_pipes[0], ZMQ_POLLIN, 0 },
	    { stream_socket, 0, ZMQ_POLLIN, 0 }
	  };
	  while(continue_flag)		// reading loop
	    {
	      DEB_TRACE() << "Enter poll";
	      zmq_poll(items,2,-1);
	      DEB_TRACE() << "Exit poll";

	      if(items[0].revents & ZMQ_POLLIN)
		{
		  char buffer[1024];
		  if(read(m_pipes[0],buffer,sizeof(buffer)) == -1)
		    DEB_WARNING() << "Something strange happened!";

		  aLock.lock();
		  continue_flag = !m_wait && !m_stop;
		  aLock.unlock();
		}
	      if(items[1].revents & ZMQ_POLLIN) // reading stream
		{
		  std::vector<std::shared_ptr<Stream::Message>> pending_messages;
		  pending_messages.reserve(9);
		  int more;
		  do {
		    std::shared_ptr<Stream::Message> msg(new Stream::Message());
		    _CHECK_RETURN(zmq_msg_recv(msg->get_msg(),stream_socket,0));
		    more = zmq_msg_more(msg->get_msg());
		    pending_messages.emplace_back(msg);
		  } while(more);
		  int nb_messages = pending_messages.size();
		  DEB_TRACE() << DEB_VAR1(nb_messages);
		  if(nb_messages > 0)
		    {
		      Json::Value stream_header;
		      continue_flag = _get_json_header(pending_messages[0],stream_header);
		      if(continue_flag)
			{
			  std::string htype = stream_header.get("htype","").asString();
			  DEB_TRACE() << DEB_VAR1(htype);
#ifdef READ_HEADER
			  if(htype.find("dheader-") != std::string::npos)
			    {
			      Json::Value header;
			      continue_flag = _get_header(stream_header,nb_messages,
							  pending_messages,header);
			      
			    }
			  else 
#endif
			    if(htype.find("dimage-") != std::string::npos)
			    {
			      int frameid = stream_header.get("frame",-1).asInt();
			      DEB_TRACE() << DEB_VAR1(frameid);
			      //stream_header.get("hash","md5sum")
			      if(nb_messages < 3)
				{
				  DEB_ERROR() << "Should receive at least 3 messages part, only received " 
					      << nb_messages;
				  break;
				}

			      Timestamp tstart;
			      buffer_mgr.getStartTimestamp(tstart);
			      if (!tstart.isSet() || (tstart < m_activate_tstamp)) {
				DEB_ERROR() << "Frame before start: " << DEB_VAR1(frameid);
				continue;
			      }

			      Json::Value data_header;
			      if(!_get_json_header(pending_messages[1],data_header)) break;

			      //Data size (width,height)
			      Json::Value shape = data_header.get("shape","");
			      if(!shape.isArray() || shape.size() != 2) break;
			      FrameDim anImageDim;
			      anImageDim.setSize(Size(shape[0u].asInt(),shape[1u].asInt()));
			      //data type
			      ImageType image_type;
			      std::string dtype = data_header.get("type","none").asString();
			      if(dtype == "int32")
				image_type = Bpp32S;
			      else if(dtype == "uint32")
				image_type = Bpp32;
			      else if(dtype == "int16")
				image_type = Bpp16S;
			      else if(dtype == "uint16")
				image_type = Bpp16;
			      else
				break;
			      anImageDim.setImageType(image_type);
			      DEB_TRACE() << DEB_VAR1(anImageDim);

			      if (frameid == 0) {
				m_last_info.encoding = data_header.get("encoding", "").asString();
				m_last_info.frame_dim = anImageDim;
				m_last_info.packed_size = data_header.get("size", "-1").asInt();
				Camera::CompressionType comp_type;
				_checkCompression(m_last_info, comp_type);
			      }

			      HwFrameInfoType frame_info;
			      frame_info.acq_frame_nb = frameid;
			      void* buffer_ptr = buffer_mgr.getFrameBufferPtr(frameid);
			      m_data_2_msg[buffer_ptr] = MessageNDepth(pending_messages[2],
								       anImageDim.getDepth());

#ifdef READ_HEADER
			      if(nb_messages == 5)
				{
				  zmq_msg_t& msg = pending_messages[4]->msg;
				  char* headerpt = (char*)zmq_msg_data(&msg);
				  size_t header_size = zmq_msg_size(&msg);
				}
#endif
			      continue_flag = buffer_mgr.newFrameReady(frame_info);
			      m_cam.m_image_number++;
			      if(trigger_mode != IntTrig && trigger_mode != IntTrigMult && !--nb_frames)
				m_cam.disarm();
			    }
			    else if(htype.find("dseries_end-") != std::string::npos)
			      continue_flag = false;
			}
		    }
		}
	    }
	}
      else
	{
	  char error_buffer[256];
	  char* error_msg = strerror_r(errno,error_buffer,sizeof(error_buffer));
	  DEB_ERROR() << "Connection error: " << DEB_VAR2(errno,error_msg);
	aLock.unlock();
	}

      if(stream_socket) zmq_close(stream_socket);
      DEB_TRACE() << "disconnected from " << stream_endpoint;
      aLock.lock();
      m_active = false;
      m_wait = !m_active;
    }
  m_running = false;
}

void Stream::_checkCompression(const StreamInfo& info,
			       Camera::CompressionType& comp_type)
{
  DEB_MEMBER_FUNCT();
  DEB_PARAM() << DEB_VAR2(info, m_endianess);

  const std::string& encoding = info.encoding;

  char endianess = *encoding.rbegin();
  if (endianess != m_endianess)
    THROW_HW_ERROR(Error) << "Endianess mismatch: "
			  << "got " << endianess << ", "
			  << "expected " << m_endianess;

  if (encoding == std::string(1, m_endianess)) {
    comp_type = Camera::NoCompression;
  } else {
    const std::string lz4 = std::string("lz4") + m_endianess;
    if (encoding == lz4) {
      comp_type = Camera::LZ4;
    } else {
      const char *bs;
      switch (info.frame_dim.getImageType()) {
      case Bpp32S: case Bpp32: bs = "bs32-"; break;
      case Bpp16S: case Bpp16: bs = "bs16-"; break;
      case Bpp8S:  case Bpp8:  bs = "bs8-";  break;
      }
      if (encoding == std::string(bs) + lz4)
	comp_type = Camera::BSLZ4;
      else
	THROW_HW_ERROR(Error) << "Unexpected encoding: " << encoding;
    }
  }

  Camera::CompressionType expected_comp_type;
  getCompressionType(expected_comp_type);
  if (comp_type != expected_comp_type)
    THROW_HW_ERROR(Error) << "Unexpected compression type: " << comp_type;

  DEB_RETURN() << DEB_VAR1(comp_type);
}

void Stream::getLastStreamInfo(StreamInfo& last_info)
{
  DEB_MEMBER_FUNCT();
  last_info = m_last_info;
  DEB_RETURN() << DEB_VAR1(last_info);
}

bool Stream::get_msg(void* aDataBuffer,void*& msg_data,size_t& msg_size,int& depth)
{
  DEB_MEMBER_FUNCT();
  DEB_PARAM() << DEB_VAR1(aDataBuffer);

  AutoMutex lock(m_cond.mutex());
  Data2Message::iterator it = m_data_2_msg.find(aDataBuffer);
  if(it == m_data_2_msg.end())
    return false;

  MessageNDepth message_depth = it->second;
  std::shared_ptr<Stream::Message> message = message_depth.first;
  depth = message_depth.second;
  msg_data = zmq_msg_data(message->get_msg());
  msg_size = zmq_msg_size(message->get_msg());
  DEB_RETURN() << DEB_VAR2(msg_data,msg_size);
  return true;
}

void Stream::release_msg(void* aDataBuffer)
{
  DEB_MEMBER_FUNCT();
  DEB_PARAM() << DEB_VAR1(aDataBuffer);

  AutoMutex lock(m_cond.mutex());
  Data2Message::iterator it = m_data_2_msg.find(aDataBuffer);
  if(it == m_data_2_msg.end())
    THROW_HW_ERROR(Error) << "Internal error: releasing buffer not in list";

  m_data_2_msg.erase(it);
}

void Stream::release_all_msgs()
{
  DEB_MEMBER_FUNCT();

  AutoMutex lock(m_cond.mutex());
  m_data_2_msg.clear();
}
