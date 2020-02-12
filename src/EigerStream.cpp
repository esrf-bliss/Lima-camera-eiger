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

#include "EigerCameraRequests.h"
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

  void get_msg_data_n_size(void*& data, size_t& size)
  {
    data = zmq_msg_data(&msg);
    size = zmq_msg_size(&msg);
  }

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

//		      --- Stream::ImageData ---
void Stream::ImageData::getMsgDataNSize(void*& data, size_t& size) const
{
  msg->get_msg_data_n_size(data, size);
}

std::ostream& lima::Eiger::operator <<(std::ostream& os, Stream::State state)
{
  const char *name;
  switch (state) {
  case Stream::State::Init: name = "Init"; break;
  case Stream::State::Idle: name = "Idle"; break;
  case Stream::State::Starting: name = "Starting"; break;
  case Stream::State::Connected: name = "Connected"; break;
  case Stream::State::Failed: name = "Failed"; break;
  case Stream::State::Armed: name = "Armed"; break;
  case Stream::State::Running: name = "Running"; break;
  case Stream::State::Stopped: name = "Stopped"; break;
  case Stream::State::Aborting: name = "Aborting"; break;
  case Stream::State::Quitting: name = "Quitting"; break;
  default: name = "Unknown";
  }
  return os << name;
}

std::ostream& lima::Eiger::operator <<(std::ostream& os,
				       const Stream::ImageData& img_data)
{
  void *msg_data;
  size_t msg_size;
  img_data.getMsgDataNSize(msg_data, msg_size);
  return os << "<"
	    << "data=" << msg_data << ", "
	    << "size=" << msg_size << ", "
	    << "depth=" << img_data.depth << ", "
	    << "comp_type=" << img_data.comp_type
	    << ">";
}

//			 --- Stream class ---
inline bool Stream::_isRunning() const
{
  return ((m_state != Idle) && (m_state != Failed));
}

inline Json::Value Stream::_get_json_header(MessagePtr &msg)
{
  DEB_MEMBER_FUNCT();
  void* data;
  size_t data_size;
  msg->get_msg_data_n_size(data, data_size);
  DEB_TRACE() << "json_header=" << std::string((char *) data, data_size);
  
  const char* begin = (const char*)data;
  const char* end = begin + data_size;
  Json::Value header;
  Json::Reader reader;
  if (!reader.parse(begin,end,header))
    THROW_HW_ERROR(Error) << "Error parsing header: " << std::string(begin, end);
  return header;
}

inline Json::Value Stream::_get_global_header(const Json::Value& stream_header,
					      MessageList& pending_messages)
{
  DEB_MEMBER_FUNCT();
  HeaderDetail header_detail;
  std::string s = stream_header.get("header_detail","").asString();
  {
    AutoMutex lock(m_cond.mutex());
    const std::string& expected = m_header_detail_str.value();
    if (s != expected)
      THROW_HW_ERROR(Error) << "Error: got" << s << ", " << DEB_VAR1(expected);
    header_detail = m_header_detail;
  }
  int nb_parts;
  int header_message_id;
  switch (header_detail) {
  case OFF:
    nb_parts = 1;
    header_message_id = 0;
    break;
  case BASIC:
    nb_parts = 2;
    header_message_id = 1;
    break;
  case ALL:
    nb_parts = 8;
    header_message_id = 1;
    break;
  }
  int nb_messages = pending_messages.size();
  if (nb_messages < nb_parts)
    THROW_HW_ERROR(Error) << "Invalid " << DEB_VAR1(nb_messages)
			  << " for header_detail=" << s;
    
  return _get_json_header(pending_messages[header_message_id]);
}

Stream::Stream(Camera& cam) : 
  m_cam(cam),
  m_header_detail(OFF),
  m_state(Init),
  m_buffer_ctrl_obj(new Stream::_BufferCtrlObj(*this))
{
  DEB_CONSTRUCTOR();

  bool is_le = (htole16(0x1234) == 0x1234);
  m_endianess = (is_le ? '<' : '>');

  m_buffer_mgr = &m_buffer_ctrl_obj->getBuffer();

  m_active = _getStreamMode();
  getEigerParam(m_cam,Requests::STREAM_HEADER_DETAIL,m_header_detail_str);
  DEB_TRACE() << DEB_VAR1(m_header_detail_str.value());

  m_zmq_context = zmq_ctx_new();
  if(pipe(m_pipes))
    THROW_HW_ERROR(Error) << "Can't open pipe";

  pthread_create(&m_thread_id,NULL,_runFunc,this);

  AutoMutex lock(m_cond.mutex());
  while (m_state != Idle)
    m_cond.wait();
}

Stream::~Stream()
{
  DEB_DESTRUCTOR();

  {
    AutoMutex aLock(m_cond.mutex());
    DEB_TRACE() << "Quitting";
    m_state = Quitting;
    m_cond.broadcast();
    _send_synchro();
  }

  if(m_thread_id > 0)
    pthread_join(m_thread_id,NULL);

  close(m_pipes[0]),close(m_pipes[1]);
  zmq_ctx_destroy(m_zmq_context);
}

void Stream::_setStreamMode(bool enabled)
{
  DEB_MEMBER_FUNCT();
  DEB_PARAM() << DEB_VAR1(enabled);
  std::string enabled_str = enabled ? "enabled" : "disabled";
  DEB_TRACE() << "STREAM_MODE:" << DEB_VAR1(enabled_str);
  setEigerParam(m_cam,Requests::STREAM_MODE,enabled_str);
}

bool Stream::_getStreamMode()
{
  DEB_MEMBER_FUNCT();
  std::string enabled_str;
  getEigerParam(m_cam,Requests::STREAM_MODE,enabled_str);
  DEB_TRACE() << "STREAM_MODE:" << DEB_VAR1(enabled_str);
  bool enabled = (enabled_str == "enabled");
  DEB_RETURN() << DEB_VAR1(enabled);
  return enabled;
}

void Stream::start()
{
  DEB_MEMBER_FUNCT();
  AutoMutex aLock(m_cond.mutex());
  if (m_state != Armed)
    THROW_HW_ERROR(Error) << "Stream is not Armed (no global header)";
  DEB_TRACE() << "Running";
  m_state = Running;
  m_buffer_mgr->setStartTimestamp(Timestamp::now());
}

void Stream::stop()
{
  DEB_MEMBER_FUNCT();
  AutoMutex aLock(m_cond.mutex());
  bool connected = (m_state == Connected);
  if (!_isRunning() || connected) {
    if (connected)
      _abort();
    return;
  }
  DEB_TRACE() << "Stopped";
  m_state = Stopped;
  _send_synchro();
  while (_isRunning())
    m_cond.wait();
}

void Stream::abort()
{
  DEB_MEMBER_FUNCT();
  AutoMutex aLock(m_cond.mutex());
  _abort();
}

void Stream::_send_synchro()
{
  DEB_MEMBER_FUNCT();

  if(write(m_pipes[1],"|",1) == -1)
    DEB_ERROR() << "Something wrong happened!";
}

void Stream::_abort()
{
  DEB_MEMBER_FUNCT();

  if(m_state == Failed) {
    m_state = Idle;
    THROW_HW_ERROR(Error) << "Stream failed";
  } else if(!_isRunning())
    return;

  DEB_TRACE() << "Aborting";
  m_state = Aborting;
  _send_synchro();
  m_cond.broadcast();
  while(_isRunning())
    m_cond.wait();
}

bool Stream::isRunning() const
{
  DEB_MEMBER_FUNCT();
  AutoMutex aLock(m_cond.mutex());
  DEB_TRACE() << DEB_VAR1(m_state);
  bool running = _isRunning();
  DEB_RETURN() << DEB_VAR1(running);
  return running;
}

void Stream::getHeaderDetail(Stream::HeaderDetail& detail) const
{
  AutoMutex lock(m_cond.mutex());
  detail = m_header_detail;
}

void Stream::setHeaderDetail(Stream::HeaderDetail detail)
{
  AutoMutex lock(m_cond.mutex());
  m_header_detail = detail;
}

void Stream::setActive(bool active)
{
  DEB_MEMBER_FUNCT();
  DEB_PARAM() << DEB_VAR1(active);

  AutoMutex lock(m_cond.mutex());
  DEB_TRACE() << DEB_VAR2(m_active.value(), m_state);

  bool is_ready = ((m_state == Connected) || (m_state == Armed));
  bool do_abort = (!active && is_ready);
  if(!do_abort && _isRunning()) {
    DEB_WARNING() << "Stream is Running: Aborting!";
    do_abort = true;
  }
  if(do_abort) {
    _abort();
    is_ready = false;
  }

  if(active) {
    std::string s;
    switch(m_header_detail) {
    case ALL:
      s = "all";break;
    case BASIC:
      s = "basic";break;
    default:
      s = "none";
    }
    if(m_header_detail_str.changed(s)) {
      DEB_TRACE() << "STREAM_HEADER_DETAIL:" << DEB_VAR1(m_header_detail_str.value());
      setEigerParam(m_cam,Requests::STREAM_HEADER_DETAIL,m_header_detail_str);
    }
  }

  if(m_active.changed(active))
    _setStreamMode(m_active);

  if(!m_active || is_ready)
    return;
  
  m_state = Starting;
  m_cond.broadcast();
  while(m_state == Starting)
    m_cond.wait();
  
  if (m_state == Failed) {
    m_state = Idle;
    THROW_HW_ERROR(Error) << "Error starting stream";
  } else if (m_state != Connected) {
    THROW_HW_ERROR(Error) << "Internal error: " << DEB_VAR1(m_state);
  }
}

void Stream::waitArmed(double timeout)
{
  DEB_MEMBER_FUNCT();
  DEB_PARAM() << DEB_VAR1(timeout);
  AutoMutex lock(m_cond.mutex());
  Timestamp t0 = Timestamp::now();
  DEB_TRACE() << DEB_VAR1(m_state);
  while(m_state == Connected) {
    double elapsed = Timestamp::now() - t0;
    if (elapsed >= timeout)
      break;
    m_cond.wait(timeout - elapsed);
  }
  if (m_state == Failed)
    m_state = Idle;
  if (m_state != Armed)
    THROW_HW_ERROR(Error) << "Global header not received";
}

HwBufferCtrlObj* Stream::getBufferCtrlObj()
{
  DEB_MEMBER_FUNCT();
  return m_buffer_ctrl_obj.get();
}

void* Stream::_runFunc(void *streamPt)
{
  ((Stream*)streamPt)->_run();
  return NULL;
}

void Stream::_run()
{
  DEB_MEMBER_FUNCT();
  
  AutoMutex lock(m_cond.mutex());
  m_state = Idle;
  while (1) {
    DEB_TRACE() << "Wait";
    m_cond.broadcast();

    while((m_state == Idle) || (m_state == Failed))
      m_cond.wait();
    if(m_state == Quitting)
      break;
    else if(m_state == Starting)
      DEB_TRACE() << "Running: " << DEB_VAR1(m_state);
    else
      DEB_ERROR() << "Invalid " << DEB_VAR1(m_state);

    try {
      {
	AutoMutexUnlock u(lock);
	_run_sequence();
      }
      m_state = Idle;
    } catch (Exception& e) {
      m_state = Failed;
    }
  }
}

void Stream::_run_sequence()
{
  DEB_MEMBER_FUNCT();

  //create stream socket
  void *stream_socket = zmq_socket(m_zmq_context,ZMQ_PULL);
  if (!stream_socket)
    THROW_HW_ERROR(Error) << "Could not create zmq_socket";

  struct zmq_socket_deleter {
    void operator()(void *s) { zmq_close(s); }
  };
  std::unique_ptr<void, zmq_socket_deleter> socket_ptr(stream_socket);

  char stream_endpoint[256];
  snprintf(stream_endpoint,sizeof(stream_endpoint),
	   "tcp://%s:9999",m_cam.getDetectorIp().c_str());
  if(zmq_connect(stream_socket,stream_endpoint) != 0) {
    char error_buffer[256];
    const char *error_msg = strerror_r(errno,error_buffer,sizeof(error_buffer));
    THROW_HW_ERROR(Error) << "Connection error to " << stream_endpoint << ": "
			  << DEB_VAR2(errno, error_msg);
  }

  {
    AutoMutex lock(m_cond.mutex());
    TrigMode trigger_mode;
    m_cam.getTrigMode(trigger_mode);
    m_ext_trigger = ((trigger_mode != IntTrig) &&
		     (trigger_mode != IntTrigMult));
    m_cam.getCompressionType(m_comp_type);

    DEB_TRACE() << "Connected to " << stream_endpoint;
    m_state = Connected;
    m_cond.broadcast();
  }

  m_stopped = false;
  m_waiting_global_header = true;

  //  Initialize poll set
  zmq_pollitem_t items [] = {
    { NULL, m_pipes[0], ZMQ_POLLIN, 0 },
    { stream_socket, 0, ZMQ_POLLIN, 0 }
  };

  bool continue_flag = true;
  while(continue_flag) {	// reading loop
    DEB_TRACE() << "Enter poll";
    zmq_poll(items,2,-1);
    DEB_TRACE() << "Exit poll";

    if(items[0].revents & ZMQ_POLLIN) { // reading synchro pipe
      char buffer[1024];
      if(read(m_pipes[0],buffer,sizeof(buffer)) == -1)
	DEB_WARNING() << "Something strange happened!";

      {
	AutoMutex lock(m_cond.mutex());
	continue_flag = !((m_state == Aborting) || (m_state == Quitting));
	m_stopped = (m_state == Stopped);
      }
      if (!continue_flag)
	break;
    }

    if(items[1].revents & ZMQ_POLLIN) { // reading stream
      try {
	continue_flag = _read_zmq_messages(stream_socket);
      } catch (Exception& e) {
	std::ostringstream err_msg;
	err_msg << "Stream error: " << e.getErrMsg();
	Event::Code err_code = Event::CamFault;
	Event *event = new Event(Hardware, Event::Error, Event::Camera,
				 err_code, err_msg.str());
	DEB_EVENT(*event) << DEB_VAR1(*event);
 	m_cam.reportEvent(event);
      }
    }
  }
}

bool Stream::_read_zmq_messages(void *stream_socket)
{
  DEB_MEMBER_FUNCT();

  MessageList pending_messages;
  pending_messages.reserve(9);
  int more;
  do {
    MessagePtr msg(new Stream::Message());
    zmq_msg_t *zmq_msg = msg->get_msg();
    int ret = zmq_msg_recv(zmq_msg,stream_socket,0);
    if (ret == -1) {
      if (errno == EAGAIN) {
	  DEB_TRACE() << "zmq EAGAIN";
	  break;
      }
      char errno_buffer[256];
      char *errno_msg = strerror_r(errno,errno_buffer,sizeof(errno_buffer));
      THROW_HW_ERROR(Error)
	<< "Error receiving zmq message: "
	<< DEB_VAR3(errno, errno_msg, pending_messages.size());
    }
    more = zmq_msg_more(zmq_msg);
    pending_messages.emplace_back(msg);
  } while(more);

  int nb_messages = pending_messages.size();
  DEB_TRACE() << DEB_VAR1(nb_messages);
  if (nb_messages == 0)
    return true;

  Timestamp data_rx_tstamp = Timestamp::now();

  Json::Value stream_header = _get_json_header(pending_messages[0]);
  std::string htype = stream_header.get("htype","").asString();
  DEB_TRACE() << DEB_VAR1(htype);

  bool is_global_header = (htype.find("dheader-") != std::string::npos);
  if (is_global_header != m_waiting_global_header) {
    DEB_WARNING() << "Global header mismatch: "
		  << DEB_VAR2(is_global_header, m_waiting_global_header)
		  << ": " << htype;
    return true;
  } else if (is_global_header) {
    Json::Value header = _get_global_header(stream_header,pending_messages);
    m_waiting_global_header = false;
    AutoMutex lock(m_cond.mutex());
    m_state = Armed;
    DEB_TRACE() << "Global header received: " << DEB_VAR1(m_state);
    m_cond.broadcast();
    return true;
  } else if(htype.find("dimage-") != std::string::npos) {
    int frameid = stream_header.get("frame",-1).asInt();
    DEB_TRACE() << DEB_VAR1(frameid);
    //stream_header.get("hash","md5sum")
    if (nb_messages < 3)
      THROW_HW_ERROR(Error) << "Should receive at least 3 messages part, "
			    << "only received " << nb_messages;

    Json::Value data_header = _get_json_header(pending_messages[1]);
    std::string dtype = data_header.get("type","none").asString();
    if (frameid == 0) {
      //Data size (width,height)
      Json::Value shape = data_header.get("shape","");
      if (!shape.isArray() || shape.size() != 2)
	THROW_HW_ERROR(Error) << "Invalid data shape: " << shape.asString();
      FrameDim anImageDim;
      anImageDim.setSize(Size(shape[0u].asInt(),shape[1u].asInt()));
      //data type
      ImageType image_type;
      if(dtype == "int32")
	image_type = Bpp32S;
      else if(dtype == "uint32")
	image_type = Bpp32;
      else if(dtype == "int16")
	image_type = Bpp16S;
      else if(dtype == "uint16")
	image_type = Bpp16;
      else if(dtype == "int8")
	image_type = Bpp8S;
      else if(dtype == "uint8")
	image_type = Bpp8;
      else
	THROW_HW_ERROR(Error) << "Invalid " << DEB_VAR1(dtype);
      anImageDim.setImageType(image_type);
      DEB_TRACE() << DEB_VAR1(anImageDim);
      m_dtype_str = dtype;
      m_depth = anImageDim.getDepth();

      AutoMutex lock(m_cond.mutex());
      m_last_info.encoding = data_header.get("encoding", "").asString();
      m_last_info.frame_dim = anImageDim;
      m_last_info.packed_size = data_header.get("size", "-1").asInt();
      _checkCompression(m_last_info);
    } else if (dtype != m_dtype_str)
      THROW_HW_ERROR(Error) << "Invalid " << DEB_VAR1(dtype) << ", "
			    << "expected " << DEB_VAR1(m_dtype_str);

    Json::Value config_header = _get_json_header(pending_messages[3]);
    if (frameid == 0)
      DEB_TRACE() << DEB_VAR1(config_header["start_time"].asString());

    if (m_stopped) {
      DEB_TRACE() << "Stopped: ignoring data";
      return true;
    }

    HwFrameInfoType frame_info;
    frame_info.acq_frame_nb = frameid;
    void* buffer_ptr = m_buffer_mgr->getFrameBufferPtr(frameid);
    int data_size = data_header.get("size",-1).asInt();
    {
      AutoMutex lock(m_cond.mutex());
      m_data_2_msg[buffer_ptr] = ImageData{pending_messages[2], m_depth,
					   m_comp_type};
    }
    {
      AutoMutex stat_lock(m_stat_lock);
      if (frameid > 0) {
	double transfer_time = data_rx_tstamp - m_last_data_tstamp;
	m_stat.add(data_size, transfer_time);
      }
      m_last_data_tstamp = data_rx_tstamp;
    }

    m_cam.newFrameAcquired();
    bool continue_flag = m_buffer_mgr->newFrameReady(frame_info);
    bool do_disarm = (m_ext_trigger && m_cam.allFramesAcquired());
    if (!continue_flag && !do_disarm) {
      DEB_WARNING() << "Unexpected " << DEB_VAR1(continue_flag) << ": "
		    << "Disarming camera";
      do_disarm = true;
    }
    if (do_disarm)
      m_cam.disarm();
    return true;
  } else if (htype.find("dseries_end-") != std::string::npos) {
    DEB_TRACE() << "Finishing";
    return false;
  } else {
    DEB_WARNING() << "Unknown header: " << htype;
    return true;
  }
}

void Stream::_checkCompression(const StreamInfo& info)
{
  DEB_MEMBER_FUNCT();
  DEB_PARAM() << DEB_VAR2(info, m_endianess);

  const std::string& encoding = info.encoding;

  char endianess = *encoding.rbegin();
  if (endianess != m_endianess)
    THROW_HW_ERROR(Error) << "Endianess mismatch: "
			  << "got " << endianess << ", "
			  << "expected " << m_endianess;

  CompressionType comp_type;
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
  DEB_TRACE() << DEB_VAR1(comp_type);

  if (comp_type != m_comp_type)
    THROW_HW_ERROR(Error) << "Unexpected compression type: " << comp_type;
}

void Stream::getLastStreamInfo(StreamInfo& last_info)
{
  DEB_MEMBER_FUNCT();
  last_info = m_last_info;
  DEB_RETURN() << DEB_VAR1(last_info);
}

Stream::ImageData Stream::get_msg(void* aDataBuffer)
{
  DEB_MEMBER_FUNCT();
  DEB_PARAM() << DEB_VAR1(aDataBuffer);

  AutoMutex lock(m_cond.mutex());
  Data2Message::iterator it = m_data_2_msg.find(aDataBuffer);
  if(it == m_data_2_msg.end())
    THROW_HW_ERROR(Error) << "Can't find image_data message";
  ImageData img_data = it->second;
  m_data_2_msg.erase(it);
  lock.unlock();
  if (DEB_CHECK_ANY(DebTypeReturn))
    DEB_RETURN() << DEB_VAR1(img_data);
  return img_data;
}

void Stream::release_all_msgs()
{
  DEB_MEMBER_FUNCT();

  AutoMutex lock(m_cond.mutex());
  m_data_2_msg.clear();
}

void Stream::resetStatistics()
{
  DEB_MEMBER_FUNCT();
  AutoMutex stat_lock(m_stat_lock);
  m_stat.reset();
}

void Stream::latchStatistics(StreamStatistics& stat, bool reset)
{
  DEB_MEMBER_FUNCT();
  AutoMutex stat_lock(m_stat_lock);
  stat = m_stat;
  if (reset)
    m_stat.reset();
  DEB_RETURN() << DEB_VAR1(stat);
}
