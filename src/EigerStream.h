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
#ifndef EIGERSTREAM_H
#define EIGERSTREAM_H

#include "lima/Debug.h"

#include "EigerCamera.h"
#include "EigerStreamInfo.h"
#include "lima/HwBufferMgr.h"

#include <json/json.h>

namespace lima
{
  namespace Eiger
  {
    class Stream
    {
      DEB_CLASS_NAMESPC(DebModCamera,"Stream","Eiger");
    public:
      class Message;
      typedef std::shared_ptr<Message> MessagePtr;
      typedef Camera::CompressionType CompressionType;

      enum HeaderDetail {ALL,BASIC,OFF};
      enum State {Init,Idle,Starting,Connected,Failed,Armed,Running,
		  Stopped,Aborting,Quitting};

      struct ImageData {
	MessagePtr msg;
	int depth;
	CompressionType comp_type;
	void getMsgDataNSize(void*& data, size_t& size) const;
      };

      Stream(Camera&);
      ~Stream();

      void start();
      void stop();
      void abort();
      bool isRunning() const;
      void waitArmed(double timeout);

      void getHeaderDetail(HeaderDetail&) const;
      void setHeaderDetail(HeaderDetail);
      
      void setActive(bool);
      bool isActive() const;

      HwBufferCtrlObj* getBufferCtrlObj();

      ImageData get_msg(void* aDataBuffer);
      void release_all_msgs();

      void getLastStreamInfo(StreamInfo& info);

    private:
      class _BufferCtrlObj;
      friend class _BufferCtrlObj;
      friend class ImageDataPtr;

      typedef std::map<void*,ImageData> Data2Message;
      typedef std::vector<MessagePtr> MessageList;

      bool _isRunning() const;

      static void* _runFunc(void*);
      void _run();
      void _run_sequence();
      Json::Value _get_global_header(const Json::Value& stream_header,
				     MessageList& pending_messages);
      Json::Value _get_json_header(MessagePtr &msg);
      bool _read_zmq_messages(void *stream_socket);
      void _send_synchro();
      void _abort();
      void _checkCompression(const StreamInfo& info);

      void _setStreamMode(bool enabled);
      bool _getStreamMode();

      Camera&		m_cam;
      char		m_endianess;
      bool		m_active;
      HeaderDetail	m_header_detail;
      std::string	m_header_detail_str;
      bool		m_dirty_flag;
      State		m_state;

      mutable Cond	m_cond;

      pthread_t		m_thread_id;
      void*		m_zmq_context;
      int		m_pipes[2];
      Data2Message	m_data_2_msg;
      StreamInfo	m_last_info;
      bool		m_ext_trigger;
      CompressionType	m_comp_type;

      std::unique_ptr<_BufferCtrlObj>	m_buffer_ctrl_obj;
      StdBufferCbMgr*			m_buffer_mgr;
    };

    std::ostream& operator <<(std::ostream& os, Stream::State state);
    std::ostream& operator <<(std::ostream& os,
			      const Stream::ImageData& img_data);
  }
}
#endif	// EIGERSTREAM_H
