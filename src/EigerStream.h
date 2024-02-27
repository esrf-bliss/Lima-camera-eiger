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
#ifndef EIGERSTREAM_H
#define EIGERSTREAM_H

#include "lima/Debug.h"

#include "EigerCamera.h"
#include "EigerStreamInfo.h"
#include "lima/HwBufferMgr.h"

#include "EigerStatistics.h"

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

      struct ImageData : public sideband::Data {
	MessagePtr msg;
	FrameDim decomp_fdim;
	CompressionType comp_type;

	ImageData(MessagePtr m,	FrameDim d, CompressionType c)
	  : msg(m), decomp_fdim(d), comp_type(c) {}

	void getMsgDataNSize(void*& data, size_t& size) const;
      };

      Stream(Camera&,const char* mmap_file=NULL);
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

      void getLastStreamInfo(StreamInfo& info);

      void resetStatistics();
      void latchStatistics(StreamStatistics& stat, bool reset=false);

    private:
      class _ZmqThread;
      friend class _ZmqThread;

      typedef std::vector<MessagePtr> MessageList;

      template <typename T>
      using Cache = Camera::Cache<T>;

      bool _isRunning() const;

      void _send_synchro();
      void _abort();

      void _setStreamMode(bool enabled);
      bool _getStreamMode();

      Camera&		m_cam;
      mutable Cond	m_cond;
      bool		m_active;
      Cache<std::string> m_mode_str;
      State		m_state;
      HeaderDetail	m_header_detail;
      Cache<std::string> m_header_detail_str;

      int		m_pipes[2];
      StreamInfo	m_last_info;

      std::unique_ptr<_ZmqThread>		m_thread;

      BufferAllocMgr*                           m_buffer_alloc_mgr;
      SoftBufferCtrlObj*                        m_buffer_ctrl_obj;
      StdBufferCbMgr*				m_buffer_mgr;
      SoftBufferCtrlObj::Sync*			m_buffer_sync;

      Mutex             m_stat_lock;
      StreamStatistics	m_stat;
    };

    std::ostream& operator <<(std::ostream& os, Stream::State state);
    std::ostream& operator <<(std::ostream& os,
			      const Stream::ImageData& img_data);
  }
}
#endif	// EIGERSTREAM_H
