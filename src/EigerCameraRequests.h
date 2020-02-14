//###########################################################################
// This file is part of LImA, a Library for Image Acquisition
//
// Copyright (C) : 2009-2014
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
#ifndef EIGERCAMERAREQUESTS_H
#define EIGERCAMERAREQUESTS_H

#include <eigerapi/Requests.h>
#include "EigerCamera.h"

namespace lima
{
namespace Eiger
{

typedef eigerapi::Requests::CommandReq CommandReq;
typedef eigerapi::Requests::ParamReq ParamReq;
typedef eigerapi::Requests::TransferReq TransferReq;
typedef eigerapi::CurlLoop::FutureRequest::Callback Callback;
typedef eigerapi::CurlLoop::FutureRequest::CallbackPtr CallbackPtr;


#define HANDLE_EIGERERROR(req, e) {					\
    THROW_HW_ERROR(Error) << (req)->get_url() << ":" << (e).what();	\
}

/*----------------------------------------------------------------------------
			    class Camera request helpers
 ----------------------------------------------------------------------------*/
class CameraRequest
{
public:
  CameraRequest(Camera& cam) : m_cam(cam), m_requests(m_cam.m_requests) {}

  template <typename A>
  CommandReq doSendCommandTimeout(eigerapi::Requests::COMMAND_NAME cmd,
				  double timeout, DebObj *deb_ptr, A ack)
  {
    DEB_FROM_PTR(deb_ptr);
    CommandReq req = m_requests->get_command(cmd);
    try {
      req->wait(timeout);
    } catch(const eigerapi::EigerException &e) {
      m_requests->cancel(req);
      HANDLE_EIGERERROR(req, e);
    }
    ack.succeeded();
    return req;
  }

  template <typename T, typename A>
  ParamReq doSetParam(eigerapi::Requests::PARAM_NAME param, T value,
		      DebObj *deb_ptr, A ack)
  {
    DEB_FROM_PTR(deb_ptr);
    ParamReq req = m_requests->set_param(param, value);
    try {
      req->wait();
    } catch(const eigerapi::EigerException &e) {
      HANDLE_EIGERERROR(req, e);
    }
    ack.succeeded();
    return req;
  }

  template <typename T, typename A>
  ParamReq doGetParam(eigerapi::Requests::PARAM_NAME param, T& value,
		      DebObj *deb_ptr, A ack)
  {
    DEB_FROM_PTR(deb_ptr);
    ParamReq req = m_requests->get_param(param, value);
    try {
      req->wait();
    } catch(const eigerapi::EigerException &e) {
      m_requests->cancel(req);
      HANDLE_EIGERERROR(req, e);
    }
    ack.succeeded();
    return req;
  }

  template <typename A>
  TransferReq doStartTransfer(std::string src_file_name, std::string dest_path,
			      AutoMutex& lock, DebObj *deb_ptr, A ack)
  {
    DEB_FROM_PTR(deb_ptr);
    TransferReq req;
    try {
      req = m_requests->start_transfer(src_file_name, dest_path);
    } catch(eigerapi::EigerException& e) {
      Event *event = new Event(Hardware,Event::Error, Event::Saving,
			       Event::SaveOpenError,e.what());
      AutoMutexUnlock u(lock);
      m_cam.reportEvent(event);
      req.reset();
    }
    ack.succeeded();
    return req;
  }

public:
  Camera& m_cam;
  eigerapi::Requests *m_requests;
};


#define sendEigerCommand(cam, cmd)			\
  sendEigerCommandTimeout(cam, cmd, eigerapi::CurlLoop::FutureRequest::TIMEOUT)

#define sendEigerCommandTimeout(cam, cmd, timeout)	\
  CameraRequest(cam).doSendCommandTimeout(cmd, timeout, DEB_PTR(), SuccessAck())

#define setEigerParam(cam, param, value)		\
  CameraRequest(cam).doSetParam(param, value, DEB_PTR(), SuccessAck())

#define setEigerCachedParam(cam, param, cache, value)	\
  setEigerCachedParamForce(cam, param, cache, value, false)

#define setEigerCachedParamForce(cam, param, cache, value, force)	\
  do {									\
    auto change_info(cache.change(value));				\
    if (change_info || force)						\
      CameraRequest(cam).doSetParam(param, value, DEB_PTR(), change_info); \
  } while (0)

#define getEigerParam(cam, param, value)		\
  CameraRequest(cam).doGetParam(param, value, DEB_PTR(), SuccessAck())

#define startEigerTransfer(cam, src_file_name, dest_path, lock)		\
  CameraRequest(cam).doStartTransfer(src_file_name, dest_path, lock, DEB_PTR(),\
				     SuccessAck())


/*----------------------------------------------------------------------------
			    class MultiParamRequest
 ----------------------------------------------------------------------------*/

class MultiParamRequest
{
  DEB_CLASS_NAMESPC(DebModCamera, "MultiParamRequest", "Eiger");

public:
  typedef eigerapi::Requests::PARAM_NAME Name;

  MultiParamRequest(Camera& cam)
    : m_cam(cam), m_parallel(m_cam.m_api == Camera::Eiger1)
  {}

  void addGet(Name name)
  { add(name, m_cam.m_requests->get_param(name)); }

  template <typename T>
  void addGet(Name name, T& var)
  { add(name, m_cam.m_requests->get_param(name, var)); }

  void wait()
  {
    if (m_parallel) {
      RequestList::const_iterator it, end = m_list.end();
      for (it = m_list.begin(); it != end; ++it)
	wait(*it);
    }
  }

  ParamReq operator [](Name name)
  { return m_map[name]; }

private:
  typedef std::list<ParamReq> RequestList;
  typedef std::map<Name, ParamReq> RequestMap;

  void add(Name name, ParamReq req)
  {
    m_list.push_back(req);
    m_map[name] = req;

    if (!m_parallel)
      wait(req);
  }

  void wait(ParamReq req)
  {
    DEB_MEMBER_FUNCT();

    try {
      req->wait();
    } catch (const eigerapi::EigerException &e) {
      RequestList::const_iterator it, end = m_list.end();
      for (it = m_list.begin(); it != end; ++it)
	m_cam.m_requests->cancel(*it);
      HANDLE_EIGERERROR(req, e);
    }
  }
  
  Camera& m_cam;
  bool m_parallel;
  RequestList m_list;
  RequestMap m_map;
};

} // namespace Eiger
} // namespace lima

#endif // EIGERCAMERAREQUESTS_H
