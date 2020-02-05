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

template <typename C>
void Camera::_sendCommand(C cmd, DebObj *deb_ptr)
{
  DEB_FROM_PTR(deb_ptr);
  CommandReq req = m_requests->get_command(cmd);
  try {
    req->wait();
  } catch(const eigerapi::EigerException &e) {
    m_requests->cancel(req);
    HANDLE_EIGERERROR(req, e);
  }
}

template <typename P, typename T>
void Camera::_setParam(P param, T value, DebObj *deb_ptr)
{
  DEB_FROM_PTR(deb_ptr);
  ParamReq req = m_requests->set_param(param, value);
  try {
    req->wait();
  } catch(const eigerapi::EigerException &e) {
    HANDLE_EIGERERROR(req, e);
  }
}    

template <typename P, typename T>
void Camera::_getParam(P param, T& value, DebObj *deb_ptr)
{
  DEB_FROM_PTR(deb_ptr);
  ParamReq req = m_requests->get_param(param, value);
  try {
    req->wait();
  } catch(const eigerapi::EigerException &e) {
    m_requests->cancel(req);
    HANDLE_EIGERERROR(req, e);
  }
}

template <typename R>
void Camera::_startTransfer(std::string src_file_name,
			    std::string dest_path,
			    AutoMutex& lock, R& req,
			    DebObj *deb_ptr)
{
  DEB_FROM_PTR(deb_ptr);

  try {
    req = m_requests->start_transfer(src_file_name, dest_path);
  } catch(eigerapi::EigerException& e) {
    Event *event = new Event(Hardware,Event::Error, Event::Saving,
			     Event::SaveOpenError,e.what());
    AutoMutexUnlock u(lock);
    reportEvent(event);
    req.reset();
  }
}


#define sendEigerCommand(cam, cmd)		\
  (cam)._sendCommand(cmd, DEB_PTR())

#define setEigerParam(cam, param, value)	\
  (cam)._setParam(param, value, DEB_PTR())

#define getEigerParam(cam, param, value)	\
  (cam)._getParam(param, value, DEB_PTR())

#define startEigerTransfer(cam, src_file_name, dest_path, lock, req)	\
  (cam)._startTransfer(src_file_name, dest_path, lock, req, DEB_PTR())


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

  template <typename T>
  void addSet(Name name, const T& var)
  { add(name, m_cam.m_requests->set_param(name, var)); }

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
