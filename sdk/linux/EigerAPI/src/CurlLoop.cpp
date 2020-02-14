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
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <sys/select.h>

#include "eigerapi/CurlLoop.h"
#include "eigerapi/EigerDefines.h"
#include "AutoMutex.h"

using namespace eigerapi;

// Constant
double const CurlLoop::FutureRequest::TIMEOUT = 15;

struct CURL_INIT
{
  CURL_INIT()
  {
    curl_global_init(CURL_GLOBAL_ALL);
  }
  ~CURL_INIT()
  {
    curl_global_cleanup();
  }
} global_init;


// -------------------- CurlLoop::Request --------------------

CurlLoop::FutureRequest::FutureRequest(const std::string& url) :
  m_status(IDLE),
  m_cbk(NULL),
  m_url(url)
{
  if(pthread_mutex_init(&m_lock,NULL))
    THROW_EIGER_EXCEPTION("pthread_mutex_init","Can't initialize the lock");
  if(pthread_cond_init(&m_cond,NULL))
    THROW_EIGER_EXCEPTION("pthread_cond_init",
			  "Can't initialize the variable condition");
  m_handle = curl_easy_init();
  curl_easy_setopt(m_handle,CURLOPT_URL, url.c_str());
  curl_easy_setopt(m_handle, CURLOPT_PROXY, "");
#ifdef DEBUG
  curl_easy_setopt(m_handle, CURLOPT_VERBOSE, 1L);
#endif
}

CurlLoop::FutureRequest::~FutureRequest()
{
  curl_easy_cleanup(m_handle);
  delete m_cbk;

  pthread_cond_destroy(&m_cond);
  pthread_mutex_destroy(&m_lock);
}

inline void CurlLoop::FutureRequest::handle_result(CURLcode result)
{
  Lock lock(&m_lock);
  if(m_status != FutureRequest::CANCEL)
    {
      switch(result)
	{
	case CURLE_OK:
	  m_status = FutureRequest::OK;
	  break;
	default: // error
	  m_status = FutureRequest::ERROR;
	  m_error_code = curl_easy_strerror(result);
	  break;
	}
    }
  pthread_cond_broadcast(&m_cond);
  if(m_cbk)
    _status_changed();
  _request_finished();
}

void CurlLoop::FutureRequest::wait(double timeout,bool lock_flag) const
{
  Lock lock(&m_lock,lock_flag);

  if(m_status == IDLE)		// weird init
    THROW_EIGER_EXCEPTION("weird initialization","status == IDLE");

  int retcode = 0;
  if(timeout >= 0.)
    {
      struct timeval now;
      struct timespec waitTimeout;
      gettimeofday(&now,NULL);
      waitTimeout.tv_sec = now.tv_sec + long(timeout);
      waitTimeout.tv_nsec = ((now.tv_usec * 1000) +
			     long((timeout - long(timeout)) * 1e9));
      if(waitTimeout.tv_nsec >= 1000000000L) // Carry
	++waitTimeout.tv_sec,waitTimeout.tv_nsec -= 1000000000L;

      while(retcode != ETIMEDOUT && m_status == RUNNING)
	retcode = pthread_cond_timedwait(&m_cond,&m_lock,&waitTimeout);

      if(retcode == ETIMEDOUT)
	THROW_EIGER_EXCEPTION("wait_status","timeout");
    }
  else
    while(m_status == RUNNING)
      pthread_cond_wait(&m_cond,&m_lock);

  if(m_status == ERROR)
    THROW_EIGER_EXCEPTION("wait_status",m_error_code.c_str());
}

CurlLoop::FutureRequest::Status
CurlLoop::FutureRequest::get_status() const
{
  Lock lock(&m_lock);
  return m_status;
}

struct StatusChangedCallback {
  CurlLoop::FutureRequest::CallbackPtr cbk;
  CurlLoop::FutureRequest::Status status;
  std::string error;
  void fire() { cbk->status_changed(status, error); }
};
typedef std::unique_ptr<StatusChangedCallback> StatusChangedCallbackPtr;

inline void CurlLoop::FutureRequest::_status_changed()
{
  StatusChangedCallback cbk{*m_cbk, m_status, m_error_code};
  if (!m_cbk_in_thread) {
    cbk.fire();
    return;
  }

#define ERROR(x)						\
  THROW_EIGER_EXCEPTION("CurlLoop::FutureRequest::_status_changed", x)

  StatusChangedCallbackPtr tcbk(new StatusChangedCallback(cbk));
  pthread_t thread;
  pthread_attr_t attr;
  if (pthread_attr_init(&attr))
    ERROR("could not init thread attr");
  else if (pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED))
    ERROR("could not set thread attr detached state");
  else if (pthread_create(&thread, &attr, _callback_thread_runFunc, tcbk.get()))
    ERROR("could not create thread");
  else
    tcbk.release();

#undef ERROR
}

void *CurlLoop::FutureRequest::_callback_thread_runFunc(void *data)
{
  StatusChangedCallbackPtr cbk((StatusChangedCallback *) data);
  cbk->fire();
  return NULL;
}

void CurlLoop::FutureRequest::register_callback(CallbackPtr& cbk,
						bool in_thread)
{
  if (!cbk)
    THROW_EIGER_EXCEPTION("register_callback","invalid callback");

  Lock lock(&m_lock);
  delete m_cbk;
  m_cbk = new CallbackPtr(cbk);
  m_cbk_in_thread = in_thread;
  if(m_status != RUNNING)
    _status_changed();
}


// ----------------- CurlLoop::ActiveCurlRequest  ------------

struct CurlLoop::ActiveCurlRequest
{
  CurlReq req;
  CURLM *multi_handle;

  ActiveCurlRequest(CurlReq r, CURLM *mh)
    : req(r), multi_handle(mh)
  { curl_multi_add_handle(multi_handle, req->get_handle()); }

  ~ActiveCurlRequest()
  { curl_multi_remove_handle(multi_handle, req->get_handle()); };
};


// ------------------------ CurlLoop -------------------------

CurlLoop::CurlLoop() :
  m_multi_handle(NULL),
  m_quit(false),
  m_thread_id(0)
{
  if(pthread_mutex_init(&m_lock,NULL))
    THROW_EIGER_EXCEPTION("pthread_mutex_init","Can't initialize the lock");
  if(pthread_cond_init(&m_cond,NULL))
    THROW_EIGER_EXCEPTION("pthread_cond_init",
			  "Can't initialize the variable condition");

  m_multi_handle = curl_multi_init();

  if(pipe(m_pipes))
    THROW_EIGER_EXCEPTION("pipe","Can't create pipe");
  
  //set pipe none blocking
  fcntl(m_pipes[0],F_SETFL,O_NONBLOCK);
  fcntl(m_pipes[1],F_SETFL,O_NONBLOCK);

  pthread_create(&m_thread_id,NULL,_runFunc,this);
}

CurlLoop::~CurlLoop()
{
  quit();

  curl_multi_cleanup(m_multi_handle);

  pthread_cond_destroy(&m_cond);
  pthread_mutex_destroy(&m_lock);

}

void CurlLoop::quit()
{
  if(!m_thread_id) return;

  Lock alock(&m_lock);
  m_quit = true;
  write(m_pipes[1],"|",1);
  pthread_cond_broadcast(&m_cond);
  alock.unLock();

  if(m_thread_id > 0)
    pthread_join(m_thread_id,NULL);

  close(m_pipes[1]),close(m_pipes[0]);
  m_pipes[0] = m_pipes[1] = -1;
  m_thread_id = 0;
}

void CurlLoop::add_request(CurlReq new_request)
{
  Lock alock(&m_lock);

  m_new_requests.push_back(new_request);
  new_request->m_status = FutureRequest::RUNNING;

  if(write(m_pipes[1],"|",1) == -1 && errno != EAGAIN)
    THROW_EIGER_EXCEPTION("write into pipe","synchronization failed");

  pthread_cond_broadcast(&m_cond);
}

void CurlLoop::cancel_request(CurlReq request)
{
  {
    Lock alock(&m_lock);
    m_cancel_requests.push_back(request);
  }
  {
    Lock req_lock(&request->m_lock);
    request->m_status = FutureRequest::CANCEL;
  }
}
void* CurlLoop::_runFunc(void *curlloopPt)
{
  ((CurlLoop*)curlloopPt)->_run();
  return NULL;
}

inline void CurlLoop::_check_new_requests()
{
  ListRequests::iterator i, end = m_new_requests.end();
  for(i = m_new_requests.begin(); i != end; ++i)
    {
      const CurlReq& req = *i;
      ActReq act_req(new ActiveCurlRequest(req,m_multi_handle));
      MapRequests::value_type map_value(req->get_handle(),std::move(act_req));
      typedef std::pair<MapRequests::iterator,bool> InsertResult;
      InsertResult result = m_pending_requests.insert(std::move(map_value));
      if(!result.second)
	THROW_EIGER_EXCEPTION("CurlLoop::_check_new_requests",
			      "handle found in pending requests");
    }
  m_new_requests.clear();
}

inline bool CurlLoop::_wait_input_events()
{
  fd_set fdread;
  fd_set fdwrite;
  fd_set fdexcep;

  FD_ZERO(&fdread);
  FD_SET(m_pipes[0],&fdread);

  FD_ZERO(&fdwrite);
  FD_ZERO(&fdexcep);

  //get curl timeout
  struct timeval timeout{1,0};
  struct timeval* timeoutPt;
  long curl_timeout = -1;
  curl_multi_timeout(m_multi_handle,&curl_timeout);
  if(curl_timeout >=0)
    {
      timeout.tv_sec = curl_timeout / 1000;
      timeout.tv_usec = (curl_timeout % 1000) * 1000;
      timeoutPt = &timeout;
    }
  else
    timeoutPt = NULL;

  int max_fd = -1;
  CURLMcode mc = curl_multi_fdset(m_multi_handle, &fdread,&fdwrite,&fdexcep,
				  &max_fd);
  if(mc != CURLM_OK)
    THROW_EIGER_EXCEPTION("CurlLoop", "error in curl_multi_fdset");
  else if(max_fd == -1) // fds are not ready in curl, wait 100ms
    {
      usleep(100 * 1000);
      return true;
    }

  int ret = select(max_fd + 1,&fdread,&fdwrite,&fdexcep, timeoutPt);
  if(ret < 0)
    {
      if(errno != EINTR)
	THROW_EIGER_EXCEPTION("CurlLoop", "error in select");
      return false;
    }

  return true;
}

inline void CurlLoop::_remove_canceled_requests()
{
  ListRequests::iterator i, end = m_cancel_requests.end();
  for(i = m_cancel_requests.begin(); i != end; ++i)
    {
      const CurlReq& req = *i;
      MapRequests::iterator request = m_pending_requests.find(req->get_handle());
      if(request != m_pending_requests.end())
	m_pending_requests.erase(request);
    }
  m_cancel_requests.clear();
}

void CurlLoop::_run()
{
  Lock lock(&m_lock);
  while(!m_quit)
    {
      while(!m_quit && 
	    m_pending_requests.empty() && m_new_requests.empty())
	pthread_cond_wait(&m_cond,&m_lock);
      if(m_quit)
	break;

      //Add all new requests
      _check_new_requests();

      {
	Unlock u(lock);
	if (!_wait_input_events())
	  continue;
	
	// flush pipe
	char buffer[1024];
	if(read(m_pipes[0],buffer,sizeof(buffer)) == -1 && errno != EAGAIN)
	  THROW_EIGER_EXCEPTION("CurlLoop", "error reading from pipe");
	
	int nb_running;
	curl_multi_perform(m_multi_handle,&nb_running);
      }

      CURLMsg *msg;
      int msg_left;
      while((msg = curl_multi_info_read(m_multi_handle,&msg_left)))
	{
	  if(msg->msg != CURLMSG_DONE)
	    continue;

	  CURL *handle = msg->easy_handle;
	  MapRequests::iterator request = m_pending_requests.find(handle);
	  if(request == m_pending_requests.end())
	    continue;

	  CurlReq req = request->second->req;
	  m_pending_requests.erase(request);

	  Unlock u(lock);
	  req->handle_result(msg->data.result);
	}

      //Remove canceled request
      _remove_canceled_requests();
    }

  //cleanup
  m_pending_requests.clear();
}
