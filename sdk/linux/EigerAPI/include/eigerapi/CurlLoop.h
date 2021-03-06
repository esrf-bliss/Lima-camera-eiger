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
#ifndef EIGERAPI_CURLLOOP_H
#define EIGERAPI_CURLLOOP_H

#include <pthread.h>
#include <curl/curl.h>

#include <memory>
#include <map>
#include <list>
#include <string>

namespace eigerapi
{
  class CurlLoop
  {
  public:
    class FutureRequest
    {
      friend class CurlLoop;
    public:
      static double const TIMEOUT;
      enum Status {IDLE,CANCEL,RUNNING,OK,ERROR};
      virtual ~FutureRequest();
      
      void wait(double timeout = TIMEOUT,bool lock = true) const;
      Status get_status() const;

      class Callback
      {
      public:
	Callback() {};
	virtual ~Callback() {}
	virtual void status_changed(FutureRequest::Status status,
				    std::string error) = 0;
      };
      typedef std::shared_ptr<Callback> CallbackPtr;
      void register_callback(CallbackPtr& cbk, bool in_thread = false);

      CURL* get_handle() const {return m_handle;}
      const std::string& get_url() const {return m_url;}

      FutureRequest(const std::string& url);
    protected:
      virtual void _request_finished() {};

      void handle_result(CURLcode result);
      void _status_changed();

      bool check_http_response(const char *ptr, size_t size);

      static void *_callback_thread_runFunc(void *data);

      CURL*				m_handle;
      Status				m_status;
      std::string			m_error_code;
      int				m_http_code;
      std::string			m_http_msg;
      // Synchro
      mutable pthread_mutex_t		m_lock;
      mutable pthread_cond_t		m_cond;
      CallbackPtr*			m_cbk;
      bool				m_cbk_in_thread;
      std::string			m_url;
    };
    typedef std::shared_ptr<FutureRequest> CurlReq;

    CurlLoop();
    ~CurlLoop();

    void quit();		// quit the curl loop

    void add_request(CurlReq);
    void cancel_request(CurlReq);

  private:
    struct ActiveCurlRequest;
    typedef std::unique_ptr<const ActiveCurlRequest> ActReq;
    typedef std::map<CURL*,ActReq> MapRequests;
    typedef std::list<CurlReq> ListRequests;
    static void* _runFunc(void*);
    void _run();
    void _check_new_requests();
    bool _wait_input_events();
    void _remove_canceled_requests();

    // Synchro
    pthread_mutex_t	m_lock;
    pthread_cond_t	m_cond;
    CURLM*		m_multi_handle;
    int			m_pipes[2];
    bool		m_quit;
    pthread_t		m_thread_id;
    //Pending Request
    MapRequests		m_pending_requests;
    ListRequests	m_new_requests;
    ListRequests	m_cancel_requests;
  };
}

#endif // EIGERAPI_CURLLOOP_H
