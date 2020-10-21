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
#ifndef EIGERAPI_REQUESTS_H
#define EIGERAPI_REQUESTS_H

#include <string>
#include <map>
#include <vector>

#include "eigerapi/CurlLoop.h"

namespace eigerapi
{
  template <typename T>
  struct HeapDeleter {
    void operator()(T *p) { ::free(p); };
  };
  template <typename T>
  using HeapPtr = std::unique_ptr<T, HeapDeleter<T>>;

  class Requests
  {
  public:
    typedef CurlLoop::CurlReq CurlReq;

    class Command : public CurlLoop::FutureRequest
    {
      friend class Requests;
    public:
      Command(const std::string& url);
      virtual ~Command();

      int get_serie_id();
    private:
      static size_t _write_callback(char *ptr,size_t size,size_t nmemb,Command*);
      void _fill_request();
      char m_data[128];
    };
  
    class Param : public CurlLoop::FutureRequest
    {
      friend class Requests;
    public:
      enum VALUE_TYPE {BOOL,DOUBLE,INT,UNSIGNED,STRING,STRING_ARRAY};
      struct Value
      {
	VALUE_TYPE type;
	union
	{
	  bool bool_val;
	  int  int_val;
	  unsigned int unsigned_val;
	  double double_val;
	}data;
	std::string string_val;
	std::vector<std::string> string_array;
      };
	
      Param(const std::string& url);
      virtual ~Param();
      Value get(double timeout = CurlLoop::FutureRequest::TIMEOUT,
		bool lock = true);
      Value get_min(double timeout = CurlLoop::FutureRequest::TIMEOUT,
		    bool lock = true);
      Value get_max(double timeout = CurlLoop::FutureRequest::TIMEOUT,
		    bool lock = true);
      Value get_allowed_values(double timeout = CurlLoop::FutureRequest::TIMEOUT,
			       bool lock = true);
    private:
      void _fill_get_request();
      template <class T>
      void _fill_set_request(const T& value);

      void _set_return_value(bool&);
      void _set_return_value(double&);
      void _set_return_value(int&);
      void _set_return_value(unsigned int&);
      void _set_return_value(std::string&);
      void _set_return_value(std::vector<std::string>&);

      virtual void _request_finished();

      static size_t _write_callback(char*, size_t, size_t, void*);

      Value _get(double timeout,bool lock,const char*);
      
      char*			m_data_buffer;
      int			m_data_size;
      int			m_data_memorysize;
      struct curl_slist*	m_headers;
      VALUE_TYPE		m_return_type;
      void*			m_return_value;
    };
    
    class Transfer : public CurlLoop::FutureRequest
    {
      friend class Requests;
    public:
      Transfer(Requests& requests,
	       const std::string& url,
	       const std::string& target_path,
	       bool delete_after_transfer = true,
	       int buffer_write_size = 64 * 1024);
      virtual ~Transfer();
    private:
      static size_t _write(void *ptr, size_t size, size_t nmemb,Transfer*);
      virtual void _request_finished();
      
      Requests&	m_requests;
      bool	m_delete_after_transfer;
      long	m_download_size;
      FILE*	m_target_file;
      HeapPtr<void> m_buffer;
    };

    typedef std::shared_ptr<Command> CommandReq;
    typedef std::shared_ptr<Param> ParamReq;
    typedef std::shared_ptr<Transfer> TransferReq;

    enum COMMAND_NAME {INITIALIZE,ARM, DISARM,TRIGGER,CANCEL,ABORT,
		       FILEWRITER_CLEAR, HV_RESET};
    enum PARAM_NAME {TEMP,
		     HUMIDITY,
                     HVSTATE,
                     HVMEASURED,
                     HVTARGET,
		     DETECTOR_STATUS,
		     PIXELDEPTH,
		     X_PIXEL_SIZE,
		     Y_PIXEL_SIZE,
		     DETECTOR_WITDH,
		     DETECTOR_HEIGHT,
		     DESCRIPTION,
		     DETECTOR_NUMBER,
		     DETECTOR_READOUT_TIME,
                     DATA_COLLECTION_DATE,
                     SOFTWARE_VERSION,
		     EXPOSURE,
		     FRAME_TIME,
		     TRIGGER_MODE,
		     COUNTRATE_CORRECTION,
		     FLATFIELD_CORRECTION,
		     EFFICIENCY_CORRECTION,
                     RETRIGGER,
		     PIXEL_MASK,
		     THRESHOLD_ENERGY,
		     VIRTUAL_PIXEL_CORRECTION,
		     PHOTON_ENERGY,
 		     NIMAGES,
		     NTRIGGER,
		     AUTO_SUMMATION,
		     FILEWRITER_MODE,
		     FILEWRITER_COMPRESSION,
		     FILEWRITER_NAME_PATTERN,
		     NIMAGES_PER_FILE,
		     FILEWRITER_STATUS,
		     FILEWRITER_ERROR,
		     FILEWRITER_TIME,
		     FILEWRITER_BUFFER_FREE,
		     FILEWRITER_LS,
		     FILEWRITER_LS2,
		     STREAM_MODE,
		     STREAM_HEADER_DETAIL,
		     HEADER_BEAM_CENTER_X,
		     HEADER_BEAM_CENTER_Y,
		     HEADER_CHI_INCREMENT,
		     HEADER_CHI_START,
		     HEADER_DETECTOR_DISTANCE,
		     HEADER_KAPPA_INCREMENT,
		     HEADER_KAPPA_START,
		     HEADER_OMEGA_INCREMENT,
		     HEADER_OMEGA_START,
		     HEADER_PHI_INCREMENT,
		     HEADER_PHI_START,
		     HEADER_WAVELENGTH,
		     COMPRESSION_TYPE,
    };

    Requests(const std::string& address);
    ~Requests();

    std::string get_api_version();

    CommandReq get_command(COMMAND_NAME);
    ParamReq get_param(PARAM_NAME);
    ParamReq get_param(PARAM_NAME,bool&);
    ParamReq get_param(PARAM_NAME,double&);
    ParamReq get_param(PARAM_NAME,int&);
    ParamReq get_param(PARAM_NAME,unsigned int&);
    ParamReq get_param(PARAM_NAME,std::string&);
    ParamReq get_param(PARAM_NAME,std::vector<std::string>&);

    ParamReq set_param(PARAM_NAME,bool);
    ParamReq set_param(PARAM_NAME,double);
    ParamReq set_param(PARAM_NAME,int);
    ParamReq set_param(PARAM_NAME,unsigned int);
    ParamReq set_param(PARAM_NAME,const std::string&);
    ParamReq set_param(PARAM_NAME,const char*);

    TransferReq start_transfer(const std::string& src_filename,
			       const std::string& target_path,
			       bool delete_after_transfer = true);
    CurlReq delete_file(const std::string& filename, bool full_url = false);
    
    void cancel(CurlReq request);
  private:
    ParamReq _create_get_param(PARAM_NAME);
    template <class T>
    ParamReq _set_param(PARAM_NAME,const T&);


    typedef std::map<int,std::string> CACHE_TYPE;
    std::string m_address;
    std::string m_api_version;
    CACHE_TYPE	m_cmd_cache_url;
    CACHE_TYPE	m_param_cache_url;
    CurlLoop	m_loop;
  };
}

#endif // EIGERAPI_REQUESTS_H
