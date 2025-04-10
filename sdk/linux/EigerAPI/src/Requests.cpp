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
#include <cstdlib>
#include <cstring>
#include <stdio.h>

#include <sstream>

#include <json/json.h>

#include "eigerapi/Requests.h"
#include "eigerapi/EigerDefines.h"
#include "AutoMutex.h"

using namespace eigerapi;

typedef Requests::CommandReq CommandReq;
typedef Requests::ParamReq ParamReq;
typedef Requests::TransferReq TransferReq;
typedef Requests::CurlReq CurlReq;

static const char* CSTR_EIGERCONFIG		= "config";
static const char* CSTR_EIGERSTATUS		= "status";
static const char* CSTR_EIGERCOMMAND		= "command";
static const char* CSTR_EIGERFILES		= "files";
static const char* CSTR_SUBSYSTEMFILEWRITER	= "filewriter";
static const char* CSTR_SUBSYSTEMSTREAM		= "stream";
static const char* CSTR_SUBSYSTEMDETECTOR	= "detector";
static const char* CSTR_DATA			= "data";
static const char* CSTR_EIGERVERSION		= "version";
static const char* CSTR_EIGERAPI		= "api";

struct ResourceDescription
{
   ResourceDescription(const char* name,
                       const char* subsystem=CSTR_SUBSYSTEMDETECTOR,
                       const char* location=CSTR_EIGERCONFIG) :
     m_name(name),
     m_subsystem(subsystem),
     m_location(location)
  {};
  
  std::string build_url(const std::ostringstream& base_url,
			const std::ostringstream& api);
  
  const char*       m_name;
  const char*       m_subsystem;   
  const char*       m_location;
  std::string	    m_url;
};

struct CommandIndex
{
  Requests::COMMAND_NAME	name;
  ResourceDescription		desc;
};

static CommandIndex CommandsDescription[] = {
  {Requests::INITIALIZE,	{"initialize",CSTR_SUBSYSTEMDETECTOR,CSTR_EIGERCOMMAND}},
  {Requests::ARM,		{"arm",CSTR_SUBSYSTEMDETECTOR,CSTR_EIGERCOMMAND}},
  {Requests::DISARM,		{"disarm",CSTR_SUBSYSTEMDETECTOR,CSTR_EIGERCOMMAND}},
  {Requests::TRIGGER,		{"trigger",CSTR_SUBSYSTEMDETECTOR,CSTR_EIGERCOMMAND}},
  {Requests::CANCEL,		{"cancel",CSTR_SUBSYSTEMDETECTOR,CSTR_EIGERCOMMAND}},
  {Requests::ABORT,		{"abort",CSTR_SUBSYSTEMDETECTOR,CSTR_EIGERCOMMAND}},
  {Requests::FILEWRITER_CLEAR,  {"clear",CSTR_SUBSYSTEMFILEWRITER,CSTR_EIGERCOMMAND}},
  {Requests::HV_RESET,          {"hv_reset",CSTR_SUBSYSTEMDETECTOR,CSTR_EIGERCOMMAND}},
};

const char* get_cmd_name(Requests::COMMAND_NAME cmd_name)
{
  int nb_cmd = sizeof(CommandsDescription) / sizeof(CommandIndex);
  for(int i = 0;i < nb_cmd;++i)
    {
      if(CommandsDescription[i].name == cmd_name)
	return CommandsDescription[i].desc.m_name;
    }
  return "not found";		// weired
}
struct ParamIndex
{
  Requests::PARAM_NAME	name;
  ResourceDescription	desc;
};

ParamIndex ParamDescription[] = {
  // Detector Read only values
  {Requests::TEMP,				{"temperature",CSTR_SUBSYSTEMDETECTOR,CSTR_EIGERSTATUS}},
  {Requests::HUMIDITY,				{"humidity",CSTR_SUBSYSTEMDETECTOR,CSTR_EIGERSTATUS}},
  {Requests::HVSTATE,				{"high_voltage/state",CSTR_SUBSYSTEMDETECTOR,CSTR_EIGERSTATUS}},
  {Requests::DETECTOR_STATUS,			{"state",CSTR_SUBSYSTEMDETECTOR,CSTR_EIGERSTATUS}},
  {Requests::BIT_DEPTH_IMAGE,			{"bit_depth_image"}},
  {Requests::BIT_DEPTH_READOUT,			{"bit_depth_readout"}},
  {Requests::X_PIXEL_SIZE,			{"x_pixel_size"}},
  {Requests::Y_PIXEL_SIZE,			{"y_pixel_size"}},
  {Requests::DETECTOR_WITDH,			{"x_pixels_in_detector"}},
  {Requests::DETECTOR_HEIGHT,			{"y_pixels_in_detector"}},
  {Requests::DESCRIPTION,			{"description"}},
  {Requests::DETECTOR_NUMBER,			{"detector_number"}},
  {Requests::DETECTOR_READOUT_TIME,		{"detector_readout_time"}},
  {Requests::DATA_COLLECTION_DATE,		{"data_collection_date"}},
  {Requests::SOFTWARE_VERSION,			{"software_version"}},

  // Detector Read/Write settings
  {Requests::EXPOSURE,				{"count_time"}},
  {Requests::FRAME_TIME,			{"frame_time"}},
  {Requests::TRIGGER_MODE,			{"trigger_mode"}},
  {Requests::COUNTRATE_CORRECTION,		{"countrate_correction_applied"}},
  {Requests::FLATFIELD_CORRECTION,		{"flatfield_correction_applied"}},
  {Requests::EFFICIENCY_CORRECTION,		{"efficiency_correction_applied"}},
  {Requests::PIXEL_MASK,			{"pixel_mask_applied"}},
  {Requests::THRESHOLD_ENERGY,			{"threshold_energy"}},
  {Requests::THRESHOLD_ENERGY2,			{"threshold/2/energy"}},
  {Requests::THRESHOLD_MODE2,			{"threshold/2/mode"}},
  {Requests::THRESHOLD_DIFF_MODE,		{"threshold/difference/mode"}},
  {Requests::VIRTUAL_PIXEL_CORRECTION,		{"virtual_pixel_correction_applied"}},
  {Requests::PHOTON_ENERGY,			{"photon_energy"}},
  {Requests::NIMAGES,				{"nimages"}},
  {Requests::NTRIGGER,				{"ntrigger"}},
  {Requests::AUTO_SUMMATION,			{"auto_summation"}},
  {Requests::RETRIGGER, 			{"retrigger"}},
  {Requests::ROI_MODE, 			{"roi_mode"}},

  // Filewriter settings
  {Requests::FILEWRITER_MODE,			{"mode",CSTR_SUBSYSTEMFILEWRITER}},
  {Requests::FILEWRITER_COMPRESSION,		{"compression_enabled", CSTR_SUBSYSTEMFILEWRITER}},
  {Requests::FILEWRITER_NAME_PATTERN,		{"name_pattern", CSTR_SUBSYSTEMFILEWRITER}},
  {Requests::NIMAGES_PER_FILE,			{"nimages_per_file", CSTR_SUBSYSTEMFILEWRITER}},
  {Requests::FILEWRITER_STATUS,			{"state", CSTR_SUBSYSTEMFILEWRITER, CSTR_EIGERSTATUS}},
  {Requests::FILEWRITER_ERROR,			{"error", CSTR_SUBSYSTEMFILEWRITER, CSTR_EIGERSTATUS}},
  {Requests::FILEWRITER_TIME,			{"time", CSTR_SUBSYSTEMFILEWRITER, CSTR_EIGERSTATUS}},
  {Requests::FILEWRITER_BUFFER_FREE,		{"buffer_free", CSTR_SUBSYSTEMFILEWRITER, CSTR_EIGERSTATUS}},
  {Requests::FILEWRITER_LS,			{"", CSTR_SUBSYSTEMFILEWRITER, CSTR_EIGERFILES}},
  {Requests::FILEWRITER_LS2,			{"files", CSTR_SUBSYSTEMFILEWRITER, CSTR_EIGERSTATUS}},
  // Stream settings
  {Requests::STREAM_MODE,			{"mode", CSTR_SUBSYSTEMSTREAM}},
  {Requests::STREAM_HEADER_DETAIL,		{"header_detail", CSTR_SUBSYSTEMSTREAM}},
  // Saving Header
  {Requests::HEADER_BEAM_CENTER_X,		{"beam_center_x"}},
  {Requests::HEADER_BEAM_CENTER_Y,		{"beam_center_y"}},
  {Requests::HEADER_CHI_INCREMENT,		{"chi_increment"}},
  {Requests::HEADER_CHI_START,			{"chi_start"}},
  {Requests::HEADER_DETECTOR_DISTANCE,		{"detector_distance"}},
  {Requests::HEADER_KAPPA_INCREMENT,		{"kappa_increment"}},
  {Requests::HEADER_KAPPA_START,		{"kappa_start"}},
  {Requests::HEADER_OMEGA_INCREMENT,		{"omega_increment"}},
  {Requests::HEADER_OMEGA_START,		{"omega_start"}},
  {Requests::HEADER_PHI_INCREMENT,		{"phi_increment"}},
  {Requests::HEADER_PHI_START,			{"phi_start"}},
  {Requests::HEADER_WAVELENGTH,			{"wavelength"}},
  // Compression
  {Requests::COMPRESSION_TYPE,			{"compression"}},
};

const char* get_param_name(Requests::PARAM_NAME param_name)
{
  int nb_param = sizeof(ParamDescription) / sizeof(ParamIndex);
  for(int i = 0;i < nb_param;++i)
    {
      if(ParamDescription[i].name == param_name)
	return ParamDescription[i].desc.m_name;
    }
  return "not found";		// weired
}

std::string ResourceDescription::build_url(const std::ostringstream& base_url,
					   const std::ostringstream& api)
{
  std::ostringstream url;
  if(!m_location)
    url << base_url.str() << m_subsystem << api.str() << m_name;
  else    
    url << base_url.str() << m_subsystem << api.str() << m_location << '/' << m_name;
  return url.str();
}

// Requests class
Requests::Requests(const std::string& address) :
  m_address(address)
{
  std::ostringstream base_url;
  base_url << "http://" << address << '/';

  std::ostringstream url;
  url  << base_url.str() << CSTR_SUBSYSTEMDETECTOR << '/'
       << CSTR_EIGERAPI << '/' << CSTR_EIGERVERSION << '/';
  
  ParamReq version_request(new Param(url.str()));
  version_request->_fill_get_request();
  m_loop.add_request(version_request);
  
  Requests::Param::Value value = version_request->get();
  m_api_version = value.string_val;
  
  std::ostringstream api;
  api << '/' << CSTR_EIGERAPI << '/' << m_api_version << '/';
  
  // COMMANDS URL CACHE
  int nb_cmd = sizeof(CommandsDescription) / sizeof(CommandIndex);
  for(int i = 0;i < nb_cmd;++i)
    {
      CommandIndex& index = CommandsDescription[i];
      m_cmd_cache_url[index.name] = index.desc.build_url(base_url,api);
    }
  // PARAMS URL CACHE
  int nb_params = sizeof(ParamDescription) / sizeof(ParamIndex);
  for(int i = 0;i < nb_params;++i)
    {
      ParamIndex& index = ParamDescription[i];
      m_param_cache_url[index.name] = index.desc.build_url(base_url,api);
    }
}

Requests::~Requests()
{
}

std::string Requests::get_api_version()
{
  return m_api_version;
}

CommandReq Requests::get_command(Requests::COMMAND_NAME cmd_name)
{
  CACHE_TYPE::iterator cmd_url = m_cmd_cache_url.find(cmd_name);
  if(cmd_url == m_cmd_cache_url.end())
    THROW_EIGER_EXCEPTION(RESOURCE_NOT_FOUND,get_cmd_name(cmd_name));

  CommandReq cmd(new Command(cmd_url->second));
  cmd->_fill_request();
  m_loop.add_request(cmd);
  return cmd;
}

ParamReq Requests::get_param(Requests::PARAM_NAME param_name)
{
  ParamReq param = _create_get_param(param_name);

  m_loop.add_request(param);
  return param;
}

#define GENERATE_GET_PARAM()						\
  ParamReq param = _create_get_param(param_name); \
  param->_set_return_value(ret_value);					\
									\
  m_loop.add_request(param);						\
  return param;							\

ParamReq Requests::get_param(Requests::PARAM_NAME param_name,bool& ret_value)
{
  GENERATE_GET_PARAM();
}

ParamReq Requests::get_param(Requests::PARAM_NAME param_name,
		    double& ret_value)
{
  GENERATE_GET_PARAM();
}

ParamReq Requests::get_param(Requests::PARAM_NAME param_name,
		    int& ret_value)
{
  GENERATE_GET_PARAM();
}

ParamReq Requests::get_param(Requests::PARAM_NAME param_name,
		    unsigned int& ret_value)
{
  GENERATE_GET_PARAM();
}

ParamReq Requests::get_param(Requests::PARAM_NAME param_name,
		    std::string& ret_value)
{
  GENERATE_GET_PARAM();
}

ParamReq Requests::get_param(Requests::PARAM_NAME param_name,
		    std::vector<std::string>& ret_value)
{
  GENERATE_GET_PARAM();
}

ParamReq Requests::_create_get_param(Requests::PARAM_NAME param_name)
{
  CACHE_TYPE::iterator param_url = m_param_cache_url.find(param_name);
  if(param_url == m_param_cache_url.end())
    THROW_EIGER_EXCEPTION(RESOURCE_NOT_FOUND,get_param_name(param_name));
  
  ParamReq param(new Param(param_url->second));
  param->_fill_get_request();
  return param;
}

template <class T>
ParamReq Requests::_set_param(Requests::PARAM_NAME param_name,const T& value)
{
  CACHE_TYPE::iterator param_url = m_param_cache_url.find(param_name);
  if(param_url == m_param_cache_url.end())
    THROW_EIGER_EXCEPTION(RESOURCE_NOT_FOUND,get_param_name(param_name));

  ParamReq param(new Param(param_url->second));
  param->_fill_set_request(value);
  m_loop.add_request(param);
  return param;
}

ParamReq Requests::set_param(Requests::PARAM_NAME name,bool value)
{
  return _set_param(name,value);
}

ParamReq Requests::set_param(PARAM_NAME name,double value)
{
  return _set_param(name,value);
}

ParamReq Requests::set_param(PARAM_NAME name,int value)
{
  return _set_param(name,value);
}

ParamReq Requests::set_param(PARAM_NAME name,unsigned int value)
{
  return _set_param(name,value);
}

ParamReq Requests::set_param(PARAM_NAME name,const std::string& value)
{
  return _set_param(name,value);
}

ParamReq Requests::set_param(PARAM_NAME name,const char* value)
{
  return _set_param(name,value);
}


TransferReq  Requests::start_transfer(const std::string& src_filename,
				      const std::string& dest_path,
				      bool delete_after_transfer)
{
  std::ostringstream url;
  url << "http://" << m_address << '/' << CSTR_DATA << '/'
      << src_filename;
  
  TransferReq transfer(new Transfer(*this,
						  url.str(),
						  dest_path,
						  delete_after_transfer));
  m_loop.add_request(transfer);
  return transfer;
}

CurlReq Requests::delete_file(const std::string& filename,bool full_url)
{
  std::ostringstream url;
  if(!full_url)
    url << "http://" << m_address << '/' << CSTR_DATA << '/'
	<< filename;
  else
    url << filename;

 CurlReq delete_req(new CurlLoop::FutureRequest(url.str()));
 CURL* handle = delete_req->get_handle();
 curl_easy_setopt(handle, CURLOPT_CUSTOMREQUEST, "DELETE"); 
 m_loop.add_request(delete_req);
 return delete_req;
}

void Requests::cancel(CurlReq req)
{
  m_loop.cancel_request(req);
}
//Class Command
Requests::Command::Command(const std::string& url) :
  CurlLoop::FutureRequest(url)
{
  m_data[0] = '\0';
}

Requests::Command::~Command()
{
}

void Requests::Command::_fill_request()
{
  curl_easy_setopt(m_handle,CURLOPT_CUSTOMREQUEST, "PUT");
  curl_easy_setopt(m_handle,CURLOPT_WRITEFUNCTION,_write_callback);
  curl_easy_setopt(m_handle,CURLOPT_WRITEDATA,this);
}

size_t Requests::Command::_write_callback(char *ptr,size_t size,
					  size_t nmemb,Requests::Command *cmd)
{
  size_t len = size * nmemb;
  if (cmd->check_http_response(ptr, len))
    return len;
  int size_to_copy = std::min(len, sizeof(m_data) - 1);
  if (size_to_copy < len)
    std::cout << "Requests::Command: Warning: unexpected (long) message: "
	      << std::string(ptr, len) << std::endl;
  memcpy(cmd->m_data,ptr,size_to_copy);
  cmd->m_data[size_to_copy] = '\0';
  return size_to_copy;
}

int Requests::Command::get_serie_id()
{
  Json::Value  root;
  Json::CharReaderBuilder rbuilder;
  std::unique_ptr<Json::CharReader> const reader(rbuilder.newCharReader());
  std::string errs;

  if (!reader->parse(m_data, m_data+strlen(m_data), &root, &errs))
    THROW_EIGER_EXCEPTION(eigerapi::JSON_PARSE_FAILED, errs.c_str());

  int seq_id = root.get("sequence id", -1).asInt();
  return seq_id;
}
//Class Param

Requests::Param::Param(const std::string& url) :
  CurlLoop::FutureRequest(url),
  m_data_buffer(NULL),
  m_data_size(0),
  m_data_memorysize(0),
  m_headers(NULL),
  m_return_value(NULL)
{
}

Requests::Param::~Param()
{
  if(m_data_memorysize)
    free(m_data_buffer);
  
  if(m_headers)
    curl_slist_free_all(m_headers);
}

Requests::Param::Value Requests::Param::_get(double timeout,bool lock,
					     const char* param_name)
{
  wait(timeout,lock);
  //check rx data
  if(!m_data_buffer)
    THROW_EIGER_EXCEPTION("No data received","");

  if(m_data_size == m_data_memorysize) // need to add an extra char
    {
      int alloc_size = (m_data_memorysize + 1024) & ~1023;
      m_data_buffer = (char*)realloc(m_data_buffer,alloc_size);
      m_data_memorysize = alloc_size;
    }

  m_data_buffer[m_data_size] = '\0'; // string ending

  //- Json decoding to return the wanted data
  Json::Value  root;
  Json::CharReaderBuilder rbuilder;
  std::unique_ptr<Json::CharReader> const reader(rbuilder.newCharReader());
  std::string errs;

  if (!reader->parse(m_data_buffer, m_data_buffer+m_data_size, &root, &errs))
    THROW_EIGER_EXCEPTION(eigerapi::JSON_PARSE_FAILED, errs.c_str());

  Value value;
  std::string json_type;
  bool is_list = root.isArray() || root.get(param_name, "no_value").isArray();
  if (!is_list) {
    json_type = root.get("value_type", "dummy").asString();
    is_list = (json_type == "list");
  }
  if(is_list)
    {
      value.type = Requests::Param::STRING_ARRAY;
      Json::Value& array = root.isArray() ? root : root[param_name];
      int array_size = array.size();
      for(int i = 0;i < array_size;++i)
	value.string_array.push_back(array[i].asString());
    }
  else
    {
      //- supported types by dectris are:
      //- bool, float, int, string or a list of float or int
      if (json_type == "bool")
	{
	  value.type = Requests::Param::BOOL;
	  value.data.bool_val = root.get(param_name, "no_value").asBool();
	}
      else if (json_type == "float")
	{
	  //- asFloat is not supported by jsoncpp
	  value.type = Requests::Param::DOUBLE;
	  value.data.double_val = root.get(param_name, -1.0).asDouble();
	}
      else if (json_type == "int")
	{
	  value.type = Requests::Param::INT;
	  value.data.int_val = root.get(param_name, -1).asInt();
	}
      else if (json_type == "uint")
	{
	  value.type = Requests::Param::UNSIGNED;
	  value.data.unsigned_val = (int) root.get(param_name, -1).asInt();
	}
      else if (json_type == "string")
	{
	  value.type = Requests::Param::STRING;
	  value.string_val = root.get(param_name,"no_value").asString();
	}
      else
	{
	  THROW_EIGER_EXCEPTION(eigerapi::DATA_TYPE_NOT_HANDLED,
				json_type.c_str());
	}
    }
  return value;
}

Requests::Param::Value Requests::Param::get(double timeout,bool lock)
{
  return _get(timeout,lock,"value");
}

Requests::Param::Value Requests::Param::get_min(double timeout,bool lock)
{
  return _get(timeout,lock,"min");
}

Requests::Param::Value Requests::Param::get_max(double timeout,bool lock)
{
  return _get(timeout,lock,"max");
}

Requests::Param::Value Requests::Param::get_allowed_values(double timeout,bool lock)
{
  return _get(timeout,lock,"allowed_values");
}

void Requests::Param::_fill_get_request()
{
  curl_easy_setopt(m_handle,CURLOPT_WRITEFUNCTION,_write_callback);
  curl_easy_setopt(m_handle,CURLOPT_WRITEDATA,this);
}

template <class T>
void Requests::Param::_fill_set_request(const T& value)
{
  Json::Value root;
  root["value"] = value;

  Json::StreamWriterBuilder wbuilder;
  std::string json_struct = Json::writeString(wbuilder, root);

  m_headers = curl_slist_append(m_headers, "Accept: application/json");
  m_headers = curl_slist_append(m_headers, "Content-Type: application/json;charset=utf-8");

  curl_easy_setopt(m_handle, CURLOPT_HTTPHEADER, m_headers);
  curl_easy_setopt(m_handle, CURLOPT_CUSTOMREQUEST, "PUT"); 
  curl_easy_setopt(m_handle, CURLOPT_FAILONERROR, true);

  m_data_buffer = strdup(json_struct.c_str()),m_data_memorysize = json_struct.length();
  curl_easy_setopt(m_handle, CURLOPT_POSTFIELDS, m_data_buffer); // data goes here
  curl_easy_setopt(m_handle, CURLOPT_POSTFIELDSIZE,json_struct.length()); // data length

  curl_easy_setopt(m_handle,CURLOPT_WRITEFUNCTION,_write_callback);
  curl_easy_setopt(m_handle,CURLOPT_WRITEDATA,this);
}

void Requests::Param::_set_return_value(bool& ret_value)
{
  m_return_value = &ret_value;
  m_return_type = BOOL;
}
void Requests::Param::_set_return_value(double& ret_value)
{
  m_return_value = &ret_value;
  m_return_type = DOUBLE;
}
void Requests::Param::_set_return_value(int& ret_value)
{
  m_return_value = &ret_value;
  m_return_type = INT;
}
void Requests::Param::_set_return_value(unsigned int& ret_value)
{
  m_return_value = &ret_value;
  m_return_type = UNSIGNED;
}
void Requests::Param::_set_return_value(std::string& ret_value)
{
  m_return_value = &ret_value;
  m_return_type = STRING;
}
void Requests::Param::_set_return_value(std::vector<std::string>& ret_value)
{
  m_return_value = &ret_value;
  m_return_type = STRING_ARRAY;
}

void Requests::Param::_request_finished()
{
  if(m_status == CANCEL) return;

  std::string error_string;
  if(m_return_value)
    {
      Value value;
      try {value = get(0,false);}
      catch(EigerException &e)
	{
	  error_string = e.what();
	  goto error;
	}

      switch(m_return_type)
	{
	case BOOL:
	  {
	    bool *return_val = (bool*)m_return_value;
	    switch(value.type)
	      {
	      case BOOL:
		*return_val = value.data.bool_val;break;
	      case INT:
		*return_val = value.data.int_val;break;
	      case UNSIGNED:
		*return_val = value.data.unsigned_val;break;
	      default:
		error_string = "Rx value is not a bool";
		goto error;
	      }
	    break;
	  }
	case DOUBLE:
	  {
	    double *return_val = (double*)m_return_value;
	    switch(value.type)
	      {
	      case INT:
		*return_val = value.data.int_val;break;
	      case UNSIGNED:
		*return_val = value.data.unsigned_val;break;
	      case DOUBLE:
		*return_val = value.data.double_val;break;
	      default:
		error_string = "Rx value is not a double";
		goto error;
	      }
	    break;
	  }
	case INT:
	  {
	    int *return_val = (int*)m_return_value;
	    switch(value.type)
	      {
	      case INT:
		*return_val = value.data.int_val;break;
	      default:
		error_string = "Rx value is not a integer";
		goto error;
	      }
	    break;
	  }
	case UNSIGNED:
	  {
	    unsigned int *return_val = (unsigned int*)m_return_value;
	    switch(value.type)
	      {
	      case INT:
		*return_val = value.data.int_val;break;
	      case UNSIGNED:
		*return_val = value.data.unsigned_val;break;
	      default:
		error_string = "Rx value is not a unsigned integer";
		goto error;
	      }
	    break;
	  }
	case STRING:
	  {
	    std::string *return_val = (std::string*)m_return_value;
	    switch(value.type)
	      {
	      case STRING:
		*return_val = value.string_val;break;
	      default:
		error_string = "Rx value is not a string";
		goto error;
	      }
	    break;
	  }
	case STRING_ARRAY:
	  {
	    std::vector<std::string> *return_val;
	    return_val = (std::vector<std::string>*)m_return_value;
	    switch(value.type)
	      {
	      case STRING_ARRAY:
		*return_val = value.string_array;break;
	      case STRING:
		(*return_val).assign(1,value.string_val);break;
	      default:
		error_string = "Rx value is not a string array";
		goto error;
	      }
	    break;
	  }
	default:
	  error_string = "Value type not yet managed";
	  goto error;
	}
    }
  return;

 error:
  if(m_error_code.empty())
    m_error_code = error_string + "(" + m_url + ")";
  else
    {
      m_error_code += "\n";
      m_error_code += error_string;
    }
  m_status = ERROR;
}

size_t Requests::Param::_write_callback(char *ptr,size_t size,
					size_t nmemb,void *userdata)
{
  Param *t = (Param*)userdata;
  int size_to_copy = size * nmemb;
  if(size_to_copy > 0)
    {
      int request_memory_size = t->m_data_size + size_to_copy;
      if(request_memory_size > t->m_data_memorysize) // realloc
	{
	  int alloc_size = (request_memory_size + 4095) & ~4095;
	  t->m_data_buffer = (char*)realloc(t->m_data_buffer,alloc_size);
	  t->m_data_memorysize = alloc_size;
	}
      memcpy(t->m_data_buffer + t->m_data_size,ptr,size_to_copy);
      t->m_data_size += size_to_copy;
    }
  return size_to_copy;
}

/*----------------------------------------------------------------------------
			   Class Transfer
----------------------------------------------------------------------------*/
Requests::Transfer::Transfer(Requests& requests,
			     const std::string& url,
			     const std::string& target_path,
			     bool delete_after_transfer,
			     int buffer_write_size) :
  CurlLoop::FutureRequest(url),
  m_requests(requests),
  m_delete_after_transfer(delete_after_transfer),
  m_download_size(0)
{
  void *ptr;
  if(posix_memalign(&ptr,4*1024,buffer_write_size))
    THROW_EIGER_EXCEPTION("Can't allocate write buffer memory","");
  m_buffer.reset(ptr);
  m_target_file = fopen(target_path.c_str(),"w+");
  if(!m_target_file)
    {
      char str_errno[1024];
      strerror_r(errno,str_errno,sizeof(str_errno));
      std::ostringstream error_buffer;
      error_buffer << "Can't open destination file " << target_path;
      THROW_EIGER_EXCEPTION(error_buffer.str().c_str(), str_errno);
    }
  setbuffer(m_target_file,(char*)m_buffer.get(),buffer_write_size);
  curl_easy_setopt(m_handle, CURLOPT_WRITEFUNCTION, _write);
  curl_easy_setopt(m_handle, CURLOPT_WRITEDATA, this);
}
Requests::Transfer::~Transfer()
{
  if(m_target_file)
    fclose(m_target_file);
}

size_t
Requests::Transfer::_write(void *ptr, size_t size,
			    size_t nmemb, Requests::Transfer *transfer)
{

  FILE *stream = transfer->m_target_file;
  size_t write_size = fwrite(ptr, size, nmemb, stream);

  Lock lock(&transfer->m_lock);
  if(write_size > 0)
    transfer->m_download_size += size * nmemb;

  return write_size;
}

void Requests::Transfer::_request_finished()
{
  fclose(m_target_file);m_target_file = NULL;
  // start new request to delete the file
  if(m_status == FutureRequest::OK &&
     m_delete_after_transfer)
    m_requests.delete_file(m_url,true);
}
