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
#include <algorithm>
#include "EigerSavingCtrlObj.h"
#include "EigerCameraRequests.h"

#include <eigerapi/Requests.h>
#include <eigerapi/EigerDefines.h>

using namespace lima;
using namespace lima::Eiger;
using namespace eigerapi;

const int MAX_SIMULTANEOUS_DOWNLOAD = 4;
/*----------------------------------------------------------------------------
			     HDF5 HEADER
----------------------------------------------------------------------------*/
struct HeaderKey2Index
{
  const char*		key_name;
  Requests::PARAM_NAME	param_name;
};

static HeaderKey2Index available_header[] = {
  {"beam_center_x",Requests::HEADER_BEAM_CENTER_X},
  {"beam_center_y",Requests::HEADER_BEAM_CENTER_Y},
  {"chi_increment",Requests::HEADER_CHI_INCREMENT},
  {"chi_start",Requests::HEADER_CHI_START},
  {"detector_distance",Requests::HEADER_DETECTOR_DISTANCE},
  {"kappa_increment",Requests::HEADER_KAPPA_INCREMENT},
  {"kappa_start",Requests::HEADER_KAPPA_START},
  {"omega_increment",Requests::HEADER_OMEGA_INCREMENT},
  {"omega_start",Requests::HEADER_OMEGA_START},
  {"phi_increment",Requests::HEADER_PHI_INCREMENT},
  {"phi_start",Requests::HEADER_PHI_START},
  {"wavelength",Requests::HEADER_WAVELENGTH},
};
/*----------------------------------------------------------------------------
			    Polling thread
----------------------------------------------------------------------------*/
class SavingCtrlObj::_PollingThread : public Thread
{
  DEB_CLASS_NAMESPC(DebModCamera,"SavingCtrlObj","_PollingThread");
public:
  _PollingThread(SavingCtrlObj&,eigerapi::Requests*);
  virtual ~_PollingThread();
protected:
  virtual void threadFunction();
private:
  SavingCtrlObj&	m_saving;
  eigerapi::Requests*	m_requests;
};

SavingCtrlObj::SavingCtrlObj(Camera& cam) :
  HwSavingCtrlObj(HwSavingCtrlObj::COMMON_HEADER,false),
  m_cam(cam),
  m_nb_file_to_watch(0),
  m_nb_file_transfer_started(0),
  m_concurrent_download(0),
  m_poll_master_file(false),
  m_quit(false)
{
  m_polling_thread = new _PollingThread(*this,this->m_cam.m_requests);
  m_polling_thread->start();
  // Known keys for common header
  int nb_header_key = sizeof(available_header) / sizeof(HeaderKey2Index);
  for(int i = 0;i < nb_header_key;++i)
    {
      HeaderKey2Index& index = available_header[i];
      m_availables_header_keys[index.key_name] = index.param_name;
    }
}

/*----------------------------------------------------------------------------
			End download callback
----------------------------------------------------------------------------*/
class SavingCtrlObj::_EndDownloadCallback : public CurlLoop::FutureRequest::Callback
{
  DEB_CLASS_NAMESPC(DebModCamera,"SavingCtrlObj","_EndDownloadCallback");
public:
  _EndDownloadCallback(SavingCtrlObj&,const std::string &filename);

  virtual void status_changed(CurlLoop::FutureRequest::Status,
			      std::string error);
private:
  SavingCtrlObj&	m_saving;
  std::string		m_filename;
};

/*----------------------------------------------------------------------------
			    SavingCtrlObj
----------------------------------------------------------------------------*/
SavingCtrlObj::~SavingCtrlObj()
{
  delete m_polling_thread;
}

void SavingCtrlObj::getPossibleSaveFormat(std::list<std::string> &format_list) const
{
  format_list.push_back(HwSavingCtrlObj::HDF5_FORMAT_STR);
}

void SavingCtrlObj::setCommonHeader(const HwSavingCtrlObj::HeaderMap& header)
{
  DEB_MEMBER_FUNCT();

  MultiParamRequest synchro(m_cam);

  for(HwSavingCtrlObj::HeaderMap::const_iterator i = header.begin();
      i != header.end();++i)
    {
      std::map<std::string,int>::iterator header_index = m_availables_header_keys.find(i->first);
      if(header_index == m_availables_header_keys.end())
	THROW_HW_ERROR(Error) << "Header key: " << i->first << " not yet managed ";
      synchro.addSet(Requests::PARAM_NAME(header_index->second),i->second);
    }

  synchro.wait();
}

void SavingCtrlObj::resetCommonHeader()
{
  // todo
}

void SavingCtrlObj::setSerieId(int value)
{
  DEB_MEMBER_FUNCT();
  DEB_PARAM() << DEB_VAR1(value);

  AutoMutex lock(m_cond.mutex());
  m_serie_id = value;
}

SavingCtrlObj::Status SavingCtrlObj::getStatus()
{
  DEB_MEMBER_FUNCT();
  AutoMutex lock(m_cond.mutex());
  bool status = m_poll_master_file ||
    (m_nb_file_to_watch != m_nb_file_transfer_started);
  DEB_RETURN() << DEB_VAR2(status,m_error_msg);
  if(m_error_msg.empty())
    return status ? RUNNING : IDLE;
  else
    return ERROR;
}

void SavingCtrlObj::stop()
{
  DEB_MEMBER_FUNCT();
  AutoMutex lock(m_cond.mutex());
  m_nb_file_transfer_started = m_nb_file_to_watch = 0;
  m_poll_master_file = false;
}

void SavingCtrlObj::_setActive(bool active, int)
{
  DEB_MEMBER_FUNCT();

  const char *active_str = active ? "enabled" : "disabled";
  DEB_TRACE() << "FILEWRITER_MODE:" << DEB_VAR1(active_str);
  setEigerParam(m_cam,Requests::FILEWRITER_MODE,active_str);
}

void SavingCtrlObj::_prepare(int)
{
  DEB_MEMBER_FUNCT();

  MultiParamRequest synchro(m_cam);

  int frames_per_file = int(m_frames_per_file);
  DEB_TRACE() << "NIMAGES_PER_FILE:" << DEB_VAR1(frames_per_file);
  synchro.addSet(Requests::NIMAGES_PER_FILE,frames_per_file);
  DEB_TRACE() << "FILEWRITER_NAME_PATTERN" << DEB_VAR1(m_prefix);
  synchro.addSet(Requests::FILEWRITER_NAME_PATTERN,m_prefix);
  synchro.wait();

  AutoMutex lock(m_cond.mutex());
  m_nb_file_transfer_started = m_nb_file_to_watch = 0;
  m_poll_master_file = true;
}

void SavingCtrlObj::_start(int)
{
  DEB_MEMBER_FUNCT();
  DEB_PARAM() << DEB_VAR1(m_active);

  int nb_frames;	m_cam.getNbFrames(nb_frames);
  double expo_time;	m_cam.getExpTime(expo_time);

  AutoMutex lock(m_cond.mutex());
  m_nb_file_transfer_started = 0;
  m_nb_file_to_watch = nb_frames / m_frames_per_file;
  if(nb_frames % m_frames_per_file) ++m_nb_file_to_watch;

  m_waiting_time = (expo_time * std::min(nb_frames,int(m_frames_per_file))) / 2.;
  
  m_cond.broadcast();
  
  DEB_TRACE() << DEB_VAR2(m_nb_file_to_watch,m_waiting_time);
}
//----------------------------------------------------------------------------
//
//----------------------------------------------------------------------------

SavingCtrlObj::_PollingThread::_PollingThread(SavingCtrlObj& saving,
					      eigerapi::Requests* requests) :
  m_saving(saving),
  m_requests(requests)
{
  pthread_attr_setscope(&m_thread_attr,PTHREAD_SCOPE_PROCESS);
}

SavingCtrlObj::_PollingThread::~_PollingThread()
{
  AutoMutex lock(m_saving.m_cond.mutex());
  m_saving.m_quit = true;
  m_saving.m_cond.broadcast();
  lock.unlock();

  join();
}

void SavingCtrlObj::_PollingThread::threadFunction()
{
  DEB_MEMBER_FUNCT();

  Camera::ApiGeneration api;
  m_saving.m_cam.getApiGeneration(api);
  Requests::PARAM_NAME ls_name = ((api == Camera::Eiger1) ? Requests::FILEWRITER_LS :
							    Requests::FILEWRITER_LS2);

  AutoMutex lock(m_saving.m_cond.mutex());
  
  while(!m_saving.m_quit)
    {
      while(!m_saving.m_quit &&
	    (m_saving.m_concurrent_download >= MAX_SIMULTANEOUS_DOWNLOAD ||
	     (!m_saving.m_poll_master_file &&
	      (m_saving.m_nb_file_to_watch == 
	       m_saving.m_nb_file_transfer_started))))
	{
	  m_saving.m_cond.broadcast();
	  m_saving.m_cond.wait();
	}

      if(m_saving.m_quit) break;
      std::string prefix = m_saving.m_prefix;
      std::string directory = m_saving.m_directory;

      char id_str[] = "$id";
      size_t id_pos = prefix.find(id_str);
      if (id_pos != std::string::npos) {
	char serie_id_str[32];
	snprintf(serie_id_str, sizeof(serie_id_str), "%d", m_saving.m_serie_id);
	std::string aux = prefix.substr(0, id_pos) + serie_id_str;
	size_t id_end = id_pos + sizeof(id_str);
	if (prefix.size() > id_end)
	  aux += prefix.substr(id_end);
	prefix = aux;
      }
      DEB_TRACE() << DEB_VAR2(directory, prefix);

      int total_nb_frames; m_saving.m_cam.getNbFrames(total_nb_frames);
      
      int frames_per_file = m_saving.m_frames_per_file;

      //Ls request
      std::vector<std::string> files;
      {
	AutoMutexUnlock u(lock);
	getEigerParam(m_saving.m_cam,ls_name,files);
      }

      // try to download master file
      if(m_saving.m_poll_master_file)
	{
	  std::ostringstream src_file_name;
	  src_file_name << prefix <<  "_master.h5";
	  bool master_file_found = false;
	  for(std::vector<std::string>::iterator i = files.begin();
	      !master_file_found && i != files.end();++i)
	    master_file_found = *i == src_file_name.str();

	  if(master_file_found)
	    {
	      std::string master_file_name = prefix + "_master.h5";
	      std::string dest_path = directory + "/" + master_file_name;
	      TransferReq master_file_req;
	      startEigerTransfer(m_saving.m_cam,src_file_name.str(),
				 dest_path,lock,master_file_req);
	      if (!master_file_req) {
		// stop the loop
		m_saving.m_nb_file_to_watch = m_saving.m_nb_file_transfer_started = 0;
		continue;
	      }
	      CallbackPtr end_cbk(new _EndDownloadCallback(m_saving,src_file_name.str()));
	      {
		AutoMutexUnlock u(lock);
		master_file_req->register_callback(end_cbk);
	      }
	      m_saving.m_poll_master_file = false;
	      ++m_saving.m_concurrent_download;
	    }
	}
      
      if(m_saving.m_nb_file_transfer_started < m_saving.m_nb_file_to_watch)
	{
	  int next_file_nb = m_saving.m_nb_file_transfer_started + 1;

	  std::sort(files.begin(),files.end());
	  char file_nb[32];
	  snprintf(file_nb,sizeof(file_nb),"%.6d",next_file_nb);

	  std::ostringstream src_file_name;
	  src_file_name << prefix << "_data_"  << file_nb << ".h5";
	  
	  //init find the first file_name of the list
	  std::vector<std::string>::iterator file_name = files.begin();
	  for(;file_name != files.end();++file_name)
	    if(*file_name == src_file_name.str()) break;

	  for(;file_name != files.end() &&
		m_saving.m_concurrent_download < MAX_SIMULTANEOUS_DOWNLOAD;
	      ++file_name,++next_file_nb)
	    {

	      snprintf(file_nb,sizeof(file_nb),"%.6d",next_file_nb);
	      src_file_name.clear();src_file_name.seekp(0);
	      src_file_name << prefix << "_data_"
			    << file_nb << ".h5";
	      if(*file_name == src_file_name.str()) // will start the transfer
		{
		  if(next_file_nb > m_saving.m_nb_file_to_watch)
		    {
		      DEB_WARNING() << "Something weird happened " 
				    << DEB_VAR2(next_file_nb,m_saving.m_nb_file_to_watch);
		      break;
		    }

		  DEB_TRACE() << "Start transfer file: " << DEB_VAR1(*file_name);
		  std::string dest_path = directory + "/" + src_file_name.str();
		  TransferReq file_req;
		  startEigerTransfer(m_saving.m_cam,src_file_name.str(),
				     dest_path, lock,file_req);
		  if (!file_req) {
		    // stop the loop
		    m_saving.m_nb_file_to_watch = m_saving.m_nb_file_transfer_started = 0;
		    break;
		  }
		  ++m_saving.m_nb_file_transfer_started,++m_saving.m_concurrent_download;
		  CallbackPtr end_cbk(new _EndDownloadCallback(m_saving,src_file_name.str()));
		  {
		    AutoMutexUnlock u(lock);
		    file_req->register_callback(end_cbk);
		  }

		  if(m_saving.m_callback)
		    {
		      int written_frame = m_saving.m_nb_file_transfer_started * frames_per_file;
		      if(written_frame > total_nb_frames)
			written_frame = total_nb_frames;
		      
		      //lima index start at 0
		      --written_frame;
		      bool continueFlag;
		      {
			AutoMutexUnlock u(lock);
			continueFlag = m_saving.m_callback->newFrameWritten(written_frame);
		      }
		      if(!continueFlag) // stop the loop
			m_saving.m_nb_file_to_watch = m_saving.m_nb_file_transfer_started = 0;
		    }
		}
	      else
		break;
	    }
	}

      m_saving.m_cond.wait(m_saving.m_waiting_time);
    }
}

void SavingCtrlObj::_download_finished(std::string filename, bool ok,
				       std::string error)
{
  DEB_MEMBER_FUNCT();
  DEB_PARAM() << DEB_VAR3(filename, ok, error);

  m_cam.newFrameAcquired();

  AutoMutex lock(m_cond.mutex());
  if(!ok)
    {
      m_error_msg = "Failed to download file: ";
      m_error_msg += filename;
      DEB_ERROR() << m_error_msg << ": " << error;
      //Stop the polling
      m_poll_master_file = false;
      m_nb_file_transfer_started = m_nb_file_to_watch = 0;
    }

  --m_concurrent_download;
  m_cond.broadcast();
}

/*----------------------------------------------------------------------------
		      class _EndDownloadCallback
----------------------------------------------------------------------------*/
SavingCtrlObj::_EndDownloadCallback::_EndDownloadCallback(SavingCtrlObj& saving,
							  const std::string& filename) :
  m_saving(saving),
  m_filename(filename)
{
}

void SavingCtrlObj::_EndDownloadCallback::
status_changed(CurlLoop::FutureRequest::Status status, std::string error)
{
  DEB_MEMBER_FUNCT();
  DEB_PARAM() << DEB_VAR2(status, error);
  bool ok = (status == CurlLoop::FutureRequest::OK);
  m_saving._download_finished(m_filename, ok, error);
}
