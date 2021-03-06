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
#include "EigerInterface.h"
#include "EigerCamera.h"
#include "EigerDetInfoCtrlObj.h"
#include "EigerSyncCtrlObj.h"
#include "EigerEventCtrlObj.h"
#include "EigerSavingCtrlObj.h"
#include "EigerStream.h"
#include "EigerDecompress.h"

using namespace lima;
using namespace lima::Eiger;
using namespace std;


//-----------------------------------------------------
// @brief Ctor
//-----------------------------------------------------
Interface::Interface(Camera& cam) : m_cam(cam) 
{
  DEB_CONSTRUCTOR();
  m_det_info = new DetInfoCtrlObj(cam);
  m_cap_list.push_back(HwCap(m_det_info));

  m_sync     = new SyncCtrlObj(cam);
  m_cap_list.push_back(HwCap(m_sync));

  m_saving = new SavingCtrlObj(cam);
  m_cap_list.push_back(HwCap(m_saving));

  m_event = new EventCtrlObj(cam);
  m_cap_list.push_back(HwCap(m_event));

  m_stream = new Stream(cam);
  
  HwBufferCtrlObj* buffer = m_stream->getBufferCtrlObj();
  m_cap_list.push_back(HwCap(buffer));	

  m_decompress = new Decompress(*m_stream);
  m_cap_list.push_back(HwCap(m_decompress));
}

//-----------------------------------------------------
// @brief Dtor
//-----------------------------------------------------
Interface::~Interface()
{
    DEB_DESTRUCTOR();
    delete m_det_info;
    delete m_sync;
    delete m_saving;
    delete m_stream;
    delete m_decompress;
}

//-----------------------------------------------------
// @brief return the capability list
//-----------------------------------------------------
void Interface::getCapList(HwInterface::CapList &cap_list) const
{
    DEB_MEMBER_FUNCT();
    cap_list = m_cap_list;
}

//-----------------------------------------------------
// @brief reset the interface, stop the acqisition
//-----------------------------------------------------
void Interface::reset(ResetLevel reset_level)
{
    DEB_MEMBER_FUNCT();
    DEB_PARAM() << DEB_VAR1(reset_level);

    stopAcq();
}

//-----------------------------------------------------
// @brief do nothing
//-----------------------------------------------------
void Interface::prepareAcq()
{
    DEB_MEMBER_FUNCT();

    if (m_cam.getStatus() == Camera::Armed)
      m_cam.disarm();

    bool use_filewriter = m_saving->isActive(); 
    m_stream->setActive(!use_filewriter);
    m_decompress->setActive(!use_filewriter);

    m_stream->release_all_msgs();
    m_stream->resetStatistics();

    try {
      m_cam.prepareAcq();
      int serie_id; m_cam.getSerieId(serie_id);
      m_saving->setSerieId(serie_id);
      if (!use_filewriter) {
	double stream_armed_timeout = 5.0;
	m_stream->waitArmed(stream_armed_timeout);
      }
    } catch (...) {
      m_saving->stop();
      m_stream->stop();
      throw;
    }
}

//-----------------------------------------------------
// @brief start the camera acquisition
//-----------------------------------------------------
void Interface::startAcq()
{
    DEB_MEMBER_FUNCT();
    TrigMode trig_mode;
    m_cam.getTrigMode(trig_mode);
    int nb_trig_frames;
    m_cam.getNbTriggeredFrames(nb_trig_frames);
    // start data retrieval subsystems only in first call
    if ((trig_mode != IntTrigMult) || (nb_trig_frames == 0)) {
      // either we use eiger saving or the raw stream
      if(m_saving->isActive())
	m_saving->start();
      else
	m_stream->start();
    }

    m_cam.startAcq();
}

//-----------------------------------------------------
// @brief stop the camera acquisition
//-----------------------------------------------------
void Interface::stopAcq()
{
  DEB_MEMBER_FUNCT();
  m_cam.stopAcq();
  m_saving->stop();
  m_stream->stop();
}

//-----------------------------------------------------
// @brief return the status of detector/acquisition
//-----------------------------------------------------
void Interface::getStatus(StatusType& status)
{
    DEB_MEMBER_FUNCT();
    
    Camera::Status Eiger_status;
      
    Eiger_status = m_cam.getStatus();
    switch (Eiger_status)
    {
      case Camera::Ready:
	{
	  bool mult_trig_in_progress = false;
	  TrigMode trig_mode;
	  m_cam.getTrigMode(trig_mode);
	  if (trig_mode == IntTrigMult) {
	    int tot_nb_frames, nb_trig_frames;
	    m_cam.getNbFrames(tot_nb_frames);
	    m_cam.getNbTriggeredFrames(nb_trig_frames);
	    mult_trig_in_progress = (nb_trig_frames != tot_nb_frames);
	  }
	  if (mult_trig_in_progress)
	    status.set(HwInterface::StatusType::Ready);
	  else if (m_saving->isActive())
	    {
	      SavingCtrlObj::Status saving_status = m_saving->getStatus();
	      switch(saving_status)
		{
		case SavingCtrlObj::IDLE:
		  status.set(HwInterface::StatusType::Ready);break;
		case SavingCtrlObj::RUNNING:
		  status.set(HwInterface::StatusType::Readout);break;
		default:
		  status.set(HwInterface::StatusType::Fault);break;
		}
	    }
	  else if (m_stream->isRunning())
	    status.set(HwInterface::StatusType::Readout);
	  else
	    status.set(HwInterface::StatusType::Ready);
	}
        break;

      case Camera::Exposure:
        status.set(HwInterface::StatusType::Exposure);
        break;

      case Camera::Armed:
        status.set(HwInterface::StatusType::Ready);
        break;

      case Camera::Fault:
        status.set(HwInterface::StatusType::Fault);
        break;
        
      case Camera::Initializing:
	status.set(HwInterface::StatusType::Config);
	break;
    }
    
    DEB_RETURN() << DEB_VAR1(status);
}

//-----------------------------------------------------
// @brief return the hw number of acquired frames
//-----------------------------------------------------
int Interface::getNbHwAcquiredFrames()
{
     DEB_MEMBER_FUNCT();
     int acq_frames;
     m_cam.getNbHwAcquiredFrames(acq_frames);
     return acq_frames;
}

void Interface::getLastStreamInfo(StreamInfo& last_info)
{
     DEB_MEMBER_FUNCT();
     m_stream->getLastStreamInfo(last_info);
}

void Interface::latchStreamStatistics(StreamStatistics& stat, bool reset)
{
     DEB_MEMBER_FUNCT();
     m_stream->latchStatistics(stat, reset);
}

