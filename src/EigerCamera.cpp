///###########################################################################
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
//############################################################################

#include <sstream>
#include <iostream>
#include <string>
#include <math.h>
#include <algorithm>
#include <pthread.h>
#include "EigerCamera.h"
#include "EigerCameraRequests.h"
#include "lima/Timestamp.h"

using namespace lima;
using namespace lima::Eiger;
using namespace std;
using namespace eigerapi;

#define sendCommand(cmd)			\
  sendEigerCommand(*this, cmd)

#define sendCommandTimeout(cmd, timeout)	\
  sendEigerCommandTimeout(*this, cmd, timeout)

#define setParam(param, value)			\
  setEigerParam(*this, param, value)

#define setCachedParam(param, cache, value)	\
  setEigerCachedParam(*this, param, cache, value)

#define setCachedParamForce(param, cache, value, force)	\
  setEigerCachedParamForce(*this, param, cache, value, force)

#define getParam(param, value)			\
  getEigerParam(*this, param, value)


/*----------------------------------------------------------------------------
			    Callback class
 ----------------------------------------------------------------------------*/
class Camera::TriggerCallback : public Callback
{
  DEB_CLASS_NAMESPC(DebModCamera, "Camera::TriggerCallback", "Eiger");
public:
  TriggerCallback(Camera& cam, bool disarm, double duration)
    : m_cam(cam), m_disarm(disarm), m_duration(duration),
      m_start_ts(Timestamp::now())
  {}

  void status_changed(CurlLoop::FutureRequest::Status status,
		      std::string error)
  {
    DEB_MEMBER_FUNCT();
    DEB_PARAM() << DEB_VAR2(status, error);
    bool ok = (status == CurlLoop::FutureRequest::OK);
    if (!ok) {
      // the HTTP server can close the connection without completing
      // trigger commands longer than 5 min. a retry (by libcurl) fails
      const double timeout_limit = 5 * 60;
      bool long_trigger = (m_duration > timeout_limit - 1);
      // if the command lasted the expected duration, ignore the error
      double elapsed = Timestamp::now() - m_start_ts;
      if (long_trigger && (fabs(elapsed - m_duration) < 2)) {
	DEB_ALWAYS() << "Ignoring trigger command error";
	ok = true;
      }
    }
    if (!ok)
      DEB_ERROR() << DEB_VAR1(error); 
    m_cam._trigger_finished(ok, m_disarm);
  }
private:
  Camera& m_cam;
  bool m_disarm;
  double m_duration;
  Timestamp m_start_ts;
};

class Camera::InitCallback : public Callback
{	
  DEB_CLASS_NAMESPC(DebModCamera, "Camera::InitCallback", "Eiger");
public:
  InitCallback(Camera& cam) : m_cam(cam) {}

  void status_changed(CurlLoop::FutureRequest::Status status,
		      std::string error)
  {
    DEB_MEMBER_FUNCT();
    DEB_PARAM() << DEB_VAR2(status, error);
    bool ok = (status == CurlLoop::FutureRequest::OK);
    if (!ok)
      DEB_ERROR() << DEB_VAR1(error); 
    m_cam._initialization_finished(ok);
  }

private:
  Camera& m_cam;
};

//-----------------------------------------------------------------------------
///  Ctor
//-----------------------------------------------------------------------------
Camera::Camera(const std::string& host, int http_port, int stream_port)	///< [in] Ip address of the detector server
  : 		m_frames_triggered(0),
		m_frames_acquired(0),
                m_latency_time(0.),
                m_detectorImageType(Bpp16),
		m_initialize_state(IDLE),
		m_trigger_state(IDLE),
		m_armed(false),
		m_serie_id(0),
                m_exp_time(1.),
                m_detector_host(host),
                m_detector_http_port(http_port),
                m_detector_stream_port(stream_port)
{
    DEB_CONSTRUCTOR();
    DEB_PARAM() << DEB_VAR1(host);

    std::string http_address = host + ":" + std::to_string(http_port);
    m_requests = new Requests(http_address);

    // Detect EigerAPI version
    m_api_version = m_requests->get_api_version();
    DEB_TRACE() << DEB_VAR1(m_api_version);
    if (m_api_version == "1.6.0")
      m_api = Eiger1;
    else if (m_api_version == "1.8.0")
      m_api = Eiger2;
    else
      THROW_HW_ERROR(Error) << "Unknown " << DEB_VAR1(m_api_version);
    DEB_TRACE() << DEB_VAR1(m_api);

    // Init EigerAPI
    try {
      std::string status = getCamStatus();
      DEB_TRACE() << DEB_VAR1(status);
      if ((status != "idle") && (status != "ready"))
	THROW_HW_ERROR(Error) << "Camera is not idle/ready. "
			      << "Forcing initialization";
      _synchronize();
    } catch(Exception& e) {
      DEB_ALWAYS() << "Could not get configuration parameters, try to initialize";
      initialize();
      AutoMutex lock(m_cond.mutex());
      while (m_initialize_state == RUNNING)
	m_cond.wait();
    }

    // Display max image size
    DEB_TRACE() << "Detector max width: " << m_maxImageWidth;
    DEB_TRACE() << "Detector max height:" << m_maxImageHeight;

    // --- Set detector for software single image mode    
    setTrigMode(IntTrig);

    m_nb_frames = 1;

}


//-----------------------------------------------------------------------------
///  Dtor
//-----------------------------------------------------------------------------
Camera::~Camera()
{
    DEB_DESTRUCTOR();
    delete m_requests;
}


//----------------------------------------------------------------------------
// initialize detector
//----------------------------------------------------------------------------
void Camera::initialize()
{
  DEB_MEMBER_FUNCT();
  // Finally initialize the detector
  AutoMutex lock(m_cond.mutex());
  DEB_ALWAYS() << "Initializing detector ... ";
  m_initialize_state = RUNNING;
  CommandReq async_initialize = m_requests->get_command(Requests::INITIALIZE);
  lock.unlock();

  CallbackPtr cbk(new InitCallback(*this));
  async_initialize->register_callback(cbk, true);
}

void Camera::_initialization_finished(bool ok)
{
  DEB_MEMBER_FUNCT();
  DEB_PARAM() << DEB_VAR1(ok);

  const char *status_desc = ok ? "OK" : "Error";
  DEB_ALWAYS() << "Initialize finished: " << status_desc;

  if (ok) {
    try {
      DEB_ALWAYS() << "Synchronizing with detector ...";
      _synchronize();
      DEB_ALWAYS() << "Done!";
    } catch (const Exception& e) {
      ok = false;
    }
  }
    
  AutoMutex lock(m_cond.mutex());
  m_initialize_state = ok ? Camera::IDLE : Camera::ERROR;
  m_cond.signal();
}

//-----------------------------------------------------------------------------
/// Set detector for single image acquisition
//-----------------------------------------------------------------------------
void Camera::prepareAcq()
{
  DEB_MEMBER_FUNCT();
  AutoMutex aLock(m_cond.mutex());
  if(m_armed)
    THROW_HW_ERROR(Error) << "Camera already armed";
  
  unsigned nb_images, nb_triggers;
  switch(m_trig_mode)
    {
    case IntTrig:
    case ExtTrigSingle:
      nb_images = m_nb_frames, nb_triggers = 1; break;
    case IntTrigMult:
    case ExtTrigMult:
    case ExtGate:
      nb_images = 1, nb_triggers = m_nb_frames; break;
    default:
      THROW_HW_ERROR(Error) << "Very weird can't be in this case";
    }
  double frame_time = m_exp_time + m_latency_time;
  if(frame_time < m_min_frame_time)
    {    
      if(m_latency_time <= m_readout_time)
	frame_time = m_min_frame_time;
      else
	THROW_HW_ERROR(Error) << "This detector can't go at this frame rate (" << 1 / frame_time
			      << ") is limited to (" << 1 / m_min_frame_time << ")";
    }

  DEB_PARAM() << DEB_VAR3(frame_time, nb_images, nb_triggers);

  setCachedParam(Requests::FRAME_TIME, m_frame_time, frame_time);
  setCachedParam(Requests::NIMAGES, m_nb_images, nb_images);
  setCachedParam(Requests::NTRIGGER, m_nb_triggers, nb_triggers);

  DEB_TRACE() << "Arm start";
  double timeout = 5 * 60.; // 5 min timeout
  CommandReq arm_cmd = sendCommandTimeout(Requests::ARM, timeout);
  DEB_TRACE() << "Arm end";
  m_armed = true;
  try {
    m_serie_id = arm_cmd->get_serie_id();
  } catch(const eigerapi::EigerException &e) {
    HANDLE_EIGERERROR(arm_cmd, e);
  }
  m_frames_triggered = m_frames_acquired = 0;
}


//-----------------------------------------------------------------------------
///  start the acquisition
//-----------------------------------------------------------------------------
void Camera::startAcq()
{
  DEB_MEMBER_FUNCT();
  AutoMutex lock(m_cond.mutex());

  if((m_trig_mode == IntTrig) || (m_trig_mode == IntTrigMult))
    {
      CommandReq trigger = m_requests->get_command(Requests::TRIGGER);
      m_trigger_state = RUNNING;
      m_frames_triggered += m_nb_images;
      bool disarm_at_end = (m_frames_triggered == m_nb_frames);
      DEB_TRACE() << "Trigger start: " << DEB_VAR1(disarm_at_end);
      double duration = m_nb_images * m_frame_time;
      AutoMutexUnlock u(lock);
      CallbackPtr cbk(new TriggerCallback(*this, disarm_at_end, duration));
      trigger->register_callback(cbk, disarm_at_end);
    }
  
}


//-----------------------------------------------------------------------------
/// stop the acquisition
//-----------------------------------------------------------------------------
void Camera::stopAcq()
{
  DEB_MEMBER_FUNCT();
  AutoMutex lock(m_cond.mutex());
  if (!m_armed)
    return;

  // Ongoing Trigger callback might run, avoid Disarm and potential deadlock
  m_armed = false;
  lock.unlock();
  DEB_TRACE() << "Aborting";
  sendCommand(Requests::ABORT);
}


//-----------------------------------------------------------------------------
/// return the detector Max image size 
//-----------------------------------------------------------------------------
void Camera::getDetectorMaxImageSize(Size& size) ///< [out] image dimensions
{
	DEB_MEMBER_FUNCT();
	size = Size(m_maxImageWidth, m_maxImageHeight);
}


//-----------------------------------------------------------------------------
/// return the detector image size 
//-----------------------------------------------------------------------------
void Camera::getDetectorImageSize(Size& size) ///< [out] image dimensions
{
  DEB_MEMBER_FUNCT();

  MultiParamRequest synchro(*this);
  unsigned int width, height;
  synchro.addGet(Requests::DETECTOR_WITDH, width);
  synchro.addGet(Requests::DETECTOR_HEIGHT, height);
  synchro.wait();
  size = Size(width, height);
}


//-----------------------------------------------------------------------------
/// Get the image type
//-----------------------------------------------------------------------------
void Camera::getImageType(ImageType& type) ///< [out] image type
{
    DEB_MEMBER_FUNCT();

    type = m_detectorImageType;    
}


//-----------------------------------------------------------------------------
//! Camera::setImageType()
//-----------------------------------------------------------------------------
void Camera::setImageType(ImageType type) ///< [in] image type
{
	DEB_MEMBER_FUNCT();
	DEB_TRACE() << "Camera::setImageType - " << DEB_VAR1(type);
}


//-----------------------------------------------------------------------------
/// return the detector type
//-----------------------------------------------------------------------------
void Camera::getDetectorType(string& type) ///< [out] detector type
{
  DEB_MEMBER_FUNCT();

  type = m_detector_type;
}


//-----------------------------------------------------------------------------
/// return the detector model
//-----------------------------------------------------------------------------
void Camera::getDetectorModel(string& model) ///< [out] detector model
{
  DEB_MEMBER_FUNCT();

  model = m_detector_model;
}


//-----------------------------------------------------------------------------
/// Checks trigger mode
/*!
@return true if the given trigger mode is supported
*/
//-----------------------------------------------------------------------------
bool Camera::checkTrigMode(TrigMode trig_mode) ///< [in] trigger mode to check
{
  DEB_MEMBER_FUNCT();
  DEB_PARAM() << DEB_VAR1(trig_mode);
  switch(trig_mode)
    {
    case IntTrig:
    case IntTrigMult:
    case ExtTrigSingle:
    case ExtTrigMult:
    case ExtGate:
      return true;
    default:
      return false;
    }
}


//-----------------------------------------------------------------------------
/// Set the new trigger mode
//-----------------------------------------------------------------------------
void Camera::setTrigMode(TrigMode trig_mode) ///< [in] lima trigger mode to set
{
  DEB_MEMBER_FUNCT();
  DEB_PARAM() << DEB_VAR1(trig_mode);

  const char *trig_name;
  switch(trig_mode)
    {
    case IntTrig:
    case IntTrigMult:
      trig_name = "ints";break;
    case ExtTrigSingle:
    case ExtTrigMult:
      trig_name = "exts";break;
    case ExtGate:
      trig_name = "exte";break;
    default:
      THROW_HW_ERROR(NotSupported) << DEB_VAR1(trig_mode);
    }

  setCachedParam(Requests::TRIGGER_MODE, m_trig_mode_name, trig_name);
  m_trig_mode = trig_mode;
}


//-----------------------------------------------------------------------------
/// Get the current trigger mode
//-----------------------------------------------------------------------------
void Camera::getTrigMode(TrigMode& mode) ///< [out] current trigger mode
{
  DEB_MEMBER_FUNCT();
  mode = m_trig_mode;

  DEB_RETURN() << DEB_VAR1(mode);
}


//-----------------------------------------------------------------------------
/// Set the new exposure time
//-----------------------------------------------------------------------------
void Camera::setExpTime(double exp_time, ///< [in] exposure time to set
			bool force)
{
  DEB_MEMBER_FUNCT();
  DEB_PARAM() << DEB_VAR1(exp_time);

  setCachedParamForce(Requests::EXPOSURE, m_exp_time, exp_time, force);
}


//-----------------------------------------------------------------------------
/// Get the current exposure time
//-----------------------------------------------------------------------------
void Camera::getExpTime(double& exp_time) ///< [out] current exposure time
{
 DEB_MEMBER_FUNCT();

 exp_time = m_exp_time;

 DEB_RETURN() << DEB_VAR1(exp_time);
}


//-----------------------------------------------------------------------------
/// Set the new latency time between images
//-----------------------------------------------------------------------------
void Camera::setLatTime(double lat_time) ///< [in] latency time
{
  DEB_MEMBER_FUNCT();
  DEB_PARAM() << DEB_VAR1(lat_time);

  bool force_exp = (lat_time != m_latency_time);
  m_latency_time = lat_time;
  setExpTime(m_exp_time, force_exp);
}


//-----------------------------------------------------------------------------
/// Get the current latency time
//-----------------------------------------------------------------------------
void Camera::getLatTime(double& lat_time) ///< [out] current latency time
{
  DEB_MEMBER_FUNCT();
  
  lat_time = m_latency_time;

  DEB_RETURN() << DEB_VAR1(lat_time);
}


//-----------------------------------------------------------------------------
/// Get the exposure time range
//-----------------------------------------------------------------------------
void Camera::getExposureTimeRange(double& min_expo,	///< [out] minimum exposure time
                                  double& max_expo)   ///< [out] maximum exposure time
{
  DEB_MEMBER_FUNCT();
  double curr_expo;
  ParamReq exp_time = getParam(Requests::EXPOSURE, curr_expo);
  try {
    min_expo = exp_time->get_min().data.double_val;
    if (m_api == Eiger1)
      max_expo = exp_time->get_max().data.double_val;
    else
      max_expo = 1800;
  } catch(const eigerapi::EigerException &e) {
    HANDLE_EIGERERROR(exp_time, e);
  }
  DEB_RETURN() << DEB_VAR2(min_expo, max_expo);
}


//-----------------------------------------------------------------------------
///  Get the latency time range
//-----------------------------------------------------------------------------
void Camera::getLatTimeRange(double& min_lat, ///< [out] minimum latency
                             double& max_lat) ///< [out] maximum latency
{
  DEB_MEMBER_FUNCT();

    // --- no info on min latency
  min_lat = m_readout_time;
  double min_exp,max_exp;
  getExposureTimeRange(min_exp,max_exp);
  max_lat = max_exp;

  DEB_RETURN() << DEB_VAR2(min_lat, max_lat);
}


//-----------------------------------------------------------------------------
/// Set the number of frames to be taken
//-----------------------------------------------------------------------------
void Camera::setNbFrames(int nb_frames) ///< [in] number of frames to take
{
    DEB_MEMBER_FUNCT();
    DEB_PARAM() << DEB_VAR1(nb_frames);

    if (0==nb_frames)
      THROW_HW_ERROR(NotSupported) << "video mode is not supported";

    m_nb_frames = nb_frames;
}


//-----------------------------------------------------------------------------
/// Get the number of frames to be taken
//-----------------------------------------------------------------------------
void Camera::getNbFrames(int& nb_frames) ///< [out] current number of frames to take
{
  DEB_MEMBER_FUNCT();
  nb_frames = m_nb_frames;
  DEB_RETURN() << DEB_VAR1(nb_frames);
}


//-----------------------------------------------------------------------------
/// Get the current acquired frames
//-----------------------------------------------------------------------------
void Camera::getNbHwAcquiredFrames(int &nb_acq_frames) ///< [out] number of acquired files
{ 
  DEB_MEMBER_FUNCT();
  AutoMutex lock(m_cond.mutex());
  nb_acq_frames = m_frames_acquired;
  DEB_RETURN() << DEB_VAR1(nb_acq_frames);
}

//-----------------------------------------------------------------------------
/// Get the current triggered frames
//-----------------------------------------------------------------------------
void Camera::getNbTriggeredFrames(int &nb_trig_frames) ///< [out] number of triggered files
{
  DEB_MEMBER_FUNCT();
  AutoMutex lock(m_cond.mutex());
  nb_trig_frames = m_frames_triggered;
  DEB_RETURN() << DEB_VAR1(nb_trig_frames);
}


//-----------------------------------------------------------------------------
/// Get the camera status
//-----------------------------------------------------------------------------
Camera::Status Camera::getStatus() ///< [out] current camera status
{
  DEB_MEMBER_FUNCT();
  AutoMutex lock(m_cond.mutex());
  return _getStatus();
}

Camera::Status Camera::_getStatus()
{
  DEB_MEMBER_FUNCT();

  Camera::Status status;
  if(m_initialize_state == ERROR ||
     m_trigger_state == ERROR)
    status = Fault;
  else if(m_trigger_state == RUNNING)
    status = Exposure;
  else if(m_armed && (m_frames_triggered == 0))
    status = Armed;
  else if(m_initialize_state == RUNNING)
    status = Initializing;
  else
    status = Ready;

  DEB_RETURN() << DEB_VAR1(status);
  return status;
}
//----------------------------------------------------------------------------
// Get camera hardware status
//----------------------------------------------------------------------------
std::string Camera::getCamStatus()
{
  DEB_MEMBER_FUNCT();
  std::string status;
  getParam(Requests::DETECTOR_STATUS,status);
  DEB_RETURN() << DEB_VAR1(status);
  return status;
}
//-----------------------------------------------------------------------------
/// Tells if binning is available
/*!
@return always false, hw binning mode is not supported
*/
//-----------------------------------------------------------------------------
bool Camera::isBinningAvailable()
{
  DEB_MEMBER_FUNCT();
  return false;
}


//-----------------------------------------------------------------------------
/// return the detector pixel size in meter
//-----------------------------------------------------------------------------
void Camera::getPixelSize(double& sizex,	///< [out] horizontal pixel size
                          double& sizey)	///< [out] vertical   pixel size
{
  DEB_MEMBER_FUNCT();

  sizex = m_x_pixelsize;
  sizey = m_y_pixelsize;
  DEB_RETURN() << DEB_VAR2(sizex, sizey); 
}


//-----------------------------------------------------------------------------
/// reset the camera, no hw reset available on Eiger camera
//-----------------------------------------------------------------------------
/*
void Camera::reset()
{
    DEB_MEMBER_FUNCT();
    return;
}
*/

//-----------------------------------------------------------------------------
///    synchronize with controller
//-----------------------------------------------------------------------------
void Camera::_synchronize()
{
  DEB_MEMBER_FUNCT();
  DEB_TRACE() << "_synchronize()";

  AutoMutex lock(m_cond.mutex());

  MultiParamRequest synchro(*this);

  synchro.addGet(Requests::TRIGGER_MODE, m_trig_mode_name);
  
  synchro.addGet(Requests::X_PIXEL_SIZE, m_x_pixelsize);
  synchro.addGet(Requests::Y_PIXEL_SIZE, m_y_pixelsize);

  synchro.addGet(Requests::DETECTOR_WITDH, m_maxImageWidth);
  synchro.addGet(Requests::DETECTOR_HEIGHT, m_maxImageHeight);

  double m_latency_time;
  synchro.addGet(Requests::DETECTOR_READOUT_TIME, m_latency_time);

  synchro.addGet(Requests::DESCRIPTION, m_detector_model);
  synchro.addGet(Requests::DETECTOR_NUMBER, m_detector_type);
  synchro.addGet(Requests::EXPOSURE, m_exp_time);
  synchro.addGet(Requests::NIMAGES, m_nb_images);
  
  synchro.addGet(Requests::NTRIGGER, m_nb_triggers);
  synchro.addGet(Requests::FRAME_TIME, m_frame_time);

  bool auto_summation;
  synchro.addGet(Requests::AUTO_SUMMATION, auto_summation);

  std::string compression_type;
  synchro.addGet(Requests::COMPRESSION_TYPE, compression_type);
  
  //Synchro
  synchro.wait();

  m_detectorImageType = auto_summation ? Bpp32 : Bpp16;
  _updateImageSize();

  //Trigger mode
  std::string trig_name = m_trig_mode_name.value();
  if(trig_name == "ints")
    m_trig_mode = m_nb_triggers > 1 ? IntTrigMult : IntTrig;
  else if(trig_name == "exts")
    m_trig_mode = m_nb_triggers > 1 ? ExtTrigMult : ExtTrigSingle;
  else if(trig_name == "exte")
    m_trig_mode = ExtGate;
  else
    THROW_HW_ERROR(InvalidValue) << "Unexpected trigger mode: "
				 << DEB_VAR1(trig_name);

  Requests::PARAM_NAME param;
  try {
    param = Requests::FRAME_TIME;
    m_min_frame_time = synchro[param]->get_min().data.double_val;
    param = Requests::DETECTOR_READOUT_TIME;
    ParamReq req = synchro[param];
    Requests::Param::Value value = req->get_min();
    m_readout_time = value.data.double_val;
    if (m_readout_time <= 0) {
      value = req->get();
      m_readout_time = value.data.double_val;
    }
  } catch (const EigerException& e) {
    HANDLE_EIGERERROR(synchro[param], e);
  }

  if (compression_type == "none")
    m_compression_type = NoCompression;
  else if (compression_type == "lz4")
    m_compression_type = LZ4;
  else if (compression_type == "bslz4")
    m_compression_type = BSLZ4;
  else
    THROW_HW_ERROR(InvalidValue) << "Unexpected compression type: "
				 << DEB_VAR1(compression_type);
}

//----------------------------------------------------------------------------
//	ImageSizeChanged hook
//----------------------------------------------------------------------------
void Camera::_updateImageSize()
{
  DEB_MEMBER_FUNCT();
  Size image_size;
  getDetectorImageSize(image_size);
  DEB_TRACE() << DEB_VAR2(image_size,m_detectorImageType);
  maxImageSizeChanged(image_size,m_detectorImageType);
}

/*----------------------------------------------------------------------------
	This method is called when the trigger is finished
  ----------------------------------------------------------------------------*/
void Camera::_trigger_finished(bool ok, bool do_disarm)
{
  DEB_MEMBER_FUNCT();
  DEB_PARAM() << DEB_VAR1(ok);
  
  DEB_TRACE() << "Trigger end";
  if(!ok) {
    DEB_ERROR() << "Error in trigger command";
  } else if(do_disarm) {
    try { disarm(); }
    catch (...) { ok = false; }
  }

  AutoMutex lock(m_cond.mutex());
  m_trigger_state = ok ? IDLE : ERROR;
}

void Camera::newFrameAcquired()
{
  DEB_MEMBER_FUNCT();
  AutoMutex lock(m_cond.mutex());
  ++m_frames_acquired;
  DEB_TRACE() << DEB_VAR1(m_frames_acquired);
}

bool Camera::allFramesAcquired()
{
  DEB_MEMBER_FUNCT();
  AutoMutex lock(m_cond.mutex());
  DEB_PARAM() << DEB_VAR2(m_frames_acquired, m_nb_frames);
  bool finished = (m_frames_acquired == m_nb_frames);
  DEB_RETURN() << DEB_VAR1(finished);
  return finished;
}

//-----------------------------------------------------------------------------
/// Returns the API generation of the detector
/*!
@return API generation (eiger1, eiger2)
*/
//-----------------------------------------------------------------------------
void Camera::getApiGeneration(ApiGeneration& api)
{
  DEB_MEMBER_FUNCT();
  api = m_api;
  DEB_RETURN() << DEB_VAR1(api);
}

//-----------------------------------------------------------------------------
/// Returns the API version of the detector
/*!
@return api version string (1.6.0/1.8.0)
*/
//-----------------------------------------------------------------------------
void Camera::getApiVersion(std::string& api)
{
  DEB_MEMBER_FUNCT();
  api = m_api_version;
  DEB_RETURN() << DEB_VAR1(api);
}
//-----------------------------------------------------------------------------
/// Returns the temperature of the detector
/*!
@return temperature value
*/
//-----------------------------------------------------------------------------
void Camera::getTemperature(double &temp)
{
  DEB_MEMBER_FUNCT();
  getParam(Requests::TEMP,temp);
}


//-----------------------------------------------------------------------------
/// Returns the humidity of the detector
/*!
@return humidity value
*/
//-----------------------------------------------------------------------------
void Camera::getHumidity(double &humidity)
{
  DEB_MEMBER_FUNCT();
  getParam(Requests::HUMIDITY,humidity);
}



//-----------------------------------------------------------------------------
/// Returns the high voltage state 
/*!
@return state string
*/
//-----------------------------------------------------------------------------
void Camera::getHighVoltageState(std::string &hvstate)
{
  DEB_MEMBER_FUNCT();
  getParam(Requests::HVSTATE,hvstate);
}

void Camera::resetHighVoltage()
{
  DEB_MEMBER_FUNCT();
  sendCommand(Requests::HV_RESET);
  DEB_TRACE() << "reset HighVoltage";
}

//-----------------------------------------------------------------------------
///  Count rate correction setter
//-----------------------------------------------------------------------------
void Camera::setCountrateCorrection(bool value) ///< [in] true:enabled, false:disabled
{
    DEB_MEMBER_FUNCT();
    setParam(Requests::COUNTRATE_CORRECTION,value);
}


//-----------------------------------------------------------------------------
///  Count rate correction getter
//-----------------------------------------------------------------------------
void Camera::getCountrateCorrection(bool& value)  ///< [out] true:enabled, false:disabled
{
  DEB_MEMBER_FUNCT();
  getParam(Requests::COUNTRATE_CORRECTION,value);
}


//-----------------------------------------------------------------------------
///  FlatfieldCorrection setter
//-----------------------------------------------------------------------------
void Camera::setFlatfieldCorrection(bool value) ///< [in] true:enabled, false:disabled
{
  DEB_MEMBER_FUNCT();
  setParam(Requests::FLATFIELD_CORRECTION,value);
}


//-----------------------------------------------------------------------------
///  FlatfieldCorrection getter
//-----------------------------------------------------------------------------
void Camera::getFlatfieldCorrection(bool& value) ///< [out] true:enabled, false:disabled
{
  DEB_MEMBER_FUNCT();
  getParam(Requests::FLATFIELD_CORRECTION,value);
}

//-----------------------------------------------------------------------------
///  Retrigger mode setter
//-----------------------------------------------------------------------------
void Camera::setRetrigger(bool value) ///< [in] true:enabled, false:disabled
{
    DEB_MEMBER_FUNCT();
    setParam(Requests::RETRIGGER,value);
}


//-----------------------------------------------------------------------------
///  Retrigger getter
//-----------------------------------------------------------------------------
void Camera::getRetrigger(bool& value)  ///< [out] true:enabled, false:disabled
{
  DEB_MEMBER_FUNCT();
  getParam(Requests::RETRIGGER,value);
}


//----------------------------------------------------------------------------
// Auto Summation setter
//----------------------------------------------------------------------------
void Camera::setAutoSummation(bool value)
{
  DEB_MEMBER_FUNCT();
  DEB_PARAM() << DEB_VAR1(value);
  setParam(Requests::AUTO_SUMMATION,value);
  m_detectorImageType = value ? Bpp32 : Bpp16;
  _updateImageSize();
}

//----------------------------------------------------------------------------
// Auto Summation getter
//----------------------------------------------------------------------------
void Camera::getAutoSummation(bool& value)
{
  DEB_MEMBER_FUNCT();
  getParam(Requests::AUTO_SUMMATION,value);
  DEB_RETURN() << DEB_VAR1(value);
}


//-----------------------------------------------------------------------------
///  PixelMask setter
//-----------------------------------------------------------------------------
void Camera::setPixelMask(bool value) ///< [in] true:enabled, false:disabled
{
    DEB_MEMBER_FUNCT();
    setParam(Requests::PIXEL_MASK,value);
}


//-----------------------------------------------------------------------------
///  PixelMask getter
//-----------------------------------------------------------------------------
void Camera::getPixelMask(bool& value) ///< [out] true:enabled, false:disabled
{
  DEB_MEMBER_FUNCT();
  getParam(Requests::PIXEL_MASK,value);
}

//-----------------------------------------------------------------------------
/// EfficiencyCorrection setter
//-----------------------------------------------------------------------------
void Camera::setEfficiencyCorrection(bool enabled) ///< [in] true:enabled, false:disabled
{
    DEB_MEMBER_FUNCT();
    if (m_api == Eiger1) {
        setParam(Requests::EFFICIENCY_CORRECTION,enabled);
    } else {
        DEB_WARNING() << "Efficiency correction is not implemented";
    }
}


//-----------------------------------------------------------------------------
/// EfficiencyCorrection getter
//-----------------------------------------------------------------------------
void Camera::getEfficiencyCorrection(bool& value)  ///< [out] true:enabled, false:disabled
{
  DEB_MEMBER_FUNCT();
  if (m_api == Eiger1) {
    getParam(Requests::EFFICIENCY_CORRECTION,value);
  } else {
    DEB_WARNING() << "Efficiency correction is not implemented";
    value = false;
  }
}


//-----------------------------------------------------------------------------
///  ThresholdEnergy setter
//-----------------------------------------------------------------------------
void Camera::setThresholdEnergy(double value)
{
  DEB_MEMBER_FUNCT();
  setParam(Requests::THRESHOLD_ENERGY,value);
}


//-----------------------------------------------------------------------------
///  ThresholdEnergy getter
//-----------------------------------------------------------------------------
void Camera::getThresholdEnergy(double& value)
{
  DEB_MEMBER_FUNCT();
  getParam(Requests::THRESHOLD_ENERGY,value);
}


//-----------------------------------------------------------------------------
///  ThresholdEnergy2 setter
//-----------------------------------------------------------------------------
void Camera::setThresholdEnergy2(double value)
{
  DEB_MEMBER_FUNCT();
  setParam(Requests::THRESHOLD_ENERGY2,value);
}


//-----------------------------------------------------------------------------
///  ThresholdEnergy getter
//-----------------------------------------------------------------------------
void Camera::getThresholdEnergy2(double& value)
{
  DEB_MEMBER_FUNCT();
  getParam(Requests::THRESHOLD_ENERGY2,value);
}


//-----------------------------------------------------------------------------
///  ThresholdEnergy2 setter
//-----------------------------------------------------------------------------
void Camera::setThresholdDiffMode(bool value)
{
  DEB_MEMBER_FUNCT();
  if (value) {
    setParam(Requests::THRESHOLD_MODE2,"enabled");
    setParam(Requests::THRESHOLD_DIFF_MODE,"enabled");
  } else {
    setParam(Requests::THRESHOLD_DIFF_MODE,"disabled");
    setParam(Requests::THRESHOLD_MODE2,"disabled");
  }
}


//-----------------------------------------------------------------------------
///  ThresholdEnergy2 setter
//-----------------------------------------------------------------------------
void Camera::getThresholdDiffMode(bool& value)
{
  DEB_MEMBER_FUNCT();
  std::string mode_str;

  getParam(Requests::THRESHOLD_DIFF_MODE,mode_str);
  if (mode_str == "enabled") {
    value = true;
  } else {
    value = false;
  }
}




//-----------------------------------------------------------------------------
///  VirtualPixelCorrection setter
//-----------------------------------------------------------------------------
void Camera::setVirtualPixelCorrection(bool value) ///< [in] true:enabled, false:disabled
{
  DEB_MEMBER_FUNCT();
  setParam(Requests::VIRTUAL_PIXEL_CORRECTION,value);
}


//-----------------------------------------------------------------------------
///  VirtualPixelCorrection getter
//-----------------------------------------------------------------------------
void Camera::getVirtualPixelCorrection(bool& value) ///< [out] true:enabled, false:disabled
{
  DEB_MEMBER_FUNCT();
  getParam(Requests::VIRTUAL_PIXEL_CORRECTION,value);
}


//-----------------------------------------------------------------------------
///  PhotonEnergy setter
//-----------------------------------------------------------------------------
void Camera::setPhotonEnergy(double value) ///< [in] true:enabled, false:disabled
{
    DEB_MEMBER_FUNCT();
    setParam(Requests::PHOTON_ENERGY,value);
}


//-----------------------------------------------------------------------------
///  PhotonEnergy getter
//-----------------------------------------------------------------------------
void Camera::getPhotonEnergy(double& value) ///< [out] true:enabled, false:disabled
{
  DEB_MEMBER_FUNCT();
  getParam(Requests::PHOTON_ENERGY,value);
}

//-----------------------------------------------------------------------------
///  Wavelength setter
//-----------------------------------------------------------------------------
void Camera::setWavelength(double value) ///< [in] true:enabled, false:disabled
{
    DEB_MEMBER_FUNCT();
    setParam(Requests::HEADER_WAVELENGTH,value);
}


//-----------------------------------------------------------------------------
///  Wavelength getter
//-----------------------------------------------------------------------------
void Camera::getWavelength(double& value) ///< [out] true:enabled, false:disabled
{
  DEB_MEMBER_FUNCT();
  getParam(Requests::HEADER_WAVELENGTH,value);
}


//-----------------------------------------------------------------------------
///  BeamCenterX setter
//-----------------------------------------------------------------------------
void Camera::setBeamCenterX(double value) ///< [in] 
{
    DEB_MEMBER_FUNCT();
    setParam(Requests::HEADER_BEAM_CENTER_X,value);
}

//-----------------------------------------------------------------------------
///  BeamCenterX getter
//-----------------------------------------------------------------------------
void Camera::getBeamCenterX(double& value) ///< [out] 
{
  DEB_MEMBER_FUNCT();
  getParam(Requests::HEADER_BEAM_CENTER_X,value);
}

//-----------------------------------------------------------------------------
///  BeamCenterY setter
//-----------------------------------------------------------------------------
void Camera::setBeamCenterY(double value) ///< [in] 
{
    DEB_MEMBER_FUNCT();
    setParam(Requests::HEADER_BEAM_CENTER_Y,value);
}

//-----------------------------------------------------------------------------
///  BeamCenterY getter
//-----------------------------------------------------------------------------
void Camera::getBeamCenterY(double& value) ///< [out] 
{
  DEB_MEMBER_FUNCT();
  getParam(Requests::HEADER_BEAM_CENTER_Y,value);
}

//-----------------------------------------------------------------------------
///  DetectorDistance setter
//-----------------------------------------------------------------------------
void Camera::setDetectorDistance(double value) ///< [in] 
{
    DEB_MEMBER_FUNCT();
    setParam(Requests::HEADER_DETECTOR_DISTANCE,value);
}

//-----------------------------------------------------------------------------
///  DetectorDistance getter
//-----------------------------------------------------------------------------
void Camera::getDetectorDistance(double& value) ///< [out] 
{
  DEB_MEMBER_FUNCT();
  getParam(Requests::HEADER_DETECTOR_DISTANCE,value);
}

//-----------------------------------------------------------------------------
///  DataCollectionDate getter
//-----------------------------------------------------------------------------
void Camera::getDataCollectionDate(std::string& value) ///< [out] 
{
  DEB_MEMBER_FUNCT();
  getParam(Requests::DATA_COLLECTION_DATE,value);
}

//-----------------------------------------------------------------------------
///  SoftwareVersion getter
//-----------------------------------------------------------------------------
void Camera::getSoftwareVersion(std::string& value) ///< [out] 
{
  DEB_MEMBER_FUNCT();
  getParam(Requests::SOFTWARE_VERSION,value);
}
            
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void Camera::getCompression(bool& value) ///< [out] true:enabled, false:disabled
{
  DEB_MEMBER_FUNCT();
  getParam(Requests::FILEWRITER_COMPRESSION,value);
}


//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
void Camera::setCompression(bool value)
{
  DEB_MEMBER_FUNCT();
  setParam(Requests::FILEWRITER_COMPRESSION,value);
}

void Camera::getCompressionType(Camera::CompressionType& type)
{
  DEB_MEMBER_FUNCT();
  AutoMutex lock(m_cond.mutex());
  type = m_compression_type;
  DEB_RETURN() << DEB_VAR1(type);
}

void Camera::setCompressionType(Camera::CompressionType type)
{
  DEB_MEMBER_FUNCT();
  DEB_PARAM() << DEB_VAR1(type);

  const char *s;
  switch (type) {
  case NoCompression: s = "none"; break;
  case LZ4: s = "lz4"; break;
  case BSLZ4: s = "bslz4"; break;
  default:
    THROW_HW_ERROR(InvalidValue) << "Invalid compression type: " << type;
  }
  DEB_TRACE() << DEB_VAR1(s);

  std::string type_str;
  ParamReq type_req = getParam(Requests::COMPRESSION_TYPE, type_str);
  Requests::Param::Value types_allowed;
  try {
    types_allowed = type_req->get_allowed_values();
  } catch (const EigerException &e) {
    HANDLE_EIGERERROR(type_req, e);
  }
  const vector<string>& l = types_allowed.string_array;
  if (DEB_CHECK_ANY(DebTypeTrace)) {
    DEB_TRACE() << "allowed compression types:";
    vector<string>::const_iterator it, end = l.end();
    for (it = l.begin(); it != end; ++it)
      DEB_TRACE() << "  " << *it;
  }
  if (find(l.begin(), l.end(), s) == l.end())
    THROW_HW_ERROR(NotSupported) << "Compression type " << type
				 << " not allowed";
    
  setParam(Requests::COMPRESSION_TYPE, s);
  AutoMutex lock(m_cond.mutex());
  m_compression_type = type;
}

void Camera::getSerieId(int& serie_id)
{
  DEB_MEMBER_FUNCT();
  serie_id = m_serie_id;
  DEB_RETURN() << DEB_VAR1(serie_id);
}

void Camera::deleteMemoryFiles()
{
  DEB_MEMBER_FUNCT();
  sendCommand(Requests::FILEWRITER_CLEAR);
}

void Camera::disarm()
{
  DEB_MEMBER_FUNCT();
  AutoMutex lock(m_cond.mutex());
  if (!m_armed)
    return;

  m_armed = false;
  lock.unlock();
  DEB_TRACE() << "Disarming";
  sendCommand(Requests::DISARM);
}

const std::string& Camera::getDetectorHost() const
{
  return m_detector_host;
}

int Camera::getDetectorStreamPort() const
{
  return m_detector_stream_port;
}

std::ostream &lima::Eiger::operator <<(std::ostream& os,
				       Camera::CompressionType comp_type)
{
  std::string type_str;
  switch (comp_type) {
  case Camera::NoCompression:
    type_str = "NoCompression";
    break;
  case Camera::LZ4:
    type_str = "LZ4";
    break;
  case Camera::BSLZ4:
    type_str = "BSLZ4";
    break;
  default:
    type_str = "Unknown";
  }
  os << type_str;
  return os;
}

std::istream &lima::Eiger::operator >>(std::istream& is,
				       Camera::CompressionType& comp_type)
{
  std::string s;
  is >> s;
  std::transform(s.begin(), s.end(), s.begin(), ::toupper);
  if (s == "NOCOMPRESSION")
    comp_type = Camera::NoCompression;
  else if (s == "LZ4")
    comp_type = Camera::LZ4;
  else if (s == "BSLZ4")
    comp_type = Camera::BSLZ4;
  else
    throw LIMA_HW_EXC(InvalidValue, "Invalid Camera::CompressionType: ") << s;
  return is;
}
