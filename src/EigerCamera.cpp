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

#define setParam(param, value)			\
  setEigerParam(*this, param, value)

#define getParam(param, value)			\
  getEigerParam(*this, param, value)


/*----------------------------------------------------------------------------
			    Callback class
 ----------------------------------------------------------------------------*/
class Camera::TriggerCallback : public Callback
{
  DEB_CLASS_NAMESPC(DebModCamera, "Camera::TriggerCallback", "Eiger");
public:
  TriggerCallback(Camera& cam) : m_cam(cam) {}

  void status_changed(CurlLoop::FutureRequest::Status status,
		      std::string error)
  {
    DEB_MEMBER_FUNCT();
    DEB_PARAM() << DEB_VAR2(status, error);
    bool ok = (status == CurlLoop::FutureRequest::OK);
    if (!ok)
      DEB_ERROR() << DEB_VAR1(error); 
    m_cam._trigger_finished(ok);
  }
private:
  Camera& m_cam;
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
Camera::Camera(const std::string& detector_ip, 	///< [in] Ip address of the detector server
	       ApiGeneration api)
  : 		m_api(api),
		m_image_number(0),
                m_latency_time(0.),
                m_detectorImageType(Bpp16),
		m_initialize_state(IDLE),
		m_trigger_state(IDLE),
		m_armed(false),
		m_serie_id(0),
                m_requests(new Requests(detector_ip)),
                m_exp_time(1.),
		m_detector_ip(detector_ip)
{
    DEB_CONSTRUCTOR();
    DEB_PARAM() << DEB_VAR1(detector_ip);
    // Init EigerAPI
    try
      {
	_synchronize();
      }
    catch(Exception& e)
      {
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
    } catch (const lima::Exception& e) {
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

  MultiParamRequest synchro(*this);
  if (m_frame_time.changed(frame_time))
    synchro.addSet(Requests::FRAME_TIME, frame_time);
  if (m_nb_images.changed(nb_images))
    synchro.addSet(Requests::NIMAGES, nb_images);
  if (m_nb_triggers.changed(nb_triggers))
    synchro.addSet(Requests::NTRIGGER, nb_triggers);
  synchro.wait();

  DEB_TRACE() << "Arm start";
  double timeout = 5 * 60.; // 5 min timeout
  CommandReq arm_cmd = m_requests->get_command(Requests::ARM);
  try
    {
      arm_cmd->wait(timeout);
      DEB_TRACE() << "Arm end";
      m_serie_id = arm_cmd->get_serie_id();
      m_armed = true;
    }
  catch(const eigerapi::EigerException &e)
    {
      m_requests->cancel(arm_cmd);
      HANDLE_EIGERERROR(arm_cmd, e);
    }
  m_image_number = 0;
}


//-----------------------------------------------------------------------------
///  start the acquisition
//-----------------------------------------------------------------------------
void Camera::startAcq()
{
  DEB_MEMBER_FUNCT();
  AutoMutex lock(m_cond.mutex());

  if(m_trig_mode == IntTrig ||
     m_trig_mode == IntTrigMult)
    {
      bool disarm_at_end = ((m_trig_mode == IntTrig) ||
			    (m_image_number == m_nb_frames - 1));
      DEB_TRACE() << "Trigger start: " << DEB_VAR1(disarm_at_end);
      CommandReq trigger = m_requests->get_command(Requests::TRIGGER);
      m_trigger_state = RUNNING;
      lock.unlock();

      CallbackPtr cbk(new TriggerCallback(*this));
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

  ParamReq width_request = 
    m_requests->get_param(Requests::DETECTOR_WITDH);
  ParamReq height_request = 
    m_requests->get_param(Requests::DETECTOR_HEIGHT);

  Requests::Param::Value width = width_request->get();
  Requests::Param::Value height = height_request->get();
  size = Size(width.data.int_val, height.data.int_val);
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

  if (m_trig_mode.changed(trig_mode))
    setParam(Requests::TRIGGER_MODE,trig_name);
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

  if (m_exp_time.changed(exp_time) || force)
    setParam(Requests::EXPOSURE, exp_time);
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
const
{
  DEB_MEMBER_FUNCT();
  ParamReq exp_time = 
    m_requests->get_param(Requests::EXPOSURE);
  try
    {
      exp_time->wait();
    }
  catch(const eigerapi::EigerException &e)
    {
      m_requests->cancel(exp_time);
      HANDLE_EIGERERROR(exp_time, e);
    }

  Requests::Param::Value min_val = exp_time->get_min();
  Requests::Param::Value max_val;
  if (m_api == Eiger1)
    max_val = exp_time->get_max();
  else
    max_val.data.double_val = 1800;

  min_expo = min_val.data.double_val;
  max_expo = max_val.data.double_val;
  DEB_RETURN() << DEB_VAR2(min_expo, max_expo);
}


//-----------------------------------------------------------------------------
///  Get the latency time range
//-----------------------------------------------------------------------------
void Camera::getLatTimeRange(double& min_lat, ///< [out] minimum latency
                             double& max_lat) ///< [out] maximum latency
const
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
  nb_acq_frames = m_image_number;
}


//-----------------------------------------------------------------------------
/// Get the camera status
//-----------------------------------------------------------------------------
Camera::Status Camera::getStatus() ///< [out] current camera status
{
  DEB_MEMBER_FUNCT();

  Camera::Status status;
  AutoMutex lock(m_cond.mutex());
  if(m_initialize_state == ERROR ||
     m_trigger_state == ERROR)
    status = Fault;
  else if(m_trigger_state == RUNNING)
    status = Exposure;
  else if(m_armed)
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

  std::string trig_name;
  synchro.addGet(Requests::TRIGGER_MODE, trig_name);
  
  synchro.addGet(Requests::X_PIXEL_SIZE, m_x_pixelsize);
  synchro.addGet(Requests::Y_PIXEL_SIZE, m_y_pixelsize);

  synchro.addGet(Requests::DETECTOR_WITDH, m_maxImageWidth);
  synchro.addGet(Requests::DETECTOR_HEIGHT, m_maxImageHeight);

  synchro.addGet(Requests::DETECTOR_READOUT_TIME, m_readout_time);

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
  if(trig_name == "ints")
    m_trig_mode = m_nb_triggers > 1 ? IntTrigMult : IntTrig;
  else if(trig_name == "exts")
    m_trig_mode = m_nb_triggers > 1 ? ExtTrigMult : ExtTrigSingle;
  else if(trig_name == "exte")
    m_trig_mode = ExtGate;
  else
    THROW_HW_ERROR(InvalidValue) << "Unexpected trigger mode: "
				 << DEB_VAR1(trig_name);
  
  Requests::Param::Value min_frame_time = synchro[Requests::FRAME_TIME]->get_min();
  m_min_frame_time = min_frame_time.data.double_val;

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
void Camera::_trigger_finished(bool ok)
{
  DEB_MEMBER_FUNCT();
  DEB_PARAM() << DEB_VAR1(ok);
  
  DEB_TRACE() << "Trigger end";
  if(!ok)
    DEB_ERROR() << "Error in trigger command";
  else if(allFramesAcquired())
    try { disarm(); }
    catch (...) { ok = false; }

  AutoMutex lock(m_cond.mutex());
  m_trigger_state = ok ? IDLE : ERROR;
}

void Camera::newFrameAcquired()
{
  DEB_MEMBER_FUNCT();
  AutoMutex lock(m_cond.mutex());
  m_image_number++;
  DEB_TRACE() << DEB_VAR1(m_image_number);
}

bool Camera::allFramesAcquired()
{
  DEB_MEMBER_FUNCT();
  AutoMutex lock(m_cond.mutex());
  DEB_PARAM() << DEB_VAR2(m_image_number, m_nb_frames);
  bool finished = (m_image_number == m_nb_frames);
  DEB_RETURN() << DEB_VAR1(finished);
  return finished;
}

//-----------------------------------------------------------------------------
/// Returns the API generation of the detector
/*!
@return temperature value
*/
//-----------------------------------------------------------------------------
void Camera::getApiGeneration(ApiGeneration& api)
{
  DEB_MEMBER_FUNCT();
  api = m_api;
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
void Camera::setThresholdEnergy(double value) ///< [in] true:enabled, false:disabled
{
  DEB_MEMBER_FUNCT();
  setParam(Requests::THRESHOLD_ENERGY,value);
}


//-----------------------------------------------------------------------------
///  ThresholdEnergy getter
//-----------------------------------------------------------------------------
void Camera::getThresholdEnergy(double& value) ///< [out] true:enabled, false:disabled
{
  DEB_MEMBER_FUNCT();
  getParam(Requests::THRESHOLD_ENERGY,value);
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

  ParamReq type_req =
    m_requests->get_param(Requests::COMPRESSION_TYPE);
  Requests::Param::Value types_allowed = type_req->get_allowed_values();
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
  _disarm();
}

void Camera::_disarm()
{
  DEB_MEMBER_FUNCT();
  DEB_PARAM() << DEB_VAR1(m_armed);
  if (m_armed) {
    DEB_TRACE() << "Disarming";
    sendCommand(Requests::DISARM);
    m_armed = false;
  }
}

const std::string& Camera::getDetectorIp() const
{
  return m_detector_ip;
}
