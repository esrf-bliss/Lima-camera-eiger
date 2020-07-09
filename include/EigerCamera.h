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
#ifndef EIGERCAMERA_H
#define EIGERCAMERA_H

#include "EigerCompatibility.h"
#if defined (__GNUC__) && (__GNUC__ == 3) && defined (__ELF__)
#   define GENAPI_DECL __attribute__((visibility("default")))
#   define GENAPI_DECL_ABSTRACT __attribute__((visibility("default")))
#endif


#include <stdlib.h>
#include <limits>
#include "lima/HwMaxImageSizeCallback.h"
#include "lima/ThreadUtils.h"
#include "lima/Event.h"

#include <eigerapi/EigerDefines.h>

#include <ostream>

DEB_GLOBAL_NAMESPC(DebModCamera, "Eiger");

namespace eigerapi
{
  class Requests;
}

namespace lima
{
namespace Eiger
{

struct SuccessAck
{
  void succeeded() {}
};

class SavingCtrlObj;
class Stream;
class MultiParamRequest;

/*******************************************************************
 * \class Camera
 * \brief object controlling the Eiger camera via EigerAPI
 *******************************************************************/
class LIBEIGER Camera : public HwMaxImageSizeCallbackGen, public EventCallbackGen
{
  DEB_CLASS_NAMESPC(DebModCamera, "Camera", "Eiger");

 public:
  enum ApiGeneration { Eiger1, Eiger2 };
  enum Status { Initializing, Ready, Armed, Exposure, Fault };
  enum CompressionType {NoCompression,LZ4,BSLZ4};

  Camera(const std::string& host, int http_port=80, int stream_port=9999);
  ~Camera();

  void initialize();

  void startAcq();
  void stopAcq();
  void prepareAcq();

  // -- detector info object
  void getImageType(ImageType& type);
  void setImageType(ImageType type);

  void getDetectorType(std::string& type);
  void getDetectorModel(std::string& model);
  void getDetectorImageSize(Size& size);
  void getDetectorMaxImageSize(Size& size);

  //-- Synch control object
  bool checkTrigMode(TrigMode trig_mode);
  void setTrigMode(TrigMode  mode);
  void getTrigMode(TrigMode& mode);

  void setExpTime(double  exp_time, bool force = false);
  void getExpTime(double& exp_time);

  void setLatTime(double  lat_time);
  void getLatTime(double& lat_time);

  void getExposureTimeRange(double& min_expo, double& max_expo);
  void getLatTimeRange(double& min_lat, double& max_lat);

  void setNbFrames(int  nb_frames);
  void getNbFrames(int& nb_frames);
  void getNbHwAcquiredFrames(int &nb_acq_frames);

  bool isBinningAvailable();

  void getPixelSize(double& sizex, double& sizey);

  Camera::Status getStatus();
  std::string getCamStatus();
//void reset();

  // -- Eiger specific
  void getApiGeneration(ApiGeneration&);

  void getTemperature(double&);
  void getHumidity(double&);
         
  void setCountrateCorrection(bool);
  void getCountrateCorrection(bool&);
  void setFlatfieldCorrection(bool);
  void getFlatfieldCorrection(bool&);
  void setAutoSummation(bool);
  void getAutoSummation(bool&);
  void setEfficiencyCorrection(bool);
  void getEfficiencyCorrection(bool& value);
  void setPixelMask(bool);
  void getPixelMask(bool&);
  void setThresholdEnergy(double);
  void getThresholdEnergy(double&);
  void setVirtualPixelCorrection(bool);
  void getVirtualPixelCorrection(bool&);
  void setPhotonEnergy(double);
  void getPhotonEnergy(double&);
  void setWavelength(double);
  void getWavelength(double&);
  void setBeamCenterX(double);
  void getBeamCenterX(double&);
  void setBeamCenterY(double);
  void getBeamCenterY(double&);
  void setDetectorDistance(double);
  void getDetectorDistance(double&);
  void getDataCollectionDate(std::string&);

  void getSoftwareVersion(std::string&);

  void getCompression(bool&);
  void setCompression(bool);
  void getCompressionType(CompressionType&);
  void setCompressionType(CompressionType);
  void getSerieId(int&);
  void deleteMemoryFiles();
  void disarm();

  const std::string& getDetectorHost() const;
  int getDetectorStreamPort() const;

 private:
  friend class Interface;
  friend class SavingCtrlObj;
  friend class Stream;
  friend class MultiParamRequest;
  friend class CameraRequest;

  enum InternalStatus {IDLE,RUNNING,ERROR};
  class TriggerCallback;
  friend class TriggerCallback;
  class InitCallback;
  friend class InitCallback;

  Camera::Status _getStatus();

  void _synchronize(); /// Used during plug-in initialization
  void _trigger_finished(bool ok, bool do_disarm);
  void _initialization_finished(bool ok);

  void _updateImageSize();

  void getNbTriggeredFrames(int& nb_trig_frames);
  void newFrameAcquired();
  bool allFramesAcquired();

  template <typename T>
  struct Cache
  {
    struct ChangeInfo : public SuccessAck
    {
      ChangeInfo(Cache& c, T v) : cache(c), new_val(v)
      {}
      operator bool()
      { return new_val != cache.val; }
      void succeeded()
      { cache.val = new_val; }
      Cache& cache;
      T new_val;
    };

    T val;

    Cache() = default;
    Cache(T v) : val(v) {}

    operator T&() { return val; }
    T& value() { return val; }

    Cache& operator =(T new_val)
    { val = new_val; return *this; }

    ChangeInfo change(T new_val)
    { return {*this, new_val}; }
  };


  //-----------------------------------------------------------------------------
  //- lima stuff
  int                       m_nb_frames;
  Cache<unsigned int>       m_nb_images;
  Cache<unsigned int>       m_nb_triggers;
  int                       m_frames_triggered;
  int                       m_frames_acquired;
  double                    m_latency_time;
  TrigMode                  m_trig_mode;
  Cache<std::string>        m_trig_mode_name;

  //- camera stuff
  ApiGeneration             m_api;
  std::string               m_detector_model;
  std::string               m_detector_type;
  unsigned int              m_maxImageWidth, m_maxImageHeight;
  ImageType                 m_detectorImageType;

  InternalStatus            m_initialize_state;
  InternalStatus            m_trigger_state;
  bool                      m_armed;
  int                       m_serie_id;
  //- EigerAPI stuff
  eigerapi::Requests*       m_requests;

  double                    m_temperature;
  double                    m_humidity;
  Cache<double>             m_exp_time;
  Cache<double>             m_frame_time;
  double                    m_readout_time;
  double                    m_x_pixelsize, m_y_pixelsize;
  Cond                      m_cond;
  std::string               m_detector_host;
  int                       m_detector_http_port;
  int                       m_detector_stream_port;
  double                    m_min_frame_time;
  CompressionType           m_compression_type;
};

} // namespace Eiger
} // namespace lima


#endif // EIGERCAMERA_H
