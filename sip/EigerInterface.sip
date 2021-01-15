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
namespace Eiger
{

  class DetInfoCtrlObj : HwDetInfoCtrlObj
  {
%TypeHeaderCode
#include "EigerDetInfoCtrlObj.h"
%End

  public:
    DetInfoCtrlObj(Eiger::Camera& simu);
    virtual ~DetInfoCtrlObj();

    virtual void getMaxImageSize(Size& max_image_size /Out/);
    virtual void getDetectorImageSize(Size& det_image_size /Out/);

    virtual void getDefImageType(ImageType& def_image_type /Out/);
    virtual void getCurrImageType(ImageType& curr_image_type /Out/);
    virtual void setCurrImageType(ImageType  curr_image_type);

    virtual void getPixelSize(double& x_size /Out/,double &y_size /Out/);
    virtual void getDetectorType(std::string& det_type /Out/);
    virtual void getDetectorModel(std::string& det_model /Out/);

    virtual void registerMaxImageSizeCallback(HwMaxImageSizeCallback& cb);
    virtual void unregisterMaxImageSizeCallback(HwMaxImageSizeCallback& cb);
  };

  /*******************************************************************
   * \class Interface
   * \brief Eiger hardware interface
   *******************************************************************/
  class Interface : HwInterface
  {
%TypeHeaderCode
#include <EigerInterface.h>
%End

  public:
    Interface(Eiger::Camera& cam /KeepReference/);
    virtual ~Interface();

    //- From HwInterface
    //virtual void    getCapList(CapList&) const;
    virtual void    getCapList(std::vector<HwCap> &cap_list /Out/) const;
    virtual void    reset(ResetLevel reset_level);
    virtual void    prepareAcq();
    virtual void    startAcq();
    virtual void    stopAcq();
    virtual void    getStatus(StatusType& status /Out/);
    virtual int     getNbHwAcquiredFrames();

    //! get the camera object to access it directly from client
    Eiger::Camera& getCamera();

    void getLastStreamInfo(Eiger::StreamInfo& last_info /Out/);
    void latchStreamStatistics(Eiger::StreamStatistics& stat /Out/,
                               bool reset=false);
  };
};
