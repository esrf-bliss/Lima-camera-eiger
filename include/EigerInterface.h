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
#ifndef EIGERINTERFACE_H
#define EIGERINTERFACE_H

#include "EigerCompatibility.h"
#include "lima/HwInterface.h"
#include "EigerRoiCtrlObj.h"

namespace lima
{
    namespace Eiger
    {

      class DetInfoCtrlObj;
      class SyncCtrlObj;
      class SavingCtrlObj;
      class EventCtrlObj;
      class Camera;
      class Stream;
      class StreamInfo;
      class StreamStatistics;
      class Decompress;

	/*******************************************************************
	* \class Interface
	* \brief Eiger hardware interface
	*******************************************************************/
	class LIBEIGER Interface : public HwInterface
	{
	DEB_CLASS_NAMESPC(DebModCamera, "EigerInterface", "Eiger");

	public:
	    Interface(Camera& cam, const char* mmap_file=NULL);
	    virtual ~Interface();

	    //- From HwInterface
	    virtual void    getCapList(CapList&) const;
	    virtual void    reset(ResetLevel reset_level);
	    virtual void    prepareAcq();
	    virtual void    startAcq();
	    virtual void    stopAcq();
	    virtual void    getStatus(StatusType& status);
	    virtual int     getNbHwAcquiredFrames();
    
		//! get the camera object to access it directly from client
		Camera& getCamera() { return m_cam;}

	    void getLastStreamInfo(StreamInfo& info);
	    void latchStreamStatistics(StreamStatistics& stat,
				       bool reset=false);
		bool hasHwRoiSupport();
		void getSupportedHwRois(std::list<Eiger::RoiCtrlObj::PATTERN2ROI>& hwrois) const;
		void getModelSize(std::string& model) const;

	private:
	    Camera&         m_cam;
	    CapList         m_cap_list;
	    DetInfoCtrlObj* m_det_info;
		RoiCtrlObj*     m_roi;
	    SyncCtrlObj*    m_sync;
	    SavingCtrlObj*  m_saving;
	    EventCtrlObj*   m_event;
	    Stream*	        m_stream;
	    Decompress*	    m_decompress;
	};

    } // namespace Eiger
} // namespace lima

#endif // EigerINTERFACE_H
