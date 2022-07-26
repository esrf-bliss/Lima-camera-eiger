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
#ifndef EIGERROICTRLOBJ_H
#define EIGERROICTRLOBJ_H

#include <eiger_export.h>

#include "lima/HwRoiCtrlObj.h"
#include "EigerCamera.h"
#include "EigerDetInfoCtrlObj.h"

namespace lima
{
namespace Eiger
{


/*******************************************************************
 * \class RoiCtrlObj
 * \brief Control object providing Eiger Roi interface
 *******************************************************************/

class EIGER_EXPORT RoiCtrlObj : public HwRoiCtrlObj
{
DEB_CLASS_NAMESPC(DebModCamera, "RoiCtrlObj", "Eiger");

public:
  	typedef std::pair<std::string,lima::Roi> PATTERN2ROI;
  	typedef std::list<PATTERN2ROI> ROIS;

    RoiCtrlObj(Camera& cam);
	virtual ~RoiCtrlObj();

	virtual void setRoi(const Roi& set_roi);
	virtual void getRoi(Roi& hw_roi);
	virtual void checkRoi(const Roi& set_roi, Roi& hw_roi);
	void getModelSize(std::string& model) const;
	void getSupportedHwRois(std::list<PATTERN2ROI> & rois) const;
	bool hasHwRoiSupport();

private:

	inline ROIS::const_iterator _getRoi(const Roi& roi) const;
	inline ROIS::const_iterator _getRoi(const std::string& roi_pattern) const;

	Camera& m_cam;
	ROIS m_supported_rois;
	std::string m_model_size;
	std::string m_model_sensor;
};


} // namespace Eiger
} // namespace lima

#endif // EIGERROICTRLOBJ_H

