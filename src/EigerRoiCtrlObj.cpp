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
#include "EigerRoiCtrlObj.h"
#include <iterator>

using namespace lima;
using namespace lima::Eiger;
using namespace std;

//-----------------------------------------------------
// @brief Ctor
//-----------------------------------------------------
RoiCtrlObj::RoiCtrlObj(Camera& cam)
    : m_cam(cam)

{
    DEB_CONSTRUCTOR();    

    string model;
    cam.getDetectorModel(model);
    stringstream ss(model);
    istream_iterator<string> begin(ss);
    istream_iterator<string> end;
    // split long model info into token, supposing it will always
    // space-separated words:
    //  - "Dectris EIGER2 Si 4M"
    //  - "Dectris EIGER2 CdTe 4M"
    vector<string> model_list(begin, end);
    m_model_size = model_list[3];
    m_model_sensor = model_list[2];
    
    Size det_max_size;
    m_cam.getDetectorMaxImageSize(det_max_size);

    if (hasHwRoiSupport())
    {
        if (m_model_size == "9M")
        {
            Roi r4M_left(Point(0,550), Point(2067,2711));
            m_supported_rois.push_back(PATTERN2ROI("4M-L", r4M_left));
            Roi r4M_right(Point(1040,550), Point(3107,2711));
            m_supported_rois.push_back(PATTERN2ROI("4M-R", r4M_right));            
        }
        if (m_model_size == "16M")
        {
            Point p1(1040,1100), p2(3107,3261);
            Roi r4M(p1,p2);
            m_supported_rois.push_back(PATTERN2ROI("4M", r4M));
        }
    }
    Roi full(Point(0,0), det_max_size);
    m_supported_rois.push_back(PATTERN2ROI("disabled", full));
}

//-----------------------------------------------------
// @brief Dtor
//-----------------------------------------------------
RoiCtrlObj::~RoiCtrlObj()
{
    DEB_DESTRUCTOR();
}


//-----------------------------------------------------
// @brief
//-----------------------------------------------------
bool RoiCtrlObj::hasHwRoiSupport()
{
    DEB_MEMBER_FUNCT();
    if (m_model_size == "9M" || m_model_size == "16M")
        return true;
    else
        return false;        
}

//-----------------------------------------------------
// @brief
//-----------------------------------------------------
void RoiCtrlObj::checkRoi(const Roi& set_roi, Roi& hw_roi)
{
    DEB_MEMBER_FUNCT();

    ROIS::const_iterator i = _getRoi(set_roi);
    if(i == m_supported_rois.end())
        THROW_HW_ERROR(Error) << "Something weird happened";
    hw_roi = i->second;
}

//-----------------------------------------------------
// @brief
//-----------------------------------------------------
void RoiCtrlObj::setRoi(const Roi& set_roi)
{
    DEB_MEMBER_FUNCT();
    ROIS::const_iterator i;
    if(set_roi.isActive())
    {
      i = _getRoi(set_roi);
      if(i == m_supported_rois.end())
	    THROW_HW_ERROR(Error) << "Something weird happened";
    }
    else
        i = --m_supported_rois.end(); // full_frame

     m_cam.setHwRoiPattern(i->first);
}

//-----------------------------------------------------
// @brief
//-----------------------------------------------------
void RoiCtrlObj::getRoi(Roi& roi)
{
    DEB_MEMBER_FUNCT();
    string roi_pattern;
    m_cam.getHwRoiPattern(roi_pattern);

    ROIS::const_iterator i;
    i = _getRoi(roi_pattern);
    if(i == m_supported_rois.end())
        THROW_HW_ERROR(Error) << "Something weird happened";

    roi = i->second;
}

inline RoiCtrlObj::ROIS::const_iterator
RoiCtrlObj::_getRoi(const Roi& roi) const
{
  for(ROIS::const_iterator i = m_supported_rois.begin();
      i != m_supported_rois.end();++i)
    {
      if(i->second.containsRoi(roi))
	return i;
    }
  return m_supported_rois.end();
}

inline RoiCtrlObj::ROIS::const_iterator
RoiCtrlObj::_getRoi(const string& roi_pattern) const
{
  for(ROIS::const_iterator i = m_supported_rois.begin();
      i != m_supported_rois.end();++i)
    {
      if(i->first == roi_pattern)
	return i;
    }
  return m_supported_rois.end();
}

//-----------------------------------------------------
// @brief
//-----------------------------------------------------
void RoiCtrlObj::getModelSize(std::string& model) const
{
    DEB_MEMBER_FUNCT();
    model = m_model_size;
}

//-----------------------------------------------------
// @brief
//-----------------------------------------------------
void RoiCtrlObj::getSupportedHwRois(std::list<PATTERN2ROI>& hwrois) const
{
    DEB_MEMBER_FUNCT();
    hwrois = m_supported_rois;
}
