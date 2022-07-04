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

    if (hasHwRoiSupport())
    {
        Size det_max_size;
        m_cam.getDetectorMaxImageSize(det_max_size);
        int fullframe_max_frequency = -1;

        if (m_model_size == "9M")
        {
            fullframe_max_frequency = 230;
            Roi r4M_left(Point(0,550), Point(2067,2711));
            m_possible_rois.push_back(PATTERN2ROI(Pattern("4M-L", 500), r4M_left));
            Roi r4M_right(Point(1040,550), Point(3107,2711));
            m_possible_rois.push_back(PATTERN2ROI(Pattern("4M-R", 500), r4M_right));            
        }
        if (m_model_size == "16M")
        {
            fullframe_max_frequency = 130;
            Point p1(1040,1100), p2(3107,3261);
            Roi r4M(p1,p2);
            m_possible_rois.push_back(PATTERN2ROI(Pattern("4M", 500), r4M));
        }
        Roi full(Point(0,0), det_max_size);
        m_possible_rois.push_back(PATTERN2ROI(Pattern("disabled", fullframe_max_frequency), full));
        m_current_roi = full;
        m_current_max_frequency = fullframe_max_frequency;
    }
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
    if(i == m_possible_rois.end())
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
      if(i == m_possible_rois.end())
	    THROW_HW_ERROR(Error) << "Something weird happened";
    }
    else
        i = --m_possible_rois.end(); // full_frame

     m_cam.setHwRoiPattern(i->first.pattern);
  
    m_current_roi = i->second;
    m_current_max_frequency = i->first.max_frequency;

}

//-----------------------------------------------------
// @brief
//-----------------------------------------------------
void RoiCtrlObj::getRoi(Roi& roi)
{
    DEB_MEMBER_FUNCT();
    roi = m_current_roi;
}

inline RoiCtrlObj::ROIS::const_iterator
RoiCtrlObj::_getRoi(const Roi& roi) const
{
  for(ROIS::const_iterator i = m_possible_rois.begin();
      i != m_possible_rois.end();++i)
    {
      if(i->second.containsRoi(roi))
	return i;
    }
  return m_possible_rois.end();
}