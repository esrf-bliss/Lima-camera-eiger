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


%MappedType std::list<Eiger::RoiCtrlObj::PATTERN2ROI>
{
%TypeHeaderCode
#include <EigerRoiCtrlObj.h>
#include <list>
using namespace lima;
%End

%ConvertToTypeCode
if(sipIsErr == NULL)
  {
    bool aReturnFlag = PyList_Check(sipPy);
    for(int i = 0;aReturnFlag && i < PyList_Size(sipPy);++i)
      {
        PyObject *p = PyList_GET_ITEM(sipPy,i);
        aReturnFlag = PySequence_Check(p) &&
                      PySequence_Size(p) == 2 &&
                      sipCheckConvertFromPyStr(PySequence_Fast_GET_ITEM(p,0)) &&
                      sipCanConvertToType(PySequence_Fast_GET_ITEM(p,1),
                                          sipType_Roi,SIP_NOT_NONE);
      }
    return aReturnFlag;
  }
  std::list<Eiger::RoiCtrlObj::PATTERN2ROI> *roisPt = new std::list<Eiger::RoiCtrlObj::PATTERN2ROI>();

  for(int i = 0;i < PyList_Size(sipPy);++i)
    {
      PyObject *p = PyList_GET_ITEM(sipPy,i);
      PyObject *aTmpName = PySequence_Fast_GET_ITEM(p,0);
      std::string name = sipConvertFromPyStr(aTmpName);
      int state;
      lima::Roi *roi =
        reinterpret_cast<lima::Roi*>(sipConvertToType(PySequence_Fast_GET_ITEM(p,1),
                                                      sipType_Roi,0,
                                                      SIP_NOT_NONE,
                                                      &state,sipIsErr));
      if(*sipIsErr)
        {
          sipReleaseType(roi,sipType_Roi,state);
          delete roisPt;
          return 0;
        }
      roisPt->push_back(Eiger::RoiCtrlObj::PATTERN2ROI(name,*roi));
      sipReleaseType(roi,sipType_Roi,state);
    }
  *sipCppPtr = roisPt;
  return sipGetState(sipTransferObj);
%End

%ConvertFromTypeCode
   PyObject *l;
  if(!(l = PyList_New(sipCpp->size())))
    return NULL;

  sipTransferObj = NULL;
  int i = 0;
  for(Eiger::RoiCtrlObj::ROIS::iterator j = sipCpp->begin();
      j != sipCpp->end();++j,++i)
    {
      PyObject *wobj;
      lima::Roi *roi = new lima::Roi(j->second);
      if(!(wobj = sipConvertFromNewType(roi,sipType_Roi,sipTransferObj)))
        {
          delete roi;
          Py_DECREF(l);
          return NULL;
        }
      PyObject *aNamePy = sipConvertToPyStr(j->first);
      PyList_SET_ITEM(l,i,PyTuple_Pack(2,aNamePy,wobj));
      Py_DECREF(aNamePy);Py_DECREF(wobj);
    }
  return l;
%End
};

namespace Eiger
{
/*******************************************************************
 * \class RoiCtrlObj
 * \brief Eiger hardware Roi
 *******************************************************************/
 class RoiCtrlObj : HwRoiCtrlObj
 {
%TypeHeaderCode
#include <EigerRoiCtrlObj.h>
#include <list>
using namespace lima;
%End

  public:
  	typedef std::pair<std::string,lima::Roi> PATTERN2ROI;
  	typedef std::list<PATTERN2ROI> ROIS;

    RoiCtrlObj(Eiger::Camera& cam);
	  virtual ~RoiCtrlObj();

	  virtual void setRoi(const Roi& set_roi);
	  virtual void getRoi(Roi& hw_roi);
	  virtual void checkRoi(const Roi& set_roi, Roi& hw_roi);
	  void getModelSize(std::string& model /Out/) const; 
	  void getSupportedHwRois(std::list<Eiger::RoiCtrlObj::PATTERN2ROI>& rois /Out/) const;
	  bool hasHwRoiSupport();

private:
    RoiCtrlObj(const Eiger::RoiCtrlObj&);
};
};

