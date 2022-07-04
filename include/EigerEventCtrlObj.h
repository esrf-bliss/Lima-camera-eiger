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
#ifndef EIGEREVENTCTRLOBJ_H
#define EIGEREVENTCTRLOBJ_H

#include "lima/HwEventCtrlObj.h"
#include "EigerCamera.h"

namespace lima
{
    namespace Eiger
    {

/*******************************************************************
 * \class EventCtrlObj
 * \brief Control object providing Eiger event interface
 *******************************************************************/

	class /*LIBEIGER*/ EventCtrlObj : public HwEventCtrlObj
	{
	    DEB_CLASS_NAMESPC(DebModCamera, "EventCtrlObj", "Eiger");

	public:
	    EventCtrlObj(Camera& cam);
	    virtual ~EventCtrlObj();

	private:
	    class EventCallback : public lima::EventCallback {
	    public:
		EventCallback(EventCtrlObj& ctrl_obj)
		  : m_ctrl_obj(ctrl_obj) {}
		virtual void processEvent(Event *event)
		{ m_ctrl_obj.reportEvent(event); }
	    private:
		EventCtrlObj& m_ctrl_obj;
	    };
	    Camera& m_cam;
	    std::unique_ptr<EventCallback> m_cbk;
	};

    } // namespace Eiger
} // namespace lima

#endif // EIGEREVENTCTRLOBJ
