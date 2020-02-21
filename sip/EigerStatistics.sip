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
  /*******************************************************************
   * \struct StreamStatistics
   * \brief Eiger hardware interface
   *******************************************************************/
  struct StreamStatistics
  {
%TypeHeaderCode
#include <EigerStatistics.h>
%End

    int n() const;
    double ave_size() const;
    double ave_time() const;
    double ave_speed() const;
  };
};
