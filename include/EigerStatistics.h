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
#ifndef EIGERSTATISTICS_H
#define EIGERSTATISTICS_H

#include <type_traits>
#include <cmath>

namespace lima
{
namespace Eiger
{
template <typename T,
	  typename A=typename std::conditional<std::is_integral<T>::value,
					       long long, double>::type>
struct Statistics
{
  int n;
  A sx;
  A sx2;
  T xmin;
  T xmax;

  Statistics()
  { reset(); }

  void reset()
  { sx = sx2 = xmin = xmax = n = 0; }

  void add(T x)
  {
    sx += x;
    sx2 += x * x;
    ++n;
    if (x < xmin)
      xmin = x;
    if (x > xmax)
      xmax = x;
  }

  operator bool() const
  { return n; }

  double ave() const
  { return n ? (double(sx) / n) : 0; }

  double std() const
  {
    if (!n)
      return 0;
    double xm = ave();
    return std::sqrt(double(sx2) / n - xm * xm);
  }
};

struct StreamStatistics
{
  Statistics<int> stat_size;
  Statistics<double> stat_time;

  void reset()
  {
    stat_size.reset();
    stat_time.reset();
  }

  void add(int size, double elapsed)
  {
    stat_size.add(size);
    stat_time.add(elapsed);
  }

  operator bool() const
  { return stat_size && stat_time; }

  int n() const
  { return std::min(stat_size.n, stat_time.n); }

  double ave_size() const
  { return stat_size.ave(); }

  double ave_time() const
  { return stat_time.ave(); }

  double ave_speed() const
  { return *this ? (ave_size() / ave_time()) : 0; }
};

template <typename T>
std::ostream& operator <<(std::ostream& os, const Statistics<T>& s)
{
  os << "<n=" << s.n;
  if (s)
    os << ", ave=" << s.ave() << ", std=" << s.std();
  return os << ">";
}

inline
std::ostream& operator <<(std::ostream& os, const StreamStatistics& s)
{
  return os << "<size=" << s.stat_size << ", time=" << s.stat_time << ", "
	    << "speed=" << (s.ave_speed() / 1e9) << ">";
}

} // namespace Eiger
} // namespace lima

#endif	// EIGERSTATISTICS_H
