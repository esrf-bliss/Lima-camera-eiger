//###########################################################################
// This file is part of LImA, a Library for Image Acquisition
//
// Copyright (C) : 2009-2015
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
#ifndef EIGERAPI_AUTOMUTEX_H
#define EIGERAPI_AUTOMUTEX_H

#include <pthread.h>

namespace eigerapi
{

//Lock class
class Lock
{
public:
  Lock(pthread_mutex_t *aLock,bool aLockFlag = true) :
    _lock(aLock),_lockFlag(false)
  {if(aLockFlag) lock();}
  
  ~Lock() {unLock();}
  void lock() 
  {
    if(!_lockFlag)
      while(pthread_mutex_lock(_lock)) ;
    _lockFlag = true;
  }
  void unLock()
  {
    if(_lockFlag)
      {
	_lockFlag = false;
	pthread_mutex_unlock(_lock);
      }
  }

private:
  pthread_mutex_t *_lock;
  bool   _lockFlag;
};

class Unlock
{
public:
  Unlock(Lock& aLock) :
    _lock(aLock)
  {_lock.unLock();}

  ~Unlock()
  {_lock.lock();}      

private:
  Lock& _lock;
};
 
} // namespace eigerapi

#endif // EIGERAPI_AUTOMUTEX_H
