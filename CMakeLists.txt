############################################################################
# This file is part of LImA, a Library for Image Acquisition
#
# Copyright (C) : 2009-2017
# European Synchrotron Radiation Facility
# BP 220, Grenoble 38043
# FRANCE
#
# This is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3 of the License, or
# (at your option) any later version.
#
# This software is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, see <http://www.gnu.org/licenses/>.
############################################################################
# CMakeLists.txt for Eiger camera

SET(CMAKE_SHARED_LINKER_FLAGS "-Wl,--add-needed")

SET(EIGER_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/sdk" CACHE PATH "location of eiger sdk")

#SOURCES
SET(eiger_srcs src/EigerCamera.cpp  src/EigerInterface.cpp
	 src/EigerDetInfoCtrlObj.cpp src/EigerSyncCtrlObj.cpp
	 src/EigerDecompress.cpp src/EigerSavingCtrlObj.cpp
	 src/EigerStream.cpp src/EigerDecompress.h
	 src/EigerStream.h sdk/linux/EigerAPI/src/CurlLoop.cpp
	 sdk/linux/EigerAPI/src/Requests.cpp)
	 
FILE(STRINGS "VERSION" eiger_vers)
ADD_LIBRARY(limaeiger SHARED ${eiger_srcs})
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#only possible after cmake 3.1.3
#TARGET_COMPILE_FEATURES(limaeiger PRIVATE -std=c++11)

#INCLUDEs
TARGET_INCLUDE_DIRECTORIES(limaeiger PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
TARGET_INCLUDE_DIRECTORIES(limaeiger PUBLIC "${EIGER_SDK_ROOT}/linux/EigerAPI/include")
TARGET_INCLUDE_DIRECTORIES(limaeiger PUBLIC "${COMMON_PATH}/../include/jsoncpp")

#LIBRARIES
FIND_LIBRARY(EIGER_LIB_CURL curl)
FIND_LIBRARY(EIGER_LIB_LZ4 lz4)
FIND_LIBRARY(EIGER_LIB_ZMQ zmq)
FIND_LIBRARY(EIGER_LIB_JSONCPP jsoncpp)
IF (("${EIGER_LIB_CURL}" STREQUAL "EIGER_LIB_CURL-NOTFOUND"))
	MESSAGE(FATAL_ERROR "eiger : libcurl not found, you can search it in cmake-gui or try to install it if it isn't here.")
ENDIF()
IF (("${EIGER_LIB_ZMQ}" STREQUAL "EIGER_LIB_ZMQ-NOTFOUND"))
	MESSAGE(FATAL_ERROR "eiger : libzmq not found, you can search it in cmake-gui or try to install it if it isn't here.")
ENDIF()
IF (("${EIGER_LIB_JSONCPP}" STREQUAL "EIGER_LIB_JSONCPP-NOTFOUND"))
	MESSAGE(FATAL_ERROR "eiger : libjsoncpp not found, you can search it in cmake-gui or try to install it if it isn't here.")
ENDIF()
IF (("${EIGER_LIB_LZ4}" STREQUAL "EIGER_LIB_LZ4-NOTFOUND"))
	MESSAGE(FATAL_ERROR "eiger : liblz4 not found, you can search it in cmake-gui or try to install it if it isn't here.")
ENDIF()

TARGET_LINK_LIBRARIES(limaeiger limacore)
TARGET_LINK_LIBRARIES(limaeiger ${EIGER_LIB_CURL})
TARGET_LINK_LIBRARIES(limaeiger ${EIGER_LIB_LZ4})
TARGET_LINK_LIBRARIES(limaeiger ${EIGER_LIB_ZMQ})
TARGET_LINK_LIBRARIES(limaeiger ${EIGER_LIB_JSONCPP})

SET_TARGET_PROPERTIES(limaeiger PROPERTIES VERSION ${eiger_vers} SOVERSION "1")

INSTALL(TARGETS limaeiger LIBRARY DESTINATION lib)

IF(COMPILE_SIP)
    SET(NAME "eiger")
    include(CameraRunSIP)
    INSTALL(FILES python/__init__.py DESTINATION "${PYTHON_SITE_PACKAGES_DIR}/Lima/Eiger")
ENDIF()
