###########################################################################
# This file is part of LImA, a Library for Image Acquisition
#
#  Copyright (C) : 2009-2017
#  European Synchrotron Radiation Facility
#  CS40220 38043 Grenoble Cedex 9 
#  FRANCE
# 
#  Contact: lima@esrf.fr
# 
#  This is free software; you can redistribute it and/or modify
#  it under the terms of the GNU General Public License as published by
#  the Free Software Foundation; either version 3 of the License, or
#  (at your option) any later version.
# 
#  This software is distributed in the hope that it will be useful,
#  but WITHOUT ANY WARRANTY; without even the implied warranty of
#  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#  GNU General Public License for more details.
# 
#  You should have received a copy of the GNU General Public License
#  along with this program; if not, see <http://www.gnu.org/licenses/>.
############################################################################
set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--add-needed")

set(EIGER_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/sdk" CACHE PATH "location of eiger sdk")

#SOURCES
set(eiger_srcs src/EigerCamera.cpp  src/EigerInterface.cpp
	 src/EigerDetInfoCtrlObj.cpp src/EigerSyncCtrlObj.cpp
	 src/EigerDecompress.cpp src/EigerSavingCtrlObj.cpp
	 src/EigerStream.cpp src/EigerDecompress.h
	 src/EigerStream.h sdk/linux/EigerAPI/src/CurlLoop.cpp
	 sdk/linux/EigerAPI/src/Requests.cpp)
	 
file(STRINGS "VERSION" eiger_vers)
add_library(limaeiger SHARED ${eiger_srcs})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#only possible after cmake 3.1.3
#TARGET_COMPILE_FEATURES(limaeiger PRIVATE -std=c++11)

#INCLUDEs
target_include_directories(limaeiger PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_include_directories(limaeiger PUBLIC "${EIGER_SDK_ROOT}/linux/EigerAPI/include")
target_include_directories(limaeiger PUBLIC "/usr/include/jsoncpp")

#LIBRARIES
find_library(EIGER_LIB_CURL curl)
find_library(EIGER_LIB_LZ4 lz4)
find_library(EIGER_LIB_ZMQ zmq)
find_library(EIGER_LIB_JSONCPP jsoncpp)
if (("${EIGER_LIB_CURL}" STREQUAL "EIGER_LIB_CURL-NOTFOUND"))
	message(FATAL_ERROR "eiger : libcurl not found, you can search it in cmake-gui or try to install it if it isn't here.")
endif()
if (("${EIGER_LIB_ZMQ}" STREQUAL "EIGER_LIB_ZMQ-NOTFOUND"))
	message(FATAL_ERROR "eiger : libzmq not found, you can search it in cmake-gui or try to install it if it isn't here.")
endif()
if (("${EIGER_LIB_JSONCPP}" STREQUAL "EIGER_LIB_JSONCPP-NOTFOUND"))
	message(FATAL_ERROR "eiger : libjsoncpp not found, you can search it in cmake-gui or try to install it if it isn't here.")
endif()
if (("${EIGER_LIB_LZ4}" STREQUAL "EIGER_LIB_LZ4-NOTFOUND"))
	message(FATAL_ERROR "eiger : liblz4 not found, you can search it in cmake-gui or try to install it if it isn't here.")
endif()

target_link_libraries(limaeiger limacore)
target_link_libraries(limaeiger ${EIGER_LIB_CURL})
target_link_libraries(limaeiger ${EIGER_LIB_LZ4})
target_link_libraries(limaeiger ${EIGER_LIB_ZMQ})
target_link_libraries(limaeiger ${EIGER_LIB_JSONCPP})

set_target_properties(limaeiger PROPERTIES VERSION ${eiger_vers} SOVERSION "1")

install(TARGETS limaeiger LIBRARY DESTINATION lib)

if(COMPILE_SIP)
    set(NAME "eiger")
    include(CameraRunSIP)
    install(FILES python/__init__.py DESTINATION "${PYTHON_SITE_PACKAGES_DIR}/Lima/Eiger")
endif()
