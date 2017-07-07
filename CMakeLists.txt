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
set(NAME "eiger")

set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--add-needed")

set(EIGER_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/sdk" CACHE PATH "location of eiger sdk")

#SOURCES
set(${NAME}_srcs src/EigerCamera.cpp  src/EigerInterface.cpp
	 src/EigerDetInfoCtrlObj.cpp src/EigerSyncCtrlObj.cpp
	 src/EigerDecompress.cpp src/EigerSavingCtrlObj.cpp
	 src/EigerStream.cpp src/EigerDecompress.h
	 src/EigerStream.h sdk/linux/EigerAPI/src/CurlLoop.cpp
	 sdk/linux/EigerAPI/src/Requests.cpp)
	 
add_library(lima${NAME} SHARED ${${NAME}_srcs})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#only possible after cmake 3.1.3
#TARGET_COMPILE_FEATURES(limaeiger PRIVATE -std=c++11)

#INCLUDEs
target_include_directories(lima${NAME} PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include")
target_include_directories(lima${NAME} PUBLIC "${EIGER_SDK_ROOT}/linux/EigerAPI/include")
target_include_directories(lima${NAME} PUBLIC "/usr/include/jsoncpp")

#LIBRARIES
find_library(EIGER_CURL_LIB curl)
find_library(EIGER_LZ4_LIB lz4)
find_library(EIGER_ZMQ_LIB zmq)
find_library(EIGER_JSONCPP_LIB jsoncpp)
if (("${EIGER_CURL_LIB}" STREQUAL "EIGER_CURL_LIB-NOTFOUND"))
	message(FATAL_ERROR "${NAME} : libcurl not found, you can search it in cmake-gui or try to install it if it isn't here.")
endif()
if (("${EIGER_ZMQ_LIB}" STREQUAL "EIGER_ZMQ_LIB-NOTFOUND"))
	message(FATAL_ERROR "${NAME} : libzmq not found, you can search it in cmake-gui or try to install it if it isn't here.")
endif()
if (("${EIGER_JSONCPP_LIB}" STREQUAL "EIGER_JSONCPP_LIB-NOTFOUND"))
	message(FATAL_ERROR "${NAME} : libjsoncpp not found, you can search it in cmake-gui or try to install it if it isn't here.")
endif()
if (("${EIGER_LZ4_LIB}" STREQUAL "EIGER_LZ4_LIB-NOTFOUND"))
	message(FATAL_ERROR "${NAME} : liblz4 not found, you can search it in cmake-gui or try to install it if it isn't here.")
endif()

target_link_libraries(lima${NAME} limacore)
target_link_libraries(lima${NAME} ${EIGER_CURL_LIB})
target_link_libraries(lima${NAME} ${EIGER_LZ4_LIB})
target_link_libraries(lima${NAME} ${EIGER_ZMQ_LIB})
target_link_libraries(lima${NAME} ${EIGER_JSONCPP_LIB})

limatools_set_library_soversion(lima${NAME} "VERSION")
install(TARGETS lima${NAME} LIBRARY DESTINATION lib)

if(LIMA_ENABLE_PYTHON)
    limatools_run_sip_for_camera(${NAME})
    install(FILES python/__init__.py DESTINATION "${PYTHON_SITE_PACKAGES_DIR}/Lima/Eiger")
endif()
