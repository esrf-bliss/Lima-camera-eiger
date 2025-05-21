###########################################################################
# This file is part of LImA, a Library for Image Acquisition
#
#  Copyright (C) : 2009-2019
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

cmake_minimum_required(VERSION 3.10)

project(eiger)

# Include additional modules that are used inconditionnaly
include(GNUInstallDirs)
include(GenerateExportHeader)

# If conda build, always set lib dir to 'lib'
if($ENV{CONDA_BUILD})
  set(CMAKE_INSTALL_LIBDIR "lib")
endif()

# Set lower / upper case project names
string(TOUPPER "${PROJECT_NAME}" PROJECT_NAME_UPPER)
string(TOLOWER "${PROJECT_NAME}" PROJECT_NAME_LOWER)

# Set targets export name (used by lima and dependencies)
set(TARGETS_EXPORT_NAME "${PROJECT_NAME_LOWER}-targets")
#set(namespace "${PROJECT_NAME}::")

# Check if project is being used directly or via add_subdirectory
set(CAMERA_MASTER_PROJECT OFF)
if (CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
  set(CAMERA_MASTER_PROJECT ON)
endif()

if (CAMERA_MASTER_PROJECT)
  find_package(Lima REQUIRED COMPONENTS devel tools)
endif()

# CMake additional macros
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake ${LIMA_CMAKE_INCLUDE_DIRS} ${CMAKE_MODULE_PATH})

include(LimaTools)

# Set version
include(project_version)

# Enable python binding code compilation using sip generator
if (CAMERA_MASTER_PROJECT)
  option(LIMA_ENABLE_PYTHON "compile python binding code?" OFF)
  option(LIMA_ENABLE_PYTANGO_SERVER "install python tango server?" OFF)

  if (LIMA_ENABLE_PYTHON)
    limatools_find_python_and_sip()
  endif()
endif()


find_package(CURL REQUIRED)
find_package(LZ4 REQUIRED)
find_package(ZMQ REQUIRED)
find_package(JsonCpp REQUIRED)

file(GLOB_RECURSE EIGER_INCS "${CMAKE_CURRENT_SOURCE_DIR}/include/*.h")

set(EIGER_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/sdk" CACHE PATH "location of eiger sdk")

add_library(eiger SHARED
  src/EigerCamera.cpp
  src/EigerInterface.cpp
  src/EigerDetInfoCtrlObj.cpp
  src/EigerSyncCtrlObj.cpp
  src/EigerEventCtrlObj.cpp
  src/EigerDecompress.cpp
  src/EigerSavingCtrlObj.cpp
  src/EigerRoiCtrlObj.cpp
  src/EigerStream.cpp
  src/EigerStreamInfo.cpp
  sdk/linux/EigerAPI/src/CurlLoop.cpp
  sdk/linux/EigerAPI/src/Requests.cpp
  ${EIGER_INCS}
)

# Generate export macros
generate_export_header(eiger)

# Set version and output name
set_target_properties(eiger PROPERTIES
  OUTPUT_NAME "lima${PROJECT_NAME_LOWER}"
  VERSION "${PROJECT_VERSION}"
  SOVERSION "${PACKAGE_VERSION_MAJOR}.${PACKAGE_VERSION_MINOR}")

# Add dependencies
target_include_directories(eiger
  PUBLIC "$<BUILD_INTERFACE:${EIGER_SDK_ROOT}/linux/EigerAPI/include>"
  PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/tools/src>"
  PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>"           # For export header
  PUBLIC "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>"
)

target_link_libraries(eiger PUBLIC
  ${CURL_LIBRARIES}
  ${LZ4_LIBRARIES}
  ${ZMQ_LIBRARIES}
  JsonCpp::JsonCpp
  limacore
  h5bshuf
)

if(WIN32)
  target_compile_definitions(eiger
    PRIVATE eiger_EXPORTS
    PUBLIC NOMINMAX)
  set_target_properties(eiger PROPERTIES PREFIX "lib" IMPORT_PREFIX "lib")
endif()

# Binding code for python
if(LIMA_ENABLE_PYTHON)
  limatools_run_sip_for_camera(eiger)
endif()

# Generate and install package config file and version
if(CAMERA_MASTER_PROJECT)
  set(PROJECT_LIBRARIES eiger)
  set(SIP_INSTALL_DIR ${CMAKE_INSTALL_DATADIR}/sip/lima)
  set(CMAKE_INSTALL_DIR ${CMAKE_INSTALL_DATADIR}/cmake/lima)
  include(package_config)
endif()

## Installation

if(TARGET jsoncpp_interface)
  install(
    TARGETS jsoncpp_interface
    EXPORT "${TARGETS_EXPORT_NAME}"
  )
endif()

install(
  TARGETS eiger
  EXPORT "${TARGETS_EXPORT_NAME}"
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}   # import library
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}   # .so files are libraries
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}   # .dll files are binaries
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}  # this does not actually install anything (but used by downstream projects)
)

install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
  COMPONENT devel
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  FILES_MATCHING PATTERN "*.h"
)

install(
  FILES
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}/project_version.h
    ${PROJECT_BINARY_DIR}/eiger_export.h
  COMPONENT devel
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

if(LIMA_ENABLE_PYTHON)
  install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/python/
    DESTINATION "$<PATH:CMAKE_PATH,NORMALIZE,${Python3_SITEARCH}/Lima/Eiger>"
  )
  if (LIMA_ENABLE_PYTANGO_SERVER)
    add_subdirectory(tango)
  endif()
endif()

## Tests
if(CAMERA_ENABLE_TESTS)
  enable_testing()
  add_subdirectory(test)
endif()
