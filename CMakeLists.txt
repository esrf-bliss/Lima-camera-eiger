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

TARGET_LINK_LIBRARIES(limaeiger limacore)
TARGET_LINK_LIBRARIES(limaeiger ${EIGER_LIB_CURL})
TARGET_LINK_LIBRARIES(limaeiger ${EIGER_LIB_LZ4})
TARGET_LINK_LIBRARIES(limaeiger ${EIGER_LIB_ZMQ})
TARGET_LINK_LIBRARIES(limaeiger ${EIGER_LIB_JSONCPP})

SET_TARGET_PROPERTIES(limaeiger PROPERTIES VERSION ${eiger_vers} SOVERSION "1")

INSTALL(TARGETS limaeiger LIBRARY DESTINATION lib)

IF(COMPILE_SIP)
    SET(NAME "eiger")
    SET(INCLUDES)
    FILE(GLOB sipfiles RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}/sip" "${CMAKE_CURRENT_SOURCE_DIR}/sip/*.sip")
    FOREACH(sipfile ${sipfiles})
        SET(INCLUDES 
        "${INCLUDES} 
        %Include ${sipfile}")
    ENDFOREACH()

    CONFIGURE_FILE(${CMAKE_SOURCE_DIR}/sip/limamodules.sip.in sip/limaeiger.sip)
    SET(SIP_CONCAT_PARTS 1)
    SET(SIP_INCLUDES ${SIP_INCLUDES}
        "${CMAKE_SOURCE_DIR}/third-party/Processlib/sip"
        "${CMAKE_BINARY_DIR}/sip/core"
        "${CMAKE_SOURCE_DIR}/third-party/Processlib/tasks/sip"
        "${CMAKE_SOURCE_DIR}/common/sip"
        "${CMAKE_SOURCE_DIR}/hardware/sip"
        "${CMAKE_SOURCE_DIR}/control/sip"
        "${CMAKE_SOURCE_DIR}/control/software_operation/sip"
        "${CMAKE_CURRENT_SOURCE_DIR}/sip")
    set(SIP_DISABLE_FEATURES WITH_CONFIG)
    set(SIP_CHECK_EXC ON)
    ADD_SIP_PYTHON_MODULE(limaeiger ${CMAKE_CURRENT_BINARY_DIR}/sip/limaeiger.sip)
    TARGET_INCLUDE_DIRECTORIES(python_module_limaeiger PRIVATE
                               ${PYTHON_INCLUDE_DIRS}
                               "${CMAKE_SOURCE_DIR}/sip"
                               "${CMAKE_SOURCE_DIR}/sip/core")
    TARGET_LINK_LIBRARIES(python_module_limaeiger limaeiger)

    INSTALL(FILES python/__init__.py DESTINATION "${PYTHON_SITE_PACKAGES_DIR}/Lima/Eiger")
ENDIF()
