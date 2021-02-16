############################################################################
# This file is part of LImA, a Library for Image Acquisition
#
# Copyright (C) : 2009-2011
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
#=============================================================================
#
# file :        Eiger.py
#
# description : Python source for the Eiger and its commands.
#                The class is derived from Device. It represents the
#                CORBA servant object which will be accessed from the
#                network. All commands which can be executed on the
#                Eiger are implemented in this file.
#
# project :     TANGO Device Server
#
# copyleft :    European Synchrotron Radiation Facility
#               BP 220, Grenoble 38043
#               FRANCE
#
#=============================================================================
#          This file is generated by POGO
#    (Program Obviously used to Generate tango Object)
#
#         (c) - Software Engineering Group - ESRF
#=============================================================================
#


import PyTango
import sys

from Lima import Core
from Lima.Server import AttrHelper

#==================================================================
#   Eiger Class Description:
#
#
#==================================================================


class Eiger(PyTango.Device_4Impl):

#--------- Add you global variables here --------------------------
    Core.DEB_CLASS(Core.DebModApplication, 'LimaCCDs')

#------------------------------------------------------------------
#    Device constructor
#------------------------------------------------------------------
    def __init__(self,cl, name):
        PyTango.Device_4Impl.__init__(self,cl,name)
        self.init_device()

        self.__ApiGeneration = {'Eiger1': EigerAcq.Camera.Eiger1,
                                'Eiger2': EigerAcq.Camera.Eiger2}
        self.__CountrateCorrection = {'ON':True,
                                      'OFF':False}
        self.__FlatfieldCorrection = {'ON':True,
                                      'OFF':False}
        self.__AutoSummation = {'ON':True,
                                'OFF':False}
        self.__EfficiencyCorrection = {'ON':True,
                                       'OFF':False}
        self.__VirtualPixelCorrection = {'ON':True,
                                         'OFF':False}
        self.__Retrigger = {'ON':True,
                            'OFF':False}
        self.__PixelMask = {'ON':True,
                            'OFF':False}
        self.__ThresholdDiffMode = {'ON':True,
                                    'OFF':False}
        self.__CompressionType = {'NONE': EigerAcq.Camera.NoCompression,
                                  'LZ4': EigerAcq.Camera.LZ4,
                                  'BSLZ4': EigerAcq.Camera.BSLZ4}
        self.__PluginStatus = {'INITIALIZING': EigerAcq.Camera.Initializing,
                         'READY': EigerAcq.Camera.Ready,
                         'ARMED': EigerAcq.Camera.Armed,
                         'EXPOSURE': EigerAcq.Camera.Exposure,
                         'FAULT': EigerAcq.Camera.Fault}

#------------------------------------------------------------------
#    Device destructor
#------------------------------------------------------------------
    def delete_device(self):
        pass


#------------------------------------------------------------------
#    Device initialization
#------------------------------------------------------------------
    def init_device(self):
        self.set_state(PyTango.DevState.ON)
        self.get_device_properties(self.get_device_class())

#------------------------------------------------------------------
#    getAttrStringValueList command:
#
#    Description: return a list of authorized values if any
#    argout: DevVarStringArray
#------------------------------------------------------------------
    @Core.DEB_MEMBER_FUNCT
    def getAttrStringValueList(self, attr_name):
        return AttrHelper.get_attr_string_value_list(self,attr_name)

#==================================================================
#
#    Eiger read/write attribute methods
#
#==================================================================
    def __getattr__(self,name) :
        if name == 'read_plugin_status':
            func2call = getattr(_EigerCamera, "getStatus")
            return AttrHelper.CallableReadEnum(self.__PluginStatus, func2call)
        return AttrHelper.get_attr_4u(self,name,_EigerCamera)

#==================================================================
#
#    stream_last_info
#
#==================================================================
    @Core.DEB_MEMBER_FUNCT
    def read_stream_last_info(self, attr):
        last_info = _EigerInterface.getLastStreamInfo()
        last_info_strarr = [str(last_info.encoding),
                            str(last_info.frame_dim),
                            str(last_info.packed_size)]
        attr.set_value(last_info_strarr)

#==================================================================
#
#    stream_statistics
#
#==================================================================
    @Core.DEB_MEMBER_FUNCT
    def read_stream_stats(self, attr):
        stream_stats_arr = self.latchStreamStatistics(False)
        attr.set_value(stream_stats_arr)

    @Core.DEB_MEMBER_FUNCT
    def read_detector_ip(self, attr):
        ip_addr = self.detector_ip_address
        ip_addr = ip_addr.replace("100ge1", "")
        ip_addr = ip_addr.replace("10ge1", "")
        ip_addr = ip_addr.replace("ge1", "")
        attr.set_value(ip_addr)

#==================================================================
#
#    Eiger command methods
#
#==================================================================

#----------------------------------------------------------------------------
#                      delete all memory files
#----------------------------------------------------------------------------
    @Core.DEB_MEMBER_FUNCT
    def deleteMemoryFiles(self):
        _EigerCamera.deleteMemoryFiles()

#----------------------------------------------------------------------------
#                      initialize detector
#----------------------------------------------------------------------------
    @Core.DEB_MEMBER_FUNCT
    def initialize(self):
        _EigerCamera.initialize()

#----------------------------------------------------------------------------
#                      latch Stream statistics
#----------------------------------------------------------------------------
    @Core.DEB_MEMBER_FUNCT
    def latchStreamStatistics(self, reset):
        stream_stats = _EigerInterface.latchStreamStatistics(reset)
        return [stream_stats.n(),
                stream_stats.ave_size(),
                stream_stats.ave_time(),
                stream_stats.ave_speed()]

#----------------------------------------------------------------------------
#                      reset high voltage
#----------------------------------------------------------------------------
    @Core.DEB_MEMBER_FUNCT
    def resetHighVoltage(self):
        _EigerCamera.resetHighVoltage()

#==================================================================
#
#    EigerClass class definition
#
#==================================================================
class EigerClass(PyTango.DeviceClass):

    #    Class Properties
    class_property_list = {
        }


    #    Device Properties
    device_property_list = {
        'detector_ip_address':
        [PyTango.DevString,
         "Detector ip address",[]],
        'http_port':
        [PyTango.DevLong,
         "HTTP port number",[]],
        'stream_port':
        [PyTango.DevLong,
         "Stream port number",[]],        
        }


    #    Command definitions
    cmd_list = {
        'getAttrStringValueList':
        [[PyTango.DevString, "Attribute name"],
         [PyTango.DevVarStringArray, "Authorized String value list"]],
        'deleteMemoryFiles':
        [[PyTango.DevVoid, ""],
         [PyTango.DevVoid, ""]],
        'initialize':
        [[PyTango.DevVoid, ""],
         [PyTango.DevVoid, ""]],
        'latchStreamStatistics':
        [[PyTango.DevBoolean, "Reset statistics"],
         [PyTango.DevVarDoubleArray, "[<ave_size>, <ave_time>, <ave_speed>]"]],
        'resetHighVoltage':
        [[PyTango.DevVoid, ""],
         [PyTango.DevVoid, ""]],
        }


    #    Attribute definitions
    attr_list = {
        'api_generation':
            [[PyTango.DevString,
            PyTango.SCALAR,
            PyTango.READ]],
        'api_version':
            [[PyTango.DevString,
            PyTango.SCALAR,
            PyTango.READ]],
        'detector_ip':
            [[PyTango.DevString,
            PyTango.SCALAR,
            PyTango.READ]],
        'temperature':
            [[PyTango.DevFloat,
            PyTango.SCALAR,
            PyTango.READ]],
        'humidity':
            [[PyTango.DevFloat,
            PyTango.SCALAR,
            PyTango.READ]],
        'high_voltage_state':
            [[PyTango.DevString,
            PyTango.SCALAR,
            PyTango.READ]],
        'countrate_correction':
            [[PyTango.DevString,
            PyTango.SCALAR,
            PyTango.READ_WRITE]],
        'flatfield_correction':
            [[PyTango.DevString,
            PyTango.SCALAR,
            PyTango.READ_WRITE]],
        'auto_summation':
            [[PyTango.DevString,
            PyTango.SCALAR,
            PyTango.READ_WRITE]],
        'efficiency_correction':
            [[PyTango.DevString,
            PyTango.SCALAR,
            PyTango.READ_WRITE]],
        'virtual_pixel_correction':
            [[PyTango.DevString,
            PyTango.SCALAR,
            PyTango.READ_WRITE]],
        'retrigger':
            [[PyTango.DevString,
            PyTango.SCALAR,
            PyTango.READ_WRITE]],
        'pixel_mask':
            [[PyTango.DevString,
            PyTango.SCALAR,
            PyTango.READ_WRITE]],
        'compression_type':
            [[PyTango.DevString,
            PyTango.SCALAR,
            PyTango.READ_WRITE]],
        'threshold_energy':
            [[PyTango.DevFloat,
            PyTango.SCALAR,
            PyTango.READ_WRITE]],
        'threshold_energy2':
            [[PyTango.DevFloat,
            PyTango.SCALAR,
            PyTango.READ_WRITE]],
        'threshold_diff_mode':
            [[PyTango.DevString,
            PyTango.SCALAR,
            PyTango.READ_WRITE]],
        'photon_energy':
            [[PyTango.DevFloat,
            PyTango.SCALAR,
            PyTango.READ_WRITE]],
        'serie_id':
            [[PyTango.DevLong,
            PyTango.SCALAR,
            PyTango.READ]],
        'stream_last_info':
            [[PyTango.DevString,
            PyTango.SPECTRUM,
            PyTango.READ, 16]],
        'cam_status':
            [[PyTango.DevString,
            PyTango.SCALAR,
            PyTango.READ]],
        'plugin_status':
            [[PyTango.DevString,
            PyTango.SCALAR,
            PyTango.READ]],
        'stream_stats':
            [[PyTango.DevDouble,
            PyTango.SPECTRUM,
            PyTango.READ, 16]],
        }


#------------------------------------------------------------------
#    EigerClass Constructor
#------------------------------------------------------------------
    def __init__(self, name):
        PyTango.DeviceClass.__init__(self, name)
        self.set_type(name)

#----------------------------------------------------------------------------
# Plugins
#----------------------------------------------------------------------------
from Lima import Eiger as EigerAcq

_EigerInterface = None
_EigerCamera = None

def get_control(detector_ip_address = "0", **keys) :
    global _EigerInterface
    global _EigerCamera
    if _EigerInterface is None:
        http_port = keys.pop('http_port', 80)
        stream_port = keys.pop('stream_port', 9999)
        
        _EigerCamera = EigerAcq.Camera(detector_ip_address,
                                       http_port=http_port,
                                       stream_port=stream_port)
        _EigerInterface = EigerAcq.Interface(_EigerCamera)
    return Core.CtControl(_EigerInterface)

def get_tango_specific_class_n_device() :
    return EigerClass,Eiger
