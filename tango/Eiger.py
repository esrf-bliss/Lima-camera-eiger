############################################################################
# This file is part of LImA, a Library for Image Acquisition
#
# Copyright (C) : 2009-2026
# European Synchrotron Radiation Facility
# CS40220 38043 Grenoble Cedex 9
# FRANCE
# Contact: lima@esrf.fr
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
#         (c) - Bliss - ESRF
#=============================================================================
#
import tango
from tango import AttrWriteType, DevState
from tango.server import Device, attribute, command, device_property

from lima import core
from lima import eiger as EigerAcq
from lima.server import AttrHelper


def _make_cached_fget_fset(attr_name, enum=None):
    """Like AttrHelper.make_fget_fset, but for the few hw parameters this
    camera caches (see init_device): a real hw read takes ~100ms, so once
    the initial value has been latched in self._attr_cache, further reads
    return that cached value instead of re-querying the hardware. Writes
    always hit the hardware, and update the cache if it's tracking this
    attribute (mirrors the original get_attr_4u_with_cache behavior:
    retrigger is only cached on Eiger2 - if it's not in the cache dict,
    this falls back to a plain uncached read, exactly like the rest).
    """
    camel = AttrHelper._title_case(attr_name)
    get_name, set_name = "get" + camel, "set" + camel

    def fget(self):
        cache = self._attr_cache
        if attr_name in cache:
            return cache[attr_name]
        value = getattr(self._EigerCamera, get_name)()
        return AttrHelper.getDictKey(enum, value) if enum else value

    def fset(self, value):
        cache_value = value
        if enum:
            resolved = AttrHelper.getDictValue(enum, value.upper())
            if resolved is None:
                tango.Except.throw_exception(
                    "WrongData",
                    "Wrong value %s: %s" % (attr_name, value.upper()),
                    "LimaCCDs Class",
                )
            value = resolved
            # Cache the enum string key (what a read returns), not the
            # resolved hw value - fixes a bug where the original code
            # (CallableWriteEnumWithCache) cached the resolved value,
            # making the next read fail (str attribute, non-str cached).
            cache_value = AttrHelper.getDictKey(enum, resolved)
        getattr(self._EigerCamera, set_name)(value)
        if attr_name in self._attr_cache:
            self._attr_cache[attr_name] = cache_value

    return fget, fset


class Eiger(Device):

    core.DEB_CLASS(core.DebModule.DebModApplication, 'LimaCCDs')

    # ------------------------------------------------------------------
    #    Static properties
    # ------------------------------------------------------------------
    _EigerCamera = None
    """Reference to Lima eiger camera binding"""
    _EigerInterface = None
    """Reference to Lima eiger interface binding"""

    _ApiGeneration = {
        'Eiger1': EigerAcq.Camera.ApiGeneration.Eiger1,
        'Eiger2': EigerAcq.Camera.ApiGeneration.Eiger2,
    }
    _CountrateCorrection = {'ON': True, 'OFF': False}
    _FlatfieldCorrection = {'ON': True, 'OFF': False}
    _AutoSummation = {'ON': True, 'OFF': False}
    _EfficiencyCorrection = {'ON': True, 'OFF': False}
    _VirtualPixelCorrection = {'ON': True, 'OFF': False}
    _Retrigger = {'ON': True, 'OFF': False}
    _PixelMask = {'ON': True, 'OFF': False}
    _ThresholdDiffMode = {'ON': True, 'OFF': False}
    _CompressionType = {
        'NONE': EigerAcq.Camera.CompressionType.NoCompression,
        'LZ4': EigerAcq.Camera.CompressionType.LZ4,
        'BSLZ4': EigerAcq.Camera.CompressionType.BSLZ4,
    }
    _PluginStatus = {
        'INITIALIZING': EigerAcq.Camera.Status.Initializing,
        'READY': EigerAcq.Camera.Status.Ready,
        'ARMED': EigerAcq.Camera.Status.Armed,
        'EXPOSURE': EigerAcq.Camera.Status.Exposure,
        'FAULT': EigerAcq.Camera.Status.Fault,
    }

    # ------------------------------------------------------------------
    #    Device properties
    # ------------------------------------------------------------------
    detector_ip_address = device_property(dtype=str, default_value="")
    http_port = device_property(dtype=int, default_value=80)
    stream_port = device_property(dtype=int, default_value=9999)
    memory_mmap_file = device_property(dtype=str, default_value="")

    # ------------------------------------------------------------------
    #    Device initialization
    # ------------------------------------------------------------------
    @core.DEB_MEMBER_FUNCT
    def init_device(self):
        Device.init_device(self)

        # Leagcy code used by AttrHelper.get_attr_string_value_list
        self.__ApiGeneration = self._ApiGeneration
        self.__CountrateCorrection = self._CountrateCorrection
        self.__FlatfieldCorrection = self._FlatfieldCorrection
        self.__AutoSummation = self._AutoSummation
        self.__EfficiencyCorrection = self._EfficiencyCorrection
        self.__VirtualPixelCorrection = self._VirtualPixelCorrection
        self.__Retrigger = self._Retrigger
        self.__PixelMask = self._PixelMask
        self.__ThresholdDiffMode = self._ThresholdDiffMode
        self.__CompressionType = self._CompressionType
        self.__PluginStatus = self._PluginStatus

        # Put in cache some hardware parameters to avoid cumulative deadtime
        # That means only attribute reading return the cache value.
        # for client reading them regulary. Each hw param. reading takes ~100ms
        cached = {
            "auto_summation": self._AutoSummation,
            "countrate_correction": self._CountrateCorrection,
            "flatfield_correction": self._FlatfieldCorrection,
            "pixel_mask": self._PixelMask,
            "virtual_pixel_correction": self._VirtualPixelCorrection,
        }
        if self._EigerCamera.getApiGeneration() == EigerAcq.Camera.ApiGeneration.Eiger2:
            cached["retrigger"] = self._Retrigger

        self._attr_cache = {}
        for attr_name, enum in cached.items():
            camel = AttrHelper._title_case(attr_name)
            value = getattr(self._EigerCamera, "get" + camel)()
            self._attr_cache[attr_name] = AttrHelper.getDictKey(enum, value) if enum else value

        self.set_state(DevState.ON)

    # ------------------------------------------------------------------
    #    Commands
    # ------------------------------------------------------------------
    @command(dtype_in=str, dtype_out=(str,))
    @core.DEB_MEMBER_FUNCT
    def getAttrStringValueList(self, attr_name):
        return AttrHelper.get_attr_string_value_list(self, attr_name)

    @command
    @core.DEB_MEMBER_FUNCT
    def deleteMemoryFiles(self):
        self._EigerCamera.deleteMemoryFiles()

    @command
    @core.DEB_MEMBER_FUNCT
    def initialize(self):
        self._EigerCamera.initialize()

    @command(dtype_in=bool, doc_in="Reset statistics",
             dtype_out=(float,), doc_out="[<ave_size>, <ave_time>, <ave_speed>]")
    @core.DEB_MEMBER_FUNCT
    def latchStreamStatistics(self, reset):
        stream_stats = self._EigerInterface.latchStreamStatistics(reset)
        return [
            stream_stats.n(),
            stream_stats.ave_size(),
            stream_stats.ave_time(),
            stream_stats.ave_speed(),
        ]

    @command
    @core.DEB_MEMBER_FUNCT
    def resetHighVoltage(self):
        self._EigerCamera.resetHighVoltage()

    # ------------------------------------------------------------------
    #    Attributes with custom read/write logic
    # ------------------------------------------------------------------
    def _detector_ip_fget(self):
        ip_addr = self.detector_ip_address
        ip_addr = ip_addr.replace("100ge1", "")
        ip_addr = ip_addr.replace("10ge1", "")
        ip_addr = ip_addr.replace("ge1", "")
        return ip_addr

    detector_ip = attribute(
        dtype=str,
        access=AttrWriteType.READ,
        doc="The IP address of the detector DCU, useful to run curl commands",
        fget=_detector_ip_fget,
    )

    def _stream_last_info_fget(self):
        last_info = self._EigerInterface.getLastStreamInfo()
        return [
            str(last_info.encoding),
            str(last_info.frame_dim),
            str(last_info.packed_size),
        ]

    stream_last_info = attribute(
        dtype=(str,),
        max_dim_x=16,
        access=AttrWriteType.READ,
        doc="Information on the data stream: encoding, frame_dim and packed_size",
        fget=_stream_last_info_fget,
    )

    def _stream_stats_fget(self):
        return self.latchStreamStatistics(False)

    stream_stats = attribute(
        dtype=(float,),
        max_dim_x=16,
        access=AttrWriteType.READ,
        doc="ave_size, ave_time, ave_speed",
        fget=_stream_stats_fget,
    )

    def _has_hwroi_support_fget(self):
        return self._EigerInterface.hasHwRoiSupport()

    has_hwroi_support = attribute(
        dtype=bool,
        access=AttrWriteType.READ,
        doc="Return True if the camera supports hardware ROI",
        fget=_has_hwroi_support_fget,
    )

    def _hwroi_supported_list_fget(self):
        result = []
        for proi in self._EigerInterface.getSupportedHwRois():
            name = proi[0]
            x = proi[1].getTopLeft().x
            y = proi[1].getTopLeft().y
            w = proi[1].getSize().getWidth()
            h = proi[1].getSize().getHeight()
            result.extend([name, str(x), str(y), str(w), str(h)])
        return result

    hwroi_supported_list = attribute(
        dtype=(str,),
        max_dim_x=20,
        access=AttrWriteType.READ,
        doc='List of supported HW ROIs: ["roi1", "x", "y", "width", "height", "roi2", ...]',
        fget=_hwroi_supported_list_fget,
    )

    def _hwroi_pattern_fget(self):
        return self._EigerCamera.getHwRoiPattern()

    hwroi_pattern = attribute(
        dtype=str,
        access=AttrWriteType.READ,
        doc='"disabled", "4M-R", "4M-L" or "4M"',
        fget=_hwroi_pattern_fget,
    )

    def _model_size_fget(self):
        return self._EigerInterface.getModelSize()

    model_size = attribute(
        dtype=str,
        access=AttrWriteType.READ,
        doc="500K, 1M, 2M, 4M, 9M or 16M",
        fget=_model_size_fget,
    )

    # ------------------------------------------------------------------
    #    Attributes fully dispatched by naming convention (some cached,
    #    see _make_cached_fget_fset above)
    # ------------------------------------------------------------------
    _api_generation_fget, _ = AttrHelper.make_fget_fset(
        "api_generation", lambda self: self._EigerCamera, enum=_ApiGeneration
    )
    api_generation = attribute(
        dtype=str, access=AttrWriteType.READ, fget=_api_generation_fget
    )

    _api_version_fget, _ = AttrHelper.make_fget_fset(
        "api_version", lambda self: self._EigerCamera
    )
    api_version = attribute(
        dtype=str,
        access=AttrWriteType.READ,
        doc="The detected API version, e.g. '1.8.0'",
        fget=_api_version_fget,
    )

    _temperature_fget, _ = AttrHelper.make_fget_fset(
        "temperature", lambda self: self._EigerCamera
    )
    temperature = attribute(
        dtype=float,
        access=AttrWriteType.READ,
        doc="The sensor temperature",
        fget=_temperature_fget,
    )

    _humidity_fget, _ = AttrHelper.make_fget_fset("humidity", lambda self: self._EigerCamera)
    humidity = attribute(
        dtype=float,
        access=AttrWriteType.READ,
        doc="The humidity percentage",
        fget=_humidity_fget,
    )

    _high_voltage_state_fget, _ = AttrHelper.make_fget_fset(
        "high_voltage_state", lambda self: self._EigerCamera
    )
    high_voltage_state = attribute(
        dtype=str, access=AttrWriteType.READ, fget=_high_voltage_state_fget
    )

    _countrate_correction_fget, _countrate_correction_fset = _make_cached_fget_fset(
        "countrate_correction", enum=_CountrateCorrection
    )
    countrate_correction = attribute(
        dtype=str,
        access=AttrWriteType.READ_WRITE,
        doc="Enable or disable the countrate correction (ON, OFF)",
        fget=_countrate_correction_fget,
        fset=_countrate_correction_fset,
    )

    _flatfield_correction_fget, _flatfield_correction_fset = _make_cached_fget_fset(
        "flatfield_correction", enum=_FlatfieldCorrection
    )
    flatfield_correction = attribute(
        dtype=str,
        access=AttrWriteType.READ_WRITE,
        doc="Enable or disable the internal (vs. lima) flatfield correction (ON, OFF)",
        fget=_flatfield_correction_fget,
        fset=_flatfield_correction_fset,
    )

    _auto_summation_fget, _auto_summation_fset = _make_cached_fget_fset(
        "auto_summation", enum=_AutoSummation
    )
    auto_summation = attribute(
        dtype=str,
        access=AttrWriteType.READ_WRITE,
        doc="If enabled image depth is bpp32, if not image depth is bpp16 (ON, OFF)",
        fget=_auto_summation_fget,
        fset=_auto_summation_fset,
    )

    _efficiency_correction_fget, _efficiency_correction_fset = AttrHelper.make_fget_fset(
        "efficiency_correction", lambda self: self._EigerCamera, enum=_EfficiencyCorrection
    )
    efficiency_correction = attribute(
        dtype=str,
        access=AttrWriteType.READ_WRITE,
        doc="Enable or disable the efficiency correction (ON, OFF)",
        fget=_efficiency_correction_fget,
        fset=_efficiency_correction_fset,
    )

    _virtual_pixel_correction_fget, _virtual_pixel_correction_fset = _make_cached_fget_fset(
        "virtual_pixel_correction", enum=_VirtualPixelCorrection
    )
    virtual_pixel_correction = attribute(
        dtype=str,
        access=AttrWriteType.READ_WRITE,
        doc="Enable or disable the virtual-pixel correction (ON, OFF)",
        fget=_virtual_pixel_correction_fget,
        fset=_virtual_pixel_correction_fset,
    )

    _retrigger_fget, _retrigger_fset = _make_cached_fget_fset("retrigger", enum=_Retrigger)
    retrigger = attribute(
        dtype=str,
        access=AttrWriteType.READ_WRITE,
        doc="Enable or disable the retrigger mode (ON, OFF)",
        fget=_retrigger_fget,
        fset=_retrigger_fset,
    )

    _pixel_mask_fget, _pixel_mask_fset = _make_cached_fget_fset("pixel_mask", enum=_PixelMask)
    pixel_mask = attribute(
        dtype=str,
        access=AttrWriteType.READ_WRITE,
        doc="Enable or disable the pixel mask correction (ON, OFF)",
        fget=_pixel_mask_fget,
        fset=_pixel_mask_fset,
    )

    _compression_type_fget, _compression_type_fset = AttrHelper.make_fget_fset(
        "compression_type", lambda self: self._EigerCamera, enum=_CompressionType
    )
    compression_type = attribute(
        dtype=str,
        access=AttrWriteType.READ_WRITE,
        doc="For the data stream, supported compression: NONE, LZ4, BSLZ4",
        fget=_compression_type_fget,
        fset=_compression_type_fset,
    )

    _threshold_energy_fget, _threshold_energy_fset = AttrHelper.make_fget_fset(
        "threshold_energy", lambda self: self._EigerCamera
    )
    threshold_energy = attribute(
        dtype=float,
        access=AttrWriteType.READ_WRITE,
        doc="The threshold energy (eV); set between 50-60% of the incoming beam energy",
        fget=_threshold_energy_fget,
        fset=_threshold_energy_fset,
    )

    _threshold_energy2_fget, _threshold_energy2_fset = AttrHelper.make_fget_fset(
        "threshold_energy2", lambda self: self._EigerCamera
    )
    threshold_energy2 = attribute(
        dtype=float,
        access=AttrWriteType.READ_WRITE,
        doc="The 2nd threshold energy (eV), used only with the threshold diff mode",
        fget=_threshold_energy2_fget,
        fset=_threshold_energy2_fset,
    )

    _threshold_diff_mode_fget, _threshold_diff_mode_fset = AttrHelper.make_fget_fset(
        "threshold_diff_mode", lambda self: self._EigerCamera, enum=_ThresholdDiffMode
    )
    threshold_diff_mode = attribute(
        dtype=str,
        access=AttrWriteType.READ_WRITE,
        doc="Enable or disable the threshold diff mode, to mask gamma x-rays (ON, OFF)",
        fget=_threshold_diff_mode_fget,
        fset=_threshold_diff_mode_fset,
    )

    _photon_energy_fget, _photon_energy_fset = AttrHelper.make_fget_fset(
        "photon_energy", lambda self: self._EigerCamera
    )
    photon_energy = attribute(
        dtype=float,
        access=AttrWriteType.READ_WRITE,
        doc="The photon energy - a helper that sets the threshold to the incoming beam energy",
        fget=_photon_energy_fget,
        fset=_photon_energy_fset,
    )

    _serie_id_fget, _ = AttrHelper.make_fget_fset("serie_id", lambda self: self._EigerCamera)
    serie_id = attribute(
        dtype=int,
        access=AttrWriteType.READ,
        doc="The current acquisition serie identifier",
        fget=_serie_id_fget,
    )

    _cam_state_fget, _ = AttrHelper.make_fget_fset("cam_state", lambda self: self._EigerCamera)
    cam_state = attribute(
        dtype=str,
        access=AttrWriteType.READ,
        doc="The internal camera status",
        fget=_cam_state_fget,
    )

    # naming-convention would compute getPluginStatus; the underlying hw
    # method is really named getStatus (see the original __getattr__
    # override) - "status" (not "plugin_status") drives the get/set names.
    _plugin_status_fget, _ = AttrHelper.make_fget_fset(
        "status", lambda self: self._EigerCamera, enum=_PluginStatus
    )
    plugin_status = attribute(
        dtype=str,
        access=AttrWriteType.READ,
        doc="The camera plugin status",
        fget=_plugin_status_fget,
    )

    _dynamic_pixel_depth_fget, _dynamic_pixel_depth_fset = AttrHelper.make_fget_fset(
        "dynamic_pixel_depth", lambda self: self._EigerCamera
    )
    dynamic_pixel_depth = attribute(
        dtype=bool,
        access=AttrWriteType.READ_WRITE,
        fget=_dynamic_pixel_depth_fget,
        fset=_dynamic_pixel_depth_fset,
    )


#----------------------------------------------------------------------------
# Plugins
#----------------------------------------------------------------------------

def get_control(detector_ip_address="0",
                 _Eiger=Eiger, _Camera=EigerAcq.Camera, _Interface=EigerAcq.Interface,
                 **keys):
    camera = _Eiger._EigerCamera
    interface = _Eiger._EigerInterface

    if camera is None:
        http_port = keys.pop('http_port', 80)
        stream_port = keys.pop('stream_port', 9999)
        mmap_file = keys.pop('memory_mmap_file', None)

        camera = _Camera(detector_ip_address, http_port=http_port, stream_port=stream_port)
        if mmap_file is not None:
            print(f"Using memory map file {mmap_file}")
            interface = _Interface(camera, mmap_file.encode())
        else:
            interface = _Interface(camera)

    _Eiger._EigerCamera = camera
    _Eiger._EigerInterface = interface

    return core.CtControl(interface)

def get_tango_specific_class_n_device():
    return Eiger.TangoClassClass, Eiger
