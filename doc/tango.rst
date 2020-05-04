Eiger Tango device
==================

This is the reference documentation of the Dectris Eiger Tango device.

you can also find some useful information about the camera models/prerequisite/installation/configuration/compilation in the :ref:`Dectris Eiger camera plugin <camera-eiger>` section.

Properties
----------
==================== =============== =============== =========================================================================
Property name	     Mandatory	     Default value   Description
==================== =============== =============== =========================================================================
detector_ip_address  Yes	     N/A     	     The ip address or the hostname of the detector computer interface 
http_port            No 	     80     	     The http port number for control API
stream_port          No 	     9999     	     The port number for the data stream API
==================== =============== =============== =========================================================================


Attributes
----------
========================= ======= ======================= ======================================================================
Attribute name		  RW	  Type			  Description
========================= ======= ======================= ======================================================================
auto_summation		  rw	  DevString		  If enable image depth is bpp32 and, if not image depth is bpp16 **(\*)**
cam_status                ro      DevString               The internal camera status
compression_type          rw      DevString               For data stream, supported compression are:
                                                           - NONE
							   - LZ4
							   - BSLZ4
countrate_correction	  rw	  DevString		  Enable or disable the countrate correction **(\*)**
efficency_correction	  rw	  DevString		  Enable the efficienty correction
flatfield_correction	  rw	  DevString		  Enable or disable the internal (vs. lima) flatfield correction **(\*)**
humidity		  ro	  DevFloat		  Return the humidity percentage
pixel_mask		  rw	  DevString		  Enable or disable the pixel mask correction **(\*)**
photon_energy		  rw	  DevFloat		  The photon energy,it should be set to the incoming beam energy. Actually it’s an helper which set the threshold
plugin_status             ro      DevString               The camera plugin status
serie_id                  ro      DevLong                 The current acquisition serie identifier
stream_last_info          ro      DevString               Information on data stream, encoding, frame_dim and packed_size
stream_stats              ro      DevDouble               ave_size, ave_time, ave_speed
threshold_energy	  rw	  DevFloat		  The threshold energy, it will set the camera detection threshold. This should be set between 50 to 60 % of the incoming beam energy.
temperature		  ro	  DevFloat		  The sensor temperature
virtual_pixel_correction  rw	  DevString		  Enable or disable the virtual-pixel correction **(\*)**
========================= ======= ======================= ======================================================================

**(\*)** These attributes can take as value **ON** or **OFF**. Please refer to the Dectris documention for more information regarding
the online corrections.


Commands
--------

=======================	=============== =======================	===========================================
Command name		Arg. in		Arg. out		Description
=======================	=============== =======================	===========================================
deleteMemoryFiles	DevVoid		DevVoid			To remove the temporary mem. files
initialize              DevVoid         DevVoid                 To initialize the detector
latchStreamStatistics   DevBoolean      DevVarDoubleArray:      If True, reset the statistics
                                         - ave_size,
					 - ave_time,
					 - ave_speed
Init			DevVoid 	DevVoid			Do not use
State			DevVoid		DevLong			Return the device state
Status			DevVoid		DevString		Return the device state as a string
getAttrStringValueList	DevString:	DevVarStringArray:	Return the authorized string value list for
			Attribute name	String value list	a given attribute name
=======================	=============== =======================	===========================================
