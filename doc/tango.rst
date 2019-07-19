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
==================== =============== =============== =========================================================================


Attributes
----------
========================= ======= ======================= ======================================================================
Attribute name		  RW	  Type			  Description
========================= ======= ======================= ======================================================================
auto_summation		  rw	  DevString		  If enable image depth is bpp32 and, if not image depth is bpp16 **(\*)**
countrate_correction	  rw	  DevString		  Enable or disable the countrate correction **(\*)**
efficency_correction	  rw	  DevString		  Enable the efficienty correction
flatfield_correction	  rw	  DevString		  Enable or disable the internal (vs. lima) flatfield correction **(\*)**
humidity		  ro	  DevFloat		  Return the humidity percentage
pixel_mask		  rw	  DevString		  Enable or disable the pixel mask correction **(\*)**
photon_energy		  rw	  DevFloat		  The photon energy,it should be set to the incoming beam energy. Actually it’s an helper which set the threshold
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
Init			DevVoid 	DevVoid			Do not use
State			DevVoid		DevLong			Return the device state
Status			DevVoid		DevString		Return the device state as a string
getAttrStringValueList	DevString:	DevVarStringArray:	Return the authorized string value list for
			Attribute name	String value list	a given attribute name
=======================	=============== =======================	===========================================
