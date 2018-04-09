.. _camera-eiger:

DECTRIS EIGER
=============

.. image:: EIGER_1M_432x206.jpg

Introduction
------------

The EIGER 1M is a high performance X-Ray detector system.
It is made of two subsystems: a detector and a control server.
The control server is driven using an HTTP RESTful interface.

A C++ API for LImA has been developed at Synchrotron SOLEIL.

Prerequisite
------------

Some dependencies need to be installed:

 - libcurl
 - liblz4
 - libzmq
 - libjsoncpp

to install all dependencies on debian like system, use this command:

.. code-block:: bash

  $ sudo apt-get install libcurl4-gnutls-dev liblz4-dev libzmq3-dev libjsoncpp-dev

Installation and Module configuration
-------------------------------------

Follow the generic instructions in :ref:`build_installation`. If using CMake directly, add the following flag:

.. code-block:: sh

 -DLIMACAMERA_EIGER=true

For the Tango server installation, refers to :ref:`tango_installation`.

Initialisation and Capabilities
--------------------------------

Implementing a new plugin for new detector is driven by the LIMA framework but the developer has some freedoms to choose which standard and specific features will be made available. This section is supposed to give you the correct information regarding how the camera is exported within the LIMA framework.

Camera initialization
`````````````````````
Initialization is performed automatically within the Eigercamera object. By default the stream will be
use to retrieved images unless hardware saving is activated (CtSaving::setManagedMode(CtSaving::Hardware))

Std capabilities
````````````````

* HwDetInfo

+------------------------+-------------+--------------+--------------+--------------+
| Capability             | 1M Value    | 4M Value     | 9M Value     | 16M Value    |
+========================+=============+==============+==============+==============+
| Maximum image size     | 1030 * 1065 | 2070 * 2167  | 3110 * 3269  | 4150 * 4371  |
+------------------------+-------------+--------------+--------------+--------------+
| Pixel depth            | 12 bits     | 12 bits      | 12 bits      | 12 bits      |
+------------------------+-------------+--------------+--------------+--------------+
| Pixel size             | 75µm * 75µm | 75µm * 75µm  | 75µm * 75µm  | 75µm * 75µm  |
+------------------------+-------------+--------------+--------------+--------------+
| Maximum frame rate     | 3000Hz      | 750Hz        | 238Hz        | 133Hz        |
+------------------------+-------------+--------------+--------------+--------------+

* HwSync

  Supported trigger types are:

 - IntTrig
 - IntTrigMult
 - ExtTrigSingle
 - ExtTrigMult
 - ExtGate

* There is no hardware support for binning or roi.
* There is no shutter control.

Optional capabilities
---------------------

* **Cooling**

 * The detector uses liquid cooling.
 * The API allows accessing the temperature and humidity as read-only values.

| At the moment, the specific device supports the control of the following features of the Eiger Dectris API.
| (Extended description can be found in the Eiger API user manual from Dectris).

* **Photon energy**: This should be set to the incoming beam energy.
  Actually it's an helper which set the threshold
* **Threshold energy**: This parameter will set the camera detection threshold.
  This should be set between 50 to 60 % of the incoming beam energy.
* **Auto Summation** (if activate image depth is 32 and, if not image depth is 16)
* **HwSaving**:
  This detector can directly generate hd5f, if this feature is used.
  Internally Lima control the file writer Eiger module.
  This capability can be activated though the control part with CtSaving object with setManagedMode method.
* **Countrate correction**
* **Efficiency correction**
* **Flatfield correction**
* **LZ4 Compression**
* **Virtual pixel correction**
* **Pixelmask**

Configuration
-------------

* Device configuration

  The default values of the following properties must be updated in the specific device to meet your system configuration.

+------------------------+---------------------------------------------------------------------------------------------------+----------------+
| Property name          | Description                                                                                       | Default value  |
+========================+===================================================================================================+================+
| DetectorIP             | Defines the IP address of the Eiger control server (ex: 192.168.10.1)                             |      127.0.0.1 |
+------------------------+---------------------------------------------------------------------------------------------------+----------------+

How to use
----------

This is a python code of a simple acquisition:

.. code-block:: python

  from Lima import Eiger
  from lima import Core

  #------------------+
  #                  |
  #                  v ip adress or hostname
  cam = Eiger.Camera(lid32eiger1)

  hwint = Eiger.Interface(cam)
  ct = Core.CtControl(hwint)

  acq = ct.acquisition()

  # set hardware configuration
  # refer to the Dectris Eiger documentation for more information
  cam.setCountrateCorrection(False)
  cam.setFlatfieldCorrection(True)
  cam.setAutoSummation(False)
  cam.setEfficiencyCorrection(True)
  cam.setVirtualPixelCorrection(True)
  cam.setPixelMask(True)

  # read some parameters
  print (cam.getTemperature())
  print (cam.getHumidity())


  # set energy threshold in KeV
  cam.seThresholdEnery(16.0)
  cam.setPhotonEnergy(16.0)

  # setting new file parameters and autosaving mode
  saving=ct.saving()

  pars=saving.getParameters()
  pars.directory='/buffer/lcb18012/opisg/test_lima'
  pars.prefix='test1_'
  pars.suffix='.edf'
  pars.fileFormat=Core.CtSaving.EDF
  pars.savingMode=Core.CtSaving.AutoFrame
  saving.setParameters(pars)

  # set accumulation mode

  acq_pars= acq.getPars()

  # now ask for 10 msec exposure and 10 frames
  acq.setAcqExpoTime(0.01)
  acq.setNbImages(10)

  ct.prepareAcq()
  ct.startAcq()

  # wait for last image (#9) ready
  lastimg = ct.getStatus().ImageCounters.LastImageReady
  while lastimg !=9:
    time.sleep(1)
    lastimg = ct.getStatus().ImageCounters.LastImageReady

  # read the first image
  im0 = ct.ReadImage(0)
