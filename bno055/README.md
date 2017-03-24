# BNO055 IMU driver

This driver is meant to control the BNO055 IMU device, in a ChibiOS project.
It provides functions for a simple use of this component, but can be extended
in the future to provide other functions, according to the needs.

## Installation manual:
   In order to include this driver in your project, perform the following steps:
   - Set the USE_BNO055 symbol to TRUE in the drivers.mk file
   - Include the drivers.mk file in the main Makefile of your project (use the
       other includes as model to do this)
   - Add $(DRVSRC) in CSRC and $(DRVINC) in INCDIR
   And that's all. Easy, no?
   Simply include imudriver.h in your source code files to use the API.

## User manual:
  - Call the **initIMU()** function first, as it configures all what is required
    for a proper use of the device.
    This function set the device in IMU fusion mode, which allow the user to
    access to most of the useful data: euler angle, quaternions,...
  - Call the **setHeading()** function to fix the initial heading of the device.
  - Then, call the **getHeading()**,  **getRoll()** & **getPitch()** function.


## For the robotic cup:
  When initialising the robot, once we know which color (and thus side) we are,
  set initial heading to :
  - alpha if color1
  - (alpha + 180) % 360 (in degrees) if color2

This way, angles will be the same in both cases.