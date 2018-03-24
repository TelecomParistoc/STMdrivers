# BNO055 IMU driver

This driver is meant to control the BNO055 IMU device, in a ChibiOS project.
It provides functions for a simple use of this component, but can be extended
in the future to provide other functions, according to the needs.

## Installation manual:
   In order to include this driver in your project, perform the following steps:
   - Set the USE_BNO055 symbol to TRUE in the drivers.mk file
   - Include the drivers.mk file in the main Makefile of your project (use the
       other includes as model to do this)
   - Add ```$(DRVSRC)``` in ```CSRC``` and ```$(DRVINC)``` in ```INCDIR```
   - Select the communication protocol to use. For this, simply add the following
elements to the ```UDEFS``` variable in your makefile:
            ${DRIVERDEFS} -DIMU_PROTOCOL=${<PROTOCOL_NAME>}
Replace PROTOCOL_NAME by the protocol that you want to use. For the moment, UART
and I2C are supported.

   And that's all. Easy, no?
   Simply include imudriver.h in your source code files to use the API.

## User manual:
  - Call the **initIMU()** function first, as it configures all what is required
    for a proper use of the device.
    This function set the device in IMU fusion mode, which allow the user to
    access to most of the useful data: euler angle, quaternions,...
  - Call the **setHeading()** function to fix the initial heading of the device.
  - Then, call the **getHeading()**,  **getRoll()** & **getPitch()** function.

## Documentation
   To generate a complete documentation of the API, you can use Doxygen.
   Simply run ```doxygen Doxyfile``` in the root directory of the repository and
   the documentation will be generated in the **/doc** directory.
   Open **/doc/html/index.html** with your favorite web browser to read the
   doc.

## For the robotic cup:
  When initialising the robot, once we know which color (and thus side) we are,
  set initial heading to :
  - alpha if color1
  - (alpha + 180) % 360 (in degrees) if color2

This way, angles will be the same in both cases.

# I2C configuration for the BNO055
 - Standard mode and fast mode supported
 - 7-bit address
 - address 0x28
