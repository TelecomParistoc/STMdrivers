DRVSRC += $(DRIVERS)/bno055/src/imudriver.c

DRVINC += $(DRIVERS)/bno055/inc/

UART=1
I2C=2

DRIVERDEFS = -DUART=${UART} -DI2C=${I2C}
