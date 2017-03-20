#include "imudriver.h"
#include "bno055.h"
#include "tr_types.h"

/******************************************************************************/
/*                            Locale macro                                    */
/******************************************************************************/
#define SYS_TRIGGER_RST_SYS 0x20U /* Reset the device */
#define SYS_TRIGGER_INTERNAL_OSC 0x00 /* Use internal oscillator */
#define PAGE_0 0U

#define TIMEOUT_I2C MS2ST(4)
#define I2C_TX_BUFFER_SIZE 5U

#define MAX_ANGLE 360

/******************************************************************************/
/*                            Locale type                                     */
/******************************************************************************/
typedef union {
	int16_t angle;
	uint8_t msb;
	uint8_t lsb;
} angle_t;

/******************************************************************************/
/*                             Locale variable                                */
/******************************************************************************/
/**
 * Buffer to store the data to send through the I2C bus.
 */
static uint8_t i2c_tx_buffer[I2C_TX_BUFFER_SIZE];

/**
 * Pointer to the I2C driver to use for the BNO055.
 */
static I2CDriver* bno055_i2c_driver_ptr;

/**
 * Current heading rotation direction.
 */
static imu_rotation_direction_t headingRotationDirection = CLOCKWISE;

static double headingOffset = 0.0;

static double pitchOffset = 0.0;

static double rollOffset = 0.0;

/******************************************************************************/
/*                             Private functions                              */
/******************************************************************************/

/**
 * @brief Write a value in a register.
 *
 * @param[in] addr Address of the device to write to.
 * @param[in] reg_addr Address of the register to write to.
 * @param[in] value Value to write into the register.
 *
 * @return A value indicating success, else an error code.
 * @retval NO_ERROR Transmission succeeded.
 * @retval I2C_RESET An I2C error occured.
 * @retval I2C_TIMEOUT A timeout occured.
 * @retval UNKNOWN_ERROR An unknown error occured.
 */
static int32_t write_register(uint8_t addr, uint8_t reg_addr, uint8_t value) {
	int32_t status;

	i2c_tx_buffer[0] = reg_addr;
	i2c_tx_buffer[1] = value;
	status = i2cMasterTransmitTimeout(bno055_i2c_driver_ptr, addr, i2c_tx_buffer, 2, NULL, 0, TIMEOUT_I2C);

	switch (status) {
		case MSG_RESET:
			status = I2C_RESET;
			break;
		case MSG_TIMEOUT:
			status = I2C_TIMEOUT;
			break;
		case MSG_OK:
			status = NO_ERROR;
			break;
		default:
			status = UNKNOWN_ERROR;
			break;
	}

	return status;
}

/**
 * @brief Read the content of a register.
 *
 * @details The BNO055 uses the read-after-write mechanism. You have to first
 *          write the address of the register you want to read from and then,
 *          the device will answer with the asked value.
 *
 * @param[in] addr Address of the device to read from.
 * @param[in] reg_addr Address of the register to read from.
 * @param[out] data The place where to store the value read.
 * @param[in] size The number of bytes to read.
 *
 * @return A value indicating success, else an error code.
 * @retval NO_ERROR Transmission succeeded.
 * @retval INVALID_PARAMETER NULL data or size = 0.
 * @retval I2C_RESET An I2C error occured.
 * @retval I2C_TIMEOUT A timeout occured.
 * @retval UNKNOWN_ERROR An unknown error occured.
 */
static int32_t read_register(uint8_t addr, uint8_t reg_addr, uint8_t* data, uint8_t size) {
	int32_t status;

	if ((data == NULL) || (size == 0)) {
		status = INVALID_PARAMETER;
	} else {
		i2c_tx_buffer[0] = reg_addr;
		status = i2cMasterTransmitTimeout(bno055_i2c_driver_ptr, addr, i2c_tx_buffer, 1, data, size, TIMEOUT_I2C);

		switch (status) {
			case MSG_RESET:
				status = I2C_RESET;
				break;
			case MSG_TIMEOUT:
				status = I2C_TIMEOUT;
				break;
			case MSG_OK:
				status = NO_ERROR;
				break;
			default:
				status = UNKNOWN_ERROR;
				break;
		}
	}

	return status;
}

/**
 * @brief Change the mode in which the BNO055 operates.
 *
 * @param[in] mode The mode to set.
 *
 * @return A value indicating success or error.
 * @retval NO_ERROR Transmission succeeded.
 * @retval I2C_RESET An I2C error occured.
 * @retval I2C_TIMEOUT A timeout occured.
 */
static int32_t setMode(bno055_opmode_t mode) {
    int32_t ret_msg;

    ret_msg = write_register(BNO055_ADDRESS, BNO055_OPR_MODE_ADDR, mode);
    chThdSleepMilliseconds(30);

	return ret_msg;
}

/******************************************************************************/
/*                              Public functions                              */
/******************************************************************************/
extern int initIMU(I2CDriver* i2c_driver) {
    uint8_t id;
    int32_t ret_msg;

    /* Check parameters */
    if (i2c_driver == NULL) {
        return INVALID_PARAMETER;
    }

	bno055_i2c_driver_ptr = i2c_driver;

	/* Make sure we have the right device */
	ret_msg = read_register(BNO055_ADDRESS, BNO055_CHIP_ID_ADDR, &id, 1);
	if (ret_msg != NO_ERROR) {
		return ret_msg;
	} else if (id != BNO055_ID) {
		return INVALID_DEVICE;
	}

	/* Switch to config mode (just in case since this is the default) */
	ret_msg = setMode(OPERATION_MODE_CONFIG);
	if (ret_msg != NO_ERROR) {
		return ret_msg;
	}

	/* Reset */
	ret_msg = write_register(BNO055_ADDRESS, BNO055_SYS_TRIGGER_ADDR, SYS_TRIGGER_RST_SYS);
	if (ret_msg != NO_ERROR) {
		return ret_msg;
	}

	do {
		ret_msg = read_register(BNO055_ADDRESS, BNO055_CHIP_ID_ADDR, &id, 1);
		chThdSleepMilliseconds(10);
	} while ((id != BNO055_ID) && (ret_msg == NO_ERROR));

	if (ret_msg != NO_ERROR) {
		return ret_msg;
	}
	chThdSleepMilliseconds(50);

	/* Set to normal power mode */
	ret_msg = write_register(BNO055_ADDRESS, BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
	if (ret_msg != NO_ERROR) {
		return ret_msg;
	}
	chThdSleepMilliseconds(10);

	/* why ?? */
	ret_msg = write_register(BNO055_ADDRESS, BNO055_PAGE_ID_ADDR, PAGE_0);
	if (ret_msg != NO_ERROR) {
		return ret_msg;
	}

	/* why ?? */
	ret_msg = write_register(BNO055_ADDRESS, BNO055_SYS_TRIGGER_ADDR, SYS_TRIGGER_INTERNAL_OSC);
	if (ret_msg != NO_ERROR) {
		return ret_msg;
	}
	chThdSleepMilliseconds(10);

	/* Set the operating mode (see section 3.3) */
	ret_msg = setMode(OPERATION_MODE_IMUPLUS);
	if (ret_msg != NO_ERROR) {
		return ret_msg;
	}
	chThdSleepMilliseconds(100);

    return NO_ERROR;
}

extern void setHeadingRotationDirection(imu_rotation_direction_t direction) {
	headingRotationDirection = direction;
}

extern imu_rotation_direction_t getHeadingRotationDirection(void) {
	return headingRotationDirection;
}

extern double getHeading(void) {
    angle_t raw_angle;
	int32_t status;
	double result;

	status = read_register(BNO055_ADDRESS, BNO055_EULER_H_MSB_ADDR, &raw_angle.msb, 1);

	if (status == NO_ERROR) {
		status = read_register(BNO055_ADDRESS, BNO055_EULER_H_LSB_ADDR, &raw_angle.lsb, 1);
		if (status == NO_ERROR) {
			/* why this mask ?*/
			raw_angle.angle &= 0x1FFF;
			/* 1 degree = 16 LSB */
			result = raw_angle.angle / 16.0;

			if (headingRotationDirection)
				result = MAX_ANGLE - result;

			result = result - headingOffset;

			if (result >= MAX_ANGLE) {
				result -= MAX_ANGLE;
			} else if (result < 0) {
				result += MAX_ANGLE;
			}
		}
	}

    return result;
}

extern void setHeading(double heading) {
    if (heading >= 0 && heading < MAX_ANGLE) {
        headingOffset = 0;
        headingOffset = getHeading() - heading;
    }
}

extern double getPitch(void) {
	angle_t raw_angle;
	int32_t status;
	double result;

	status = read_register(BNO055_ADDRESS, BNO055_EULER_P_MSB_ADDR, &raw_angle.msb, 1);

	if (status == NO_ERROR) {
		status = read_register(BNO055_ADDRESS, BNO055_EULER_P_LSB_ADDR, &raw_angle.lsb, 1);
		if (status == NO_ERROR) {
			/* 1 degree = 16 LSB */
			result = raw_angle.angle / 16.0;

			result = result - pitchOffset;

			if(result >= MAX_ANGLE) {
				result -= MAX_ANGLE;
			} else if(result < 0) {
				result += MAX_ANGLE;
			}
		}
	}
    return result;
}

extern void setPitch(double pitch) {
    if (pitch >= 0 && pitch < MAX_ANGLE) {
        pitchOffset = 0;
        pitchOffset = getPitch() - pitch;
    }
}

extern double getRoll(void) {
    angle_t raw_angle;
	int32_t status;
	double result;

	status = read_register(BNO055_ADDRESS, BNO055_EULER_R_MSB_ADDR, &raw_angle.msb, 1);

	if (status == NO_ERROR) {
		status = read_register(BNO055_ADDRESS, BNO055_EULER_R_LSB_ADDR, &raw_angle.lsb, 1);
		if (status == NO_ERROR) {
			/* 1 degree = 16 LSB */
			result = raw_angle.angle / 16.0;

			result = result - rollOffset;

			if(result >= MAX_ANGLE) {
				result -= MAX_ANGLE;
			} else if(result < 0) {
				result += MAX_ANGLE;
			}
		}
	}

    return result;
}

extern void setRoll(double roll) {
    if (roll >= 0 && roll < MAX_ANGLE) {
        rollOffset = 0;
        rollOffset = getRoll() - roll;
    }
}
