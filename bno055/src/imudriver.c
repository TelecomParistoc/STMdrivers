#include "imudriver.h"
#include "bno055.h"
#include "tr_types.h"

/******************************************************************************/
/*                            Local macro                                    */
/******************************************************************************/
#define SYS_TRIGGER_RST_SYS 0x20U /* Reset the device */
#define SYS_TRIGGER_INTERNAL_OSC 0x00 /* Use internal oscillator */
#define PAGE_0 0U

#define TIMEOUT_I2C MS2ST(4)
#define I2C_TX_BUFFER_SIZE 5U

#define DEGREE_MASK 0b11111011U
#define RADIAN_MASK 0b00000100U

#define ANDROID_MASK 0b10000000U
#define WINDOWS_MASK 0b01111111U

#define MAX_VALUE_DEGREE 5760
#define MAX_VALUE_RADIAN 324000

#define MAX_ANGLE 360

/******************************************************************************/
/*                            Local type                                     */
/******************************************************************************/
/* without object */

/******************************************************************************/
/*                             Local variable                                */
/******************************************************************************/
/**
 * Buffer to store the data to send through the I2C bus.
 */
static uint8_t i2c_tx_buffer[I2C_TX_BUFFER_SIZE];

/**
 * Pointer to the I2C driver to use for the BNO055.
 */
static I2CDriver* bno055_i2c_driver_ptr;

static int MAX_VALUE = MAX_VALUE_DEGREE;

static int16_t headingOffset = 0;

static int16_t pitchOffset = 0;

static int16_t rollOffset = 0;

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
		return 1;
	} else if (id != BNO055_ID) {
		return INVALID_DEVICE;
	}

	/* Switch to config mode (just in case since this is the default) */
	ret_msg = setMode(OPERATION_MODE_CONFIG);
	if (ret_msg != NO_ERROR) {
		return 2;
	}

	/* Reset */
	ret_msg = write_register(BNO055_ADDRESS, BNO055_SYS_TRIGGER_ADDR, SYS_TRIGGER_RST_SYS);
	if (ret_msg != NO_ERROR) {
		return 3;
	}

	chThdSleepMilliseconds(500);
	id = 0;

	do {
		ret_msg = read_register(BNO055_ADDRESS, BNO055_CHIP_ID_ADDR, &id, 1);
		chThdSleepMilliseconds(10);
	} while ((id != BNO055_ID));

	if (ret_msg != NO_ERROR) {
		return 4;
	}
	chThdSleepMilliseconds(50);

	/* Set to normal power mode */
	ret_msg = write_register(BNO055_ADDRESS, BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
	if (ret_msg != NO_ERROR) {
		return 5;
	}
	chThdSleepMilliseconds(10);

	/* why ?? */
	ret_msg = write_register(BNO055_ADDRESS, BNO055_PAGE_ID_ADDR, PAGE_0);
	if (ret_msg != NO_ERROR) {
		return 6;
	}

	/* why ?? */
	ret_msg = write_register(BNO055_ADDRESS, BNO055_SYS_TRIGGER_ADDR, SYS_TRIGGER_INTERNAL_OSC);
	if (ret_msg != NO_ERROR) {
		return 7;
	}
	chThdSleepMilliseconds(10);

	/* Set the operating mode (see section 3.3) */
	ret_msg = setMode(OPERATION_MODE_IMUPLUS);
	if (ret_msg != NO_ERROR) {
		return 8;
	}
	chThdSleepMilliseconds(100);

    return NO_ERROR;
}

extern int32_t setUnit(imu_unit_t unit) {
	uint8_t unit_reg;
	int32_t status;

	status = read_register(BNO055_ADDRESS, BNO055_UNIT_SEL_ADDR, &unit_reg, 1);
	if (status != NO_ERROR) {
		return status;
	}

	if (unit == DEGREE) {
		unit_reg &= DEGREE_MASK;
		MAX_VALUE = MAX_VALUE_DEGREE;
	} else {
		unit_reg |= RADIAN_MASK;
		MAX_VALUE = MAX_VALUE_RADIAN;
	}

	return write_register(BNO055_ADDRESS, BNO055_UNIT_SEL_ADDR, unit_reg);
}

extern int32_t setFormat(imu_format_t format) {
	uint8_t unit_reg;
	int32_t status;

	status = read_register(BNO055_ADDRESS, BNO055_UNIT_SEL_ADDR, &unit_reg, 1);
	if (status != NO_ERROR) {
		return status;
	}

	if (format == ANDROID) {
		unit_reg |= ANDROID_MASK;
	} else {
		unit_reg &= WINDOWS_MASK;
	}

	return write_register(BNO055_ADDRESS, BNO055_UNIT_SEL_ADDR, unit_reg);
}

extern int16_t getHeading(void) {
    static int16_t raw_angle;
	int32_t status;
	int16_t angle;

	status = read_register(BNO055_ADDRESS, BNO055_EULER_H_MSB_ADDR, ((uint8_t*)&raw_angle) + 1, 1);

	if (status != NO_ERROR)	{
		angle = ANGLE_ERROR;
	} else {

		status = read_register(BNO055_ADDRESS, BNO055_EULER_H_LSB_ADDR, (uint8_t*)&raw_angle, 1);

		if (status != NO_ERROR) {
			angle = ANGLE_ERROR;
		} else {
			raw_angle -= headingOffset;

			if (raw_angle < 0) {
				raw_angle += MAX_VALUE;
			}

			angle = raw_angle % MAX_VALUE;
		}
	}

    return angle;
}

extern int32_t setHeading(int16_t heading) {
	int32_t status;
	int16_t tmp[5];
	int16_t average;
	uint8_t i;

	if (heading >= 0 && heading < MAX_VALUE) {
		do {
			average = 0;
			for (i = 0; i < 5; ++i) {
				tmp[i] = getHeading();
				average += tmp[i];
			}
			average /= 5;
		} while (average != tmp[0]);

        headingOffset = 0;
        headingOffset = getHeading() - heading;
		status = NO_ERROR;
    } else {
		status = INVALID_PARAMETER;
	}

	return status;
}

extern int16_t getPitch(void) {
	static int16_t raw_angle;
	int32_t status;
	int16_t angle;

	status = read_register(BNO055_ADDRESS, BNO055_EULER_P_MSB_ADDR, ((uint8_t*)&raw_angle) + 1, 1);

	if (status != NO_ERROR) {
		angle = ANGLE_ERROR;
	} else {

		status = read_register(BNO055_ADDRESS, BNO055_EULER_P_LSB_ADDR, (uint8_t*)&raw_angle, 1);

		if (status != NO_ERROR) {
			angle = ANGLE_ERROR;
		} else {
			raw_angle -= pitchOffset;

			while (raw_angle < -(MAX_VALUE / 2)) {
				raw_angle += MAX_VALUE;
			}

			angle = raw_angle;
		}
	}

    return angle;
}

extern int32_t setPitch(int16_t pitch) {
	int32_t status;
	int16_t tmp[5];
	int16_t average;
	uint8_t i;

	if ((pitch >= -(MAX_VALUE / 2)) && (pitch < MAX_VALUE / 2)) {
		do {
			average = 0;
			for (i = 0; i < 5; i++) {
				tmp[i] = getPitch();
				average += tmp[i];
			}
			average /= 5;
		} while (average != tmp[0]);

        pitchOffset = 0;
        pitchOffset = getPitch() - pitch;
		status = NO_ERROR;
    } else {
		status = INVALID_PARAMETER;
	}

	return status;
}

extern int16_t getRoll(void) {
    static int16_t raw_angle;
	int32_t status;
	int16_t angle;

	status = read_register(BNO055_ADDRESS, BNO055_EULER_R_MSB_ADDR, ((uint8_t*)&raw_angle) + 1, 1);

	if (status != NO_ERROR) {
		angle = ANGLE_ERROR;
	} else {
		status = read_register(BNO055_ADDRESS, BNO055_EULER_R_LSB_ADDR, (uint8_t*)&raw_angle, 1);

		if (status != NO_ERROR) {
			angle = ANGLE_ERROR;
		} else {
			raw_angle -= rollOffset;

			while (raw_angle < -(MAX_VALUE / 4)) {
				raw_angle += MAX_VALUE / 2;
			}

			angle = raw_angle;
		}
	}

    return angle;
}

extern int32_t setRoll(int16_t roll) {
	int32_t status;
	int16_t tmp[5];
	int16_t average;
	uint8_t i;

	if ((roll >= -(MAX_VALUE / 4)) && (roll < (MAX_VALUE / 4))) {
		do {
			average = 0;
			for (i = 0; i < 5; i++) {
				tmp[i] = getRoll();
				average += tmp[i];
			}
			average /= 5;
		} while (average != tmp[0]);

        rollOffset = 0;
        rollOffset = getRoll() - roll;
		status = NO_ERROR;
    } else {
		status = INVALID_PARAMETER;
	}

	return status;
}
