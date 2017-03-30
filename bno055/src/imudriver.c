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
#define UNIT_MASK   0x04U
#define UNIT_OFFSET 2U

#define ANDROID_MASK 0b10000000U
#define WINDOWS_MASK 0b01111111U
#define FORMAT_MASK  0b10000000U
#define FORMAT_OFFSET 7U

/******************************************************************************/
/*                            Local type                                      */
/******************************************************************************/
/* without object */

/******************************************************************************/
/*                             Local variable                                 */
/******************************************************************************/
/**
 * Buffer to store the data to send through the I2C bus.
 */
static uint8_t i2c_tx_buffer[I2C_TX_BUFFER_SIZE];

/**
 * Pointer to the I2C driver to use for the BNO055.
 */
static I2CDriver* bno055_i2c_driver_ptr;

/******************************************************************************/
/*                           Global variables                                 */
/******************************************************************************/
const I2CConfig imu_i2c_conf = {
	0x20420F13, /* cf table 141 in reference manual for explanation */
	0x00000001, /* peripheral enable */
	0 /* nothing to do, all fields controlled by the driver */
};

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

	status = setMode(OPERATION_MODE_CONFIG);
	if (status != NO_ERROR) {
		return status;
	}

	status = read_register(BNO055_ADDRESS, BNO055_UNIT_SEL_ADDR, &unit_reg, 1);
	if (status != NO_ERROR) {
		return status;
	}

	if (unit == DEGREE) {
		unit_reg &= DEGREE_MASK;
	} else {
		unit_reg |= RADIAN_MASK;
	}

	status = write_register(BNO055_ADDRESS, BNO055_UNIT_SEL_ADDR, unit_reg);
	if (status != NO_ERROR) {
		return status;
	}

	return setMode(OPERATION_MODE_IMUPLUS);
}

extern int32_t getUnit(imu_unit_t* unit)
{

	int32_t status;
	uint8_t unit_reg;

	if (unit == NULL) {
		status = INVALID_PARAMETER;
	} else {
		status = read_register(BNO055_ADDRESS, BNO055_UNIT_SEL_ADDR, &unit_reg, 1);
		*unit = (imu_unit_t)((unit_reg & UNIT_MASK) >> UNIT_OFFSET);
	}

	return status;
}

extern int32_t setFormat(imu_format_t format) {
	uint8_t unit_reg;
	int32_t status;

	status = setMode(OPERATION_MODE_CONFIG);
	if (status != NO_ERROR) {
		return status;
	}

	status = read_register(BNO055_ADDRESS, BNO055_UNIT_SEL_ADDR, &unit_reg, 1);
	if (status != NO_ERROR) {
		return status;
	}

	if (format == ANDROID) {
		unit_reg |= ANDROID_MASK;
	} else {
		unit_reg &= WINDOWS_MASK;
	}

	status = write_register(BNO055_ADDRESS, BNO055_UNIT_SEL_ADDR, unit_reg);
	if (status != NO_ERROR) {
		return status;
	}

	return setMode(OPERATION_MODE_IMUPLUS);
}

extern int32_t getFormat(imu_format_t* format)
{
	uint8_t unit_reg;
	int32_t status;

	status = read_register(BNO055_ADDRESS, BNO055_UNIT_SEL_ADDR, &unit_reg, 1);
	*format = (imu_format_t)((unit_reg & FORMAT_MASK) >> FORMAT_OFFSET);

	return status;
}

extern int16_t getHeading(void)
{
	static int16_t raw_angle;
	int16_t status;
	status = read_register(BNO055_ADDRESS, BNO055_EULER_H_MSB_ADDR, ((uint8_t*)&raw_angle) + 1, 1);

	if (status != NO_ERROR)	{
		raw_angle = ANGLE_ERROR;
	} else {

		status = read_register(BNO055_ADDRESS, BNO055_EULER_H_LSB_ADDR, (uint8_t*)&raw_angle, 1);

		if (status != NO_ERROR) {
			raw_angle = ANGLE_ERROR;
		}
	}

    return raw_angle;
}

extern int16_t getPitch(void) {
	static int16_t raw_angle;
	int32_t status;

	status = read_register(BNO055_ADDRESS, BNO055_EULER_P_MSB_ADDR, ((uint8_t*)&raw_angle) + 1, 1);

	if (status != NO_ERROR) {
		raw_angle = ANGLE_ERROR;
	} else {

		status = read_register(BNO055_ADDRESS, BNO055_EULER_P_LSB_ADDR, (uint8_t*)&raw_angle, 1);

		if (status != NO_ERROR) {
			raw_angle = ANGLE_ERROR;
		}
	}

    return raw_angle;
}

extern int16_t getRoll(void) {
    static int16_t raw_angle;
	int32_t status;

	status = read_register(BNO055_ADDRESS, BNO055_EULER_R_MSB_ADDR, ((uint8_t*)&raw_angle) + 1, 1);

	if (status != NO_ERROR) {
		raw_angle = ANGLE_ERROR;
	} else {
		status = read_register(BNO055_ADDRESS, BNO055_EULER_R_LSB_ADDR, (uint8_t*)&raw_angle, 1);

		if (status != NO_ERROR) {
			raw_angle = ANGLE_ERROR;
		}
	}

    return raw_angle;
}
