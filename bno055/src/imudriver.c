#include "imudriver.h"
#include "bno055.h"
#include "i2c-cache.h"
#include "ch.h"

#define IMU_HEADING 0
#define IMU_ROLL 1
#define IMU_PITCH 2

static struct device_cache *cache = NULL;

static double headingOffset = 0.0;
static double pitchOffset = 0.0;
static double rollOffset = 0.0;

static int headingRotationDirection = 0;

#define SYS_TRIGGER_RST_SYS 0x20 /* Reset the device */

#define TIMEOUT_I2C MS2ST(4)
#define I2C_TX_BUFFER_SIZE 5

/******************************************************************************/
/*                             Locale variable                                */
/******************************************************************************/
static uint8_t i2c_tx_buffer[I2C_TX_BUFFER_SIZE];

static I2CDriver* bno055_i2c_driver_ptr;

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
 * @return An value indicating success, else an error code.
 * @retval MSG_OK Transmission succeeded.
 * @retval MSG_RESET An I2C error occured.
 * @retval MSG_TIMEOUT A timeout occured.
 */
static msg_t write_register(uint8_t addr, uint8_t reg_addr, uint8_t value) {
	i2c_tx_buffer[0] = reg_addr;
	i2c_tx_buffer[1] = value;
	return i2cMasterTransmitTimeout(bno055_i2c_driver_ptr, addr, i2c_tx_buffer, 2, NULL, 0, TIMEOUT_I2C);
}

/**
 * @brief Read the content of a register.
 *
 * @param[in] addr Address of the device to read from.
 * @param[in] reg_addr Address of the register to read from.
 */
static msg_t read_register(uint8_t addr, uint8_t reg_addr) {
	i2c_tx_buffer[0] = reg_addr;
	return i2cMasterTransmitTimeout(&I2CD1, addr, i2c_tx_buffer, 1, i2c_rx_buffer, 2, TIMEOUT_I2C);
}

/**
* @brief Change the mode in which the BNO055 operates.
*
* @param[in] mode The mode to set.
*/
static void setMode(bno055_opmode_t mode) {
    I2Cwrite8(BNO055_ADDRESS, BNO055_OPR_MODE_ADDR, mode);
    chThdSleepMilliseconds(30);
}
/******************************************************************************/
/*                              Public functions                              */
/******************************************************************************/
extern int initIMU(void) {
    /* Make sure we have the right device */
    uint8_t id = I2Cread8(BNO055_ADDRESS, BNO055_CHIP_ID_ADDR);
    if (id != BNO055_ID) {
        //printf("initIMU : ERROR wrong device ID, check IMU is connected to the I2C bus\n");
        return INVALID_DEVICE;
    }

    /* Switch to config mode (just in case since this is the default) */
    setMode(OPERATION_MODE_CONFIG);

    /* Reset */
    I2Cwrite8(BNO055_ADDRESS, BNO055_SYS_TRIGGER_ADDR, SYS_TRIGGER_RST_SYS);
    while (((uint8_t) I2Cread8(BNO055_ADDRESS, BNO055_CHIP_ID_ADDR)) != BNO055_ID) {
        chThdSleepMilliseconds(10);
    }
    delayMilli(50);

    /* Set to normal power mode */
    I2Cwrite8(BNO055_ADDRESS, BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
    delayMilli(10);

    I2Cwrite8(BNO055_ADDRESS, BNO055_PAGE_ID_ADDR, 0);

    I2Cwrite8(BNO055_ADDRESS, BNO055_SYS_TRIGGER_ADDR, 0x0);
    delayMilli(10);
    /* Set the  operating mode (see section 3.3) */
    setMode(OPERATION_MODE_IMUPLUS);
    delayMilli(100);

    /* set up the cache system */
    cache = initCache(BNO055_ADDRESS, 0, 3, 0, 0);
    cache->r16_cmds[IMU_HEADING] = BNO055_EULER_H_LSB_ADDR;
    cache->r16_cmds[IMU_ROLL] = BNO055_EULER_R_LSB_ADDR;
    cache->r16_cmds[IMU_PITCH] = BNO055_EULER_P_LSB_ADDR;

    return 0;
}

double getHeading() {
    int val = c_read16(cache, IMU_HEADING) & 0x1FFF;
    double result = val/16.0;

    if(headingRotationDirection)
        result = 360 -  result;

    result = result - headingOffset;
    if(result >= 360)
        result -= 360;
    if(result < 0)
        result += 360;
    return result;
}
void setHeading(double heading) {
    if(heading >= 0 && heading < 360) {
        headingOffset = 0;
        headingOffset = getHeading() - heading;
    }
}
void setHeadingRotationDirection(int direction) {
    headingRotationDirection = direction ? 1 : 0;
}
int getHeadingRotationDirection() {
    return headingRotationDirection;
}
double getPitch() {
    int val = c_read16(cache, IMU_PITCH);
    double result = val/16;
    result = result - pitchOffset;
    if(result >= 360)
        result -= 360;
    if(result < 0)
        result += 360;
    return result;
}
void setPitch(double pitch) {
    if(pitch >= 0 && pitch < 360) {
        pitchOffset = 0;
        pitchOffset = getPitch() - pitch;
    }
}
double getRoll() {
    int val = c_read16(cache, IMU_ROLL);
    double result = val/16;
    result = result - rollOffset;
    if(result >= 360)
        result -= 360;
    if(result < 0)
        result += 360;
    return result;
}
void setRoll(double roll) {
    if(roll >= 0 && roll < 360) {
        rollOffset = 0;
        rollOffset = getRoll() - roll;
    }
}
