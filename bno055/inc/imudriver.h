/* IMU (Inertial Motion Unit) driver for bno055
 * keeps track of the robot's relative orientation
 * functionnalities :
 *  > get/set angle unit
 *  > get/set angle format
 *  > get heading, roll and pitch
 */

#ifndef IMUDRIVER_H
#define IMUDRIVER_H

#include "hal.h"

#define ANGLE_ERROR 0xFFFF
#define ACCELERATION_ERROR 0xFFFF
#define MAX_RANGE 5760

typedef enum {
    CLOCKWISE = 0U,
    COUNTER_CLOCKWISE = 1U
} imu_rotation_direction_t;

typedef enum {
    DEGREE = 0U,
    RADIAN = 1U
} imu_unit_t;

typedef enum {
    WINDOWS = 0U,
    ANDROID = 1U
} imu_format_t;

/**
 * Configuration for the I2C driver.
 */
extern const I2CConfig imu_i2c_conf;

/**
 * @brief Initialize the bno055 and set it in IMU mode (fusion between accelerometer and gyro).
 *
 * @details This function has to be called before any other operation,
 *          UNLESS the initMotorDriver() was called : in that case calling
 *          initIMU() is redundant.
 *
 * @param[in] A pointer to the I2C driver to use to communicate with the device.
 *
 * @return An int indicating success or failure.
 * @retval NO_ERROR  Success.
 * @retval INVALID_PARAMETER i2c_driver is null.
 * @retval INVALID_DEVICE Device responding is not a BNO055.
 * @retval I2C_RESET Error in the I2C communication.
 * @retval I2C_TIMEOUT Timeout during the I2C exchange.
 * @retval UNKNWON_ERROR An unknown error occured.
 */
extern int initIMU(I2CDriver* i2c_driver);

/**
 * @brief Set the unit of the euler angles.
 *
 * @param[in] unit The unit to use.
 *
 * @return An int32_t indicating success, else an error code.
 * @retval NO_ERROR No error.
 * @retval I2C_RESET Error in the I2C communication.
 * @retval I2C_TIMEOUT Timeout in the I2C exchange.
 * @retval UNKNWON_ERROR An unknown error occurred.
 */
extern int32_t setUnit(imu_unit_t unit);

/**
 * @brief Get the unit used for the euler angle.
 *
 * @param[out] unit The unit used.
 *
 * @return An int32_t indicating success, else an error code.
 * @retval NO_ERROR No error.
 * @retval I2C_RESET Error in the I2C communication.
 * @retval I2C_TIMEOUT Timeout in the I2C exchange.
 * @retval UNKNWON_ERROR An unknown error occurred.
 */
extern int32_t getUnit(imu_unit_t* unit);

/**
 * @brief Set the data format used for the euler angles.
 *
 * @param[in] format The format to use.
 *
 * @return An int32_t indicating success, else an error code.
 * @retval NO_ERROR No error.
 * @retval I2C_RESET Error in the I2C communication.
 * @retval I2C_TIMEOUT Timeout in the I2C exchange.
 * @retval UNKNWON_ERROR An unknown error occurred.
 */
extern int32_t setFormat(imu_format_t format);

/**
 * @brief Get the format used.
 *
 * @param[out] format The format used.
 *
 * @return An int32_t indicating success, else an error code.
 * @retval NO_ERROR No error.
 * @retval I2C_RESET Error in the I2C communication.
 * @retval I2C_TIMEOUT Timeout in the I2C exchange.
 * @retval UNKNWON_ERROR An unknown error occurred.
 */
extern int32_t getFormat(imu_format_t* format);

/**
 * @brief Get heading angle.
 *
 * @details Get the heading angle, relative to the IMU start-up.
 *          The range is [0, 5760]. Value increases when turning clockwise.
 *
 * @return The heading angle.
 */
extern int16_t getHeading(void);

/**
 * @brief Get pitch angle.
 *
 * @details Get the pitch angle relative to the IMU start-up.
 *          The output range is [-2880, 2880].
 *
 * @return The pitch angle.
 */
extern int16_t getPitch(void);

/**
 * @brief Get roll angle.
 *
 * @details Get the roll angle relative to the IMU start-up.
 *          The output range is [-1440, 1440]. The value increases as the
 *          inclination increases.
 *
 * @return The roll angle.
 */
extern int16_t getRoll(void);

extern int16_t getAccelerationX(void);

extern int16_t getAccelerationY(void);

extern int16_t getAccelerationZ(void);

extern int16_t getGravityVectorX(void);

extern int16_t getGravityVectorY(void);

extern int16_t getGravityVectorZ(void);

#endif /* IMUDRIVER_H */
