/* IMU (Inertial Motion Unit) driver for bno055
 * keeps track of the robot's relative orientation
 * functionnalities :
 *  > get/set heading, roll and pitch
 */

#ifndef IMUDRIVER_H
#define IMUDRIVER_H

#include "hal.h"

#define ANGLE_ERROR 0xFFFF

typedef enum {
    CLOCKWISE = 0U,
    COUNTER_CLOCKWISE = 1U
} imu_rotation_direction_t;

typedef enum {
    DEGREE = 0U,
    RADIAN = 1U
} imu_unit_t;

typedef enum {
    ANDROID = 0U,
    WINDOWS = 1U
} imu_format_t;

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
 * @brief Get heading angle.
 *
 * @details Get the heading angle relative to the last offset set.
 *          The range is [0, 5760] (if unit is degree) or [0, 324000]  (if unit
            is radian). Value increases when turning clockwise.
 *
 * @return The relative heading angle.
 */
extern int16_t getHeading(void);

/**
 * @brief Set the euler heading angle offset.
 *
 * @details WARNING : setting angles DOES NOT MOVE the robot, it offsets the angle.
 *
 * @return An int32_t indicating success, else an error code.
 * @retval NO_ERROR No error, offset set.
 * @retval INVALID_PARAMETER heading out of range.
 *
 * @param[in] heading The current heading angle.
 */
extern int32_t setHeading(int16_t heading);

/**
 * @brief Get pitch angle.
 *
 * @details Get the pitch angle relative to the last offset set.
 *          The output range is [-2880, 2880] (if unit is degrees) or [-162000, 162000]
 *          (if unit is radian).
 *
 * @return The relative pitch angle.
 */
extern int16_t getPitch(void);

/**
 * @brief Set the euler pitch angle offset.
 *
 * @details WARNING : setting angles DOES NOT MOVE the robot, it offsets the angle.
 *
 * @return An int32_t indicating success, else an error code.
 * @retval NO_ERROR No error, offset set.
 * @retval INVALID_PARAMETER pitch out of range.
 *
 * @param[in] pitch The current pitch angle.
 */
extern int32_t setPitch(int16_t pitch);

/**
 * @brief Get roll angle.
 *
 * @details Get the roll angle relative to the last offset set.
 *          The output range is [-1440, 1440] (if unit is degree) or [-81000, 81000]
 *          (if unit is radian). The value increases as the inclination increases.
 *
 * @return The relative pitch angle.
 */
extern int16_t getRoll(void);

/**
 * @brief Set the euler roll angle offset.
 *
 * @details WARNING : setting angles DOES NOT MOVE the robot, it offsets the angle.
 *
 * @return An int32_t indicating success, else an error code.
 * @retval NO_ERROR No error, offset set.
 * @retval INVALID_PARAMETER roll out of range.
 *
 * @param[in] roll The current roll angle.
 */
extern int32_t setRoll(int16_t roll);

#endif /* IMUDRIVER_H */
