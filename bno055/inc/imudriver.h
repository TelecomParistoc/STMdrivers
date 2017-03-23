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
 * @retval I2C_RESET Error in the I2C connection.
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
 */
extern int32_t setUnit(imu_unit_t unit);

/**
 * @brief Change heading rotation direction.
 *
 * @details Use this function to change the rotation direction as measured by
 *          the IMU. It's useful for team change.
 *          The default value is CLOCKWISE.
 *
 * @param[in] direction The new direction.
 */
extern void setHeadingRotationDirection(imu_rotation_direction_t direction);

/**
 * brief Get the current heading rotation direction.
 *
 * @return The current heading rotation direction.
 *
 * @retval CLOCKWISE
 * @retval COUNTER_CLOCKWISE
 */
extern imu_rotation_direction_t getHeadingRotationDirection(void);

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
 * @param[in] heading The current heading angle.
 */
extern void setHeading(int16_t heading);

/**
 * @brief Get pitch angle.
 *
 * @details Get the pitch angle (in degrees) relative to the power-on position
 *          of the module.
 *
 * @return The relative pitch angle in degrees (from 0 to 360 excluded).
 */
extern double getPitch(void);

/**
 * @brief Set the euler pitch angle offset.
 *
 * @details WARNING : setting angles DOES NOT MOVE the robot, it offsets the angle.
 *
 * @param[in] pitch The current pitch angle.
 */
extern void setPitch(double pitch);

/**
 * @brief Get roll angle.
 *
 * @details Get the roll angle (in degrees) relative to the power-on position
 *          of the module.
 *
 * @return The relative pitch angle in degrees (from 0 to 360 excluded).
 */
extern double getRoll(void);

/**
 * @brief Set the euler roll angle offset.
 *
 * @details WARNING : setting angles DOES NOT MOVE the robot, it offsets the angle.
 *
 * @param[in] roll The current roll angle.
 */
extern void setRoll(double roll);

#endif /* IMUDRIVER_H */
