/*
 * rover_firmware.h
 *
 *  Created on: Jun 11, 2025
 *      Author: Miriam Vitolo
 */

#ifndef INC_ROVER_FIRMWARE_H_
#define INC_ROVER_FIRMWARE_H_

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "tim.h"
#include "encoder.h"
#include "mpu60x0.h"
#include "math.h"
#include "canManager.h"
#include "can_sender.h"
#include "pid_regulator.h"
#include "regulators_params.h"

/* ============================ */
/* Feature Configuration Flags */
/* ============================ */

/**
 * @brief Enable IMU (MPU60X0) sensor acquisition.
 *
 * Define this macro to enable the initialization and the reading of the MPU60X0
 * sensor. If undefined, the sensor is neither initialized nor read, and the IMU
 * data area (rover.mpu.gyro / rover.mpu.accel) keeps its current values.
 */
//#define USE_MPU_SENSOR

/**
 * @brief Enable CAN transmission of IMU data.
 *
 * Define this macro to enable the transmission of the IMU data (gyro XY, accel XY
 * and Z frames) over the CAN bus. This flag is independent from USE_MPU_SENSOR:
 * if the transmission is enabled but the sensor is disabled, the values currently
 * stored in rover.mpu.gyro / rover.mpu.accel are transmitted as-is. This allows
 * testing the full CAN communication without a physical MPU6050.
 */
//#define ENABLE_MPU_CAN_TX


#define USE_CAN_TX

/**
 * @brief Type definition for rover status codes.
 *
 * This type is used to represent the status returned by rover functions.
 * It is defined as an 8-bit unsigned integer.
 */
typedef uint8_t Rover_StatusTypeDef;

/**
 * @brief Type definition for motor status codes.
 *
 * This type is used to represent the status of motor operations.
 * It is defined as an 8-bit unsigned integer.
 */
typedef uint8_t Motor_StatusTypeDef;

/**
 * @brief Alias for FreeRTOS static queue definition.
 *
 * This typedef provides a more intuitive naming for the static queue structure.
 */
typedef StaticQueue_t osStaticMessageQDef_t;

/**
 * @brief Minimum PWM tension for desired output (in volts).
 */
#define MIN_PWM_TENSION_DESIRED						(1.875)

/**
 * @brief Maximum PWM tension for desired output (in volts).
 */
#define MAX_PWM_TENSION_DESIRED						(3.125)

/**
 * @brief Saturation range for input-to-voltage conversion.
 */
#define SATURATION_RANGE							(24.0)

/**
 * @brief Conversion factor from physical command to PWM voltage.
 */
#define CONVERSION_FACTOR							((MAX_PWM_TENSION_DESIRED-MIN_PWM_TENSION_DESIRED)/SATURATION_RANGE)

/**
 * @brief PWM tension corresponding to a stopped motor (in volts).
 */
#define STOP_PWM_TENSION    						(2.525)

/**
 * @brief Maximum value for the control signal applied to motors.
 */
#define UK_MAX              (12)

/**
 * @brief Minimum value for the control signal applied to motors.
 */
#define UK_MIN              (-UK_MAX)

/**
 * @brief Maximum allowed PWM tension.
 */
#define MAX_PWM_TENSION		(3.3)

/**
 * @brief PWM channel for rear-left motor.
 */
#define TIM_PWM_RL TIM_CHANNEL_1

/**
 * @brief PWM channel for rear-right motor.
 */
#define TIM_PWM_RR TIM_CHANNEL_2

/**
 * @brief PWM channel for front-left motor.
 */
#define TIM_PWM_FL TIM_CHANNEL_3

/**
 * @brief PWM channel for front-right motor.
 */
#define TIM_PWM_FR TIM_CHANNEL_4


/**
 * @brief Wheel radius in meters.
 */
#define WHEEL_RADIUS_M 								(0.00)


/**
 * @brief Conversion factor from RPM to radians per second.
 */
#define RPM_TO_RAD_PER_SEC    						(2.0 * M_PI / 60.0)

/**
 * @brief Period of CAN message transmission in milliseconds.
 */
#define CAN_TX_PERIOD_MS                          	(5U)

/**
 * @brief Period of MPU CAN message transmission in milliseconds.
 */
#define MPU_CAN_TX_PERIOD_MS                          	(7U)
/**
 * @brief Size of the CAN message queue.
 */
#define CAN_QUEUE_SIZE  16

/**
 * @brief Operation completed successfully.
 */
#define ROVER_OK      ((Rover_StatusTypeDef)0U)

/**
 * @brief Generic error status for rover operations.
 */
#define ROVER_ERROR   ((Rover_StatusTypeDef)1U)

/**
 * @brief Operation completed successfully for motor functions.
 */
#define MOTOR_OK 	  ((Motor_StatusTypeDef)0U)

/**
 * @brief Generic error status for motor operations.
 */
#define MOTOR_ERROR   ((Motor_StatusTypeDef)1U)

#define MAX_RPM 167

#define MIN_RPM -167
/**
 * @brief Structure representing the full state and configuration of the rover.
 *
 * This structure aggregates all hardware components (encoders, IMU, CAN bus, motor timers),
 * velocity state, and FreeRTOS message queues related to the rover's firmware.
 */
typedef struct
{
    encoder_t encoder_fl;	/**< Encoder for front-left wheel */
    encoder_t encoder_rl;	/**< Encoder for rear-left wheel */
    encoder_t encoder_fr;	/**< Encoder for front-right wheel */
    encoder_t encoder_rr;	/**< Encoder for rear-right wheel */

    encoder_config_t encoder1_config;	/**< Configuration for encoder 1 */
    encoder_config_t encoder2_config;	/**< Configuration for encoder 2 */
    encoder_config_t encoder3_config;	/**< Configuration for encoder 3 */
    encoder_config_t encoder4_config;	/**< Configuration for encoder 4 */

    pid_t pid_fl;  /**< PID regulator for front-left motor */
    pid_t pid_rl;  /**< PID regulator for rear-left motor */
    pid_t pid_fr;  /**< PID regulator for front-right motor */
    pid_t pid_rr;  /**< PID regulator for rear-right motor */

    int16_t reference_fl_rpm; /**< Reference RPM for front-left motor */
    int16_t reference_rl_rpm; /**< Reference RPM for rear-left motor */
    int16_t reference_fr_rpm; /**< Reference RPM for front-right motor */
    int16_t reference_rr_rpm; /**< Reference RPM for rear-right motor */

    TIM_HandleTypeDef *motor_timer;	/**< Timer used for motor PWM control */

    MPU60X0_t mpu;	/**< IMU sensor structure */
    MPU60X0_config_t mpu_config;	/**< IMU sensor configuration */

    canManager_t can_manager;	/**< CAN manager structure */
    canManager_config_t can_config;	/**< CAN manager configuration */

    float vx;	/**< Linear velocity in X direction (m/s) */
    float vy;	/**< Linear velocity in Y direction (m/s) */

    osMessageQueueId_t canMsgQueueHandle;	/**< Message queue handle for CAN messages */
    uint8_t canMsgQueueBuffer[CAN_QUEUE_SIZE * sizeof(can_msg_t)];	/**< Buffer for CAN message queue */
    osStaticMessageQDef_t canMsgQueueControlBlock;	/**< Control block for the static message queue */
    can_sender_t can_sender;	/**< CAN sender utility */
    osMessageQueueAttr_t canMsgQueue_attributes;	/**< Attributes of the CAN message queue */


} rover_t;

/**
 * @brief Returns the singleton instance of the rover structure.
 *
 * @return rover_t* Constant pointer to the rover instance.
 */
rover_t* const rover_get_instance(void);

/**
 * @brief Initializes the rover components (encoders, IMU, CAN, etc.).
 *
 * @return ROVER_OK if initialization is successful, otherwise ROVER_ERROR.
 */
Rover_StatusTypeDef rover_init(void);

/**
 * @brief Starts all PWM channels used for motor control.
 *
 * @return HAL_OK if all PWM channels start successfully, otherwise HAL_ERROR.
 */
HAL_StatusTypeDef Start_PWM_Channels(void);


/**
 * @brief Stops all motors by setting the control value to zero.
 *
 * @return MOTOR_OK if the operation is successful, otherwise MOTOR_ERROR.
 */
Motor_StatusTypeDef stop_all_motors(void);

/**
 * @brief Updates all motor encoder speeds.
 *
 * This function reads the actual speed in RPM from each of the four motor encoders
 * and updates their internal values.
 *
 * @return MOTOR_OK if all encoders update successfully, otherwise MOTOR_ERROR.
 */
Encoder_StatusTypeDef motor_control_step(void);

/**
 * @brief Sets the PWM duty cycle for a motor based on a physical control input.
 *
 * This function computes the appropriate compare value for a timer PWM channel
 * to represent a desired physical input as a voltage signal.
 * The mapping is defined by the following linear conversion:
 *
 * @code
 *     compare_value = ((CONVERSION_FACTOR * desiredValue + STOP_PWM_TENSION) * timer->Init.Period) / MAX_PWM_TENSION;
 * @endcode
 *
 * Where:
 * - `desiredValue` is the command in physical units.
 * - `CONVERSION_FACTOR` maps the physical input to a voltage range.
 * - `STOP_PWM_TENSION` defines the voltage corresponding to a stop state.
 * - `MAX_PWM_TENSION` represents the maximum output voltage.
 * - `timer->Init.Period` is the PWM resolution of the hardware timer.
 *
 * The resulting compare value is set into the timer’s compare register to generate the PWM signal.
 *
 * @param timer Pointer to the TIM handle configured for PWM.
 * @param channel Timer PWM channel to be updated.
 * @param desiredValue Desired control value to apply to the motor.
 *
 */
void drive_motor(TIM_HandleTypeDef* timer,HAL_TIM_ActiveChannel channel, double desiredValue);

/**
 * @brief Executes one control cycle of the rover's motor PID regulators.
 *
 * This function performs the following steps:
 * - Computes the error between reference and actual RPM for each motor.
 * - Uses PID controllers to calculate control efforts (`u_*`) for each motor.
 * - Sends the computed control efforts to the motors via `drive_motor`, applying PWM signals.
 *
 * The PID controllers are:
 * - `pid_ant_sx`: front-left motor
 * - `pid_ant_dx`: front-right motor
 * - `pid_pos_sx`: rear-left motor
 * - `pid_pos_dx`: rear-right motor
 *
 * @note The `rover` structure must contain updated encoder feedback and reference values.
 */
void rover_pid_control(void);

/**
 * @brief Clamps an RPM value to the allowed range.
 *
 * This function ensures that the given RPM value stays within the limits defined
 * by `MAX_RPM` and `MIN_RPM`. If the input exceeds these limits, it will be saturated
 * to the closest bound.
 *
 * @param value The RPM value to be saturated.
 * @return The saturated RPM value, constrained within [MIN_RPM, MAX_RPM].
 */
int16_t saturate_rpm(int16_t value);


/**
 * @brief Sends encoder-based velocity data over CAN bus.
 *
 * Updates motor encoder speeds, calculates (vx, vy), formats a CAN message,
 * and enqueues it for transmission.
 *
 * @return ROVER_OK if the message is successfully queued for transmission, otherwise ROVER_ERROR.
 */
Rover_StatusTypeDef rover_enc_can_tx_step(void);

/**
 * @brief Reads the IMU sensor and refreshes the IMU data area.
 *
 * Enabled by USE_MPU_SENSOR. Reads the current gyroscope and accelerometer values
 * from the MPU60X0 and stores them into rover.mpu.gyro / rover.mpu.accel, making them
 * available to any consumer (e.g. rover_imu_can_tx_step) independently of the CAN
 * transmission.
 *
 * If USE_MPU_SENSOR is undefined the function is a no-op and returns ROVER_OK.
 *
 * @return ROVER_OK if the sensor was read successfully, otherwise ROVER_ERROR.
 */
Rover_StatusTypeDef rover_imu_read_step(void);

/**
 * @brief Sends IMU sensor data over CAN bus.
 *
 * Enabled by ENABLE_MPU_CAN_TX. Formats the values currently stored in
 * rover.mpu.gyro / rover.mpu.accel into three separate CAN frames (gyro XY, accel XY,
 * and Z data) and queues them for transmission. This step does not read the sensor:
 * refreshing the data area is the responsibility of rover_imu_read_step(). If the
 * sensor is disabled, the last stored values are transmitted as-is, which allows
 * testing the full CAN communication without a physical MPU6050.
 *
 * If ENABLE_MPU_CAN_TX is undefined the function is a no-op and returns ROVER_OK.
 *
 * @return ROVER_OK if all messages are successfully queued, otherwise ROVER_ERROR.
 */
Rover_StatusTypeDef rover_imu_can_tx_step(void);


/**
 * @brief Sends a CAN message from the rover's transmission queue.
 *
 * Dequeues one message from the CAN sender's internal buffer and sends it over the CAN bus.
 *
 * @return ROVER_OK if a message was transmitted successfully, otherwise ROVER_ERROR.
 */
Rover_StatusTypeDef rover_can_tx_step(void);

#endif /* INC_ROVER_FIRMWARE_H_ */
