/*
 * rover_firmware.c
 *
 *  Created on: Jun 11, 2025
 *      Author: Miriam Vitolo
 */
#include "cmsis_os.h"
#include "encoder.h"
#include "rover_firmware.h"
#include "test_comunication.h"
#include "can_sender.h"
#include "i2c.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

/**
 * @brief Internal singleton instance of the rover.
 *
 * This structure is initialized once and provides centralized access to all
 * hardware drivers and runtime state related to the rover system.
 *
 * - Includes configuration for 4 encoders, MPU sensor, CAN manager, and PWM timer.
 * - Used internally via rover_get_instance().
 */
static rover_t rover = {
    .encoder1_config = {
        .cpr = ENCODER_CPR,
        .gear_ratio = ENCODER_GEAR_RATIO,
        .htim = &htim1,
        .n_channels = ENCODER_N_CHANNELS,
        .sampling_time = ENCODER_SAMPLING_TIME
    },
    .encoder2_config = {
        .cpr = ENCODER_CPR,
        .gear_ratio = ENCODER_GEAR_RATIO,
        .htim = &htim2,
        .n_channels = ENCODER_N_CHANNELS,
        .sampling_time = ENCODER_SAMPLING_TIME
    },
    .encoder3_config = {
        .cpr = ENCODER_CPR,
        .gear_ratio = ENCODER_GEAR_RATIO,
        .htim = &htim3,
        .n_channels = ENCODER_N_CHANNELS,
        .sampling_time = ENCODER_SAMPLING_TIME
    },
    .encoder4_config = {
        .cpr = ENCODER_CPR,
        .gear_ratio = ENCODER_GEAR_RATIO,
        .htim = &htim4,
        .n_channels = ENCODER_N_CHANNELS,
        .sampling_time = ENCODER_SAMPLING_TIME
    },
    .motor_timer = &htim8,
	.mpu_config = {
	    .hi2c = &hi2c2
	},
	.mpu = {
	    .accel = {0},
	    .gyro = {0}
	},
	.can_config = {
		 .hcan = &hcan1,
		 .tx_header = {
		 .IDE = CAN_ID_STD,
		 .RTR = CAN_RTR_DATA,
		 .DLC = 8,
		 .TransmitGlobalTime = DISABLE
		  },
		  .rx_fifo = CAN_RX_FIFO0,
		  .rx_interrupt = CAN_IT_RX_FIFO0_MSG_PENDING
	},
	.can_manager = {
	.config = &rover.can_config
	},
	.canMsgQueueHandle = NULL,
	.canMsgQueueBuffer = {0},
	.canMsgQueueControlBlock = {{0}},
	.can_sender = {
		 .can_manager = &rover.can_manager,
		 .xQueue = NULL,
		 .can_msg_buff = {
		 .msg = {0},
		 .id = 0
		 }
	},
	.canMsgQueue_attributes = {
		.name = "canMsgQueue",
		.cb_mem = &rover.canMsgQueueControlBlock,
		.cb_size = sizeof(rover.canMsgQueueControlBlock),
		.mq_mem = &rover.canMsgQueueBuffer,
		.mq_size = sizeof(rover.canMsgQueueBuffer)
	}


};

/**
 * @brief Singleton instance of the rover structure.
 */
static rover_t * const instance = &rover;

rover_t* const rover_get_instance() {
    return instance;
}


static inline Rover_StatusTypeDef __imu_init(){
	Rover_StatusTypeDef status = ROVER_OK;
	#ifdef USE_MPU
	status = ROVER_ERROR;
	if(MPU60X0_init(&rover.mpu, &rover.mpu_config, HAL_MAX_DELAY) == MPU60X0_OK){
		status = ROVER_OK;
	}
	#endif // USE_MPU
	return status;
}

Rover_StatusTypeDef rover_init(void){
	rover.canMsgQueueHandle = osMessageQueueNew(CAN_QUEUE_SIZE, sizeof(can_msg_t), &rover.canMsgQueue_attributes);
	rover.can_sender.xQueue = rover.canMsgQueueHandle;
	Rover_StatusTypeDef status = ROVER_ERROR;
	if ((encoder_init(&rover.encoder_fl, &rover.encoder1_config) == ENCODER_OK) &&
		(encoder_init(&rover.encoder_rl, &rover.encoder2_config) == ENCODER_OK) &&
		(encoder_init(&rover.encoder_fr, &rover.encoder3_config) == ENCODER_OK) &&
		(encoder_init(&rover.encoder_rr, &rover.encoder4_config) == ENCODER_OK) &&
		(pid_init(&rover.pid_fl, PID_ANT_SX_KP_FAST, PID_ANT_SX_KI_FAST, PID_ANT_SX_KD_FAST, UK_MIN, UK_MAX) == PID_OK) &&
		(pid_init(&rover.pid_rl, PID_POS_SX_KP_FAST, PID_POS_SX_KI_FAST, PID_POS_SX_KD_FAST, UK_MIN, UK_MAX) == PID_OK) &&
		(pid_init(&rover.pid_fr, PID_ANT_DX_KP_FAST, PID_ANT_DX_KI_FAST, PID_ANT_DX_KD_FAST, UK_MIN, UK_MAX) == PID_OK) &&
		(pid_init(&rover.pid_rr, PID_POS_DX_KP_FAST, PID_POS_DX_KI_FAST, PID_POS_DX_KD_FAST, UK_MIN, UK_MAX) == PID_OK) &&
		(__imu_init() == ROVER_OK) &&
		(Start_PWM_Channels() == HAL_OK) &&
		(stop_all_motors() == MOTOR_OK) &&
		(canManager_Init(&rover.can_manager) == CAN_MANAGER_OK) &&
		(canManager_AddAllowedId(&rover.can_manager, CAN_MESSAGES_REFERENCE_MSG_ID) == CAN_MANAGER_OK) &&
		(canManager_AddAllowedId(&rover.can_manager, CAN_MESSAGES_ENCODER_RPMS_MSG_ID) == CAN_MANAGER_OK) &&
		(canManager_AddAllowedId(&rover.can_manager, CAN_MESSAGES_IMU_GYRO_XY_MSG_ID) == CAN_MANAGER_OK) &&
		(canManager_AddAllowedId(&rover.can_manager, CAN_MESSAGES_IMU_ACCEL_XY_MSG_ID) == CAN_MANAGER_OK) &&
		(canManager_AddAllowedId(&rover.can_manager, CAN_MESSAGES_IMU_Z_MSG_ID) == CAN_MANAGER_OK) &&
		(can_sender_init(&rover.can_sender, &rover.can_manager, rover.canMsgQueueHandle) == CAN_SENDER_OK))
	{
		HAL_GPIO_WritePin(GPIOB, RELE_Pin|GPIO_PIN_13, GPIO_PIN_SET);
		status = ROVER_OK;
	}
	return status;
}

HAL_StatusTypeDef Start_PWM_Channels(void){
	HAL_StatusTypeDef status = HAL_ERROR;

	if ((rover.motor_timer != NULL) &&(HAL_TIM_PWM_Start(rover.motor_timer, TIM_PWM_RL) == HAL_OK) &&
		(HAL_TIM_PWM_Start(rover.motor_timer, TIM_PWM_RR) == HAL_OK) &&
		(HAL_TIM_PWM_Start(rover.motor_timer, TIM_PWM_FL) == HAL_OK) &&
		(HAL_TIM_PWM_Start(rover.motor_timer, TIM_PWM_FR) == HAL_OK))
	{
    status = HAL_OK;
	}
	return status;
}

Motor_StatusTypeDef stop_all_motors(void){
	Motor_StatusTypeDef status = MOTOR_ERROR;

	if (rover.motor_timer != NULL)
	{
		drive_motor(rover.motor_timer, TIM_PWM_RL, 0);
		drive_motor(rover.motor_timer, TIM_PWM_RR, 0);
		drive_motor(rover.motor_timer, TIM_PWM_FL, 0);
		drive_motor(rover.motor_timer, TIM_PWM_FR, 0);
		HAL_GPIO_WritePin(GPIOB, RELE_Pin|GPIO_PIN_13, GPIO_PIN_RESET);

		status = MOTOR_OK;
	}
	return status;
}

Motor_StatusTypeDef motor_control_step(void){
	Motor_StatusTypeDef status = MOTOR_ERROR;

    if ((encoder_update_speed(&rover.encoder_fl) == ENCODER_OK) &&
        (encoder_update_speed(&rover.encoder_rl) == ENCODER_OK) &&
        (encoder_update_speed(&rover.encoder_fr) == ENCODER_OK) &&
        (encoder_update_speed(&rover.encoder_rr) == ENCODER_OK))
    {
    	rover_pid_control();
        status = MOTOR_OK;
    }

    return status;
}

void drive_motor(TIM_HandleTypeDef* timer,HAL_TIM_ActiveChannel channel, double desiredValue){
	uint32_t compare_value;
	if(timer != NULL){
		compare_value = (uint32_t)(((CONVERSION_FACTOR*desiredValue + STOP_PWM_TENSION)*((float)timer->Init.Period))/MAX_PWM_TENSION);
		__HAL_TIM_SET_COMPARE(timer,channel,compare_value);
	}
}

void rover_pid_control(void)
{

    double u_fl = 0, u_fr = 0, u_rl = 0, u_rr = 0;

    double e_fl = rover.reference_fl_rpm - rover.encoder_fl.actual_speed_rpm;
    double e_rl = rover.reference_rl_rpm - rover.encoder_rl.actual_speed_rpm;
    double e_fr = rover.reference_fr_rpm - rover.encoder_fr.actual_speed_rpm;
    double e_rr = rover.reference_rr_rpm - rover.encoder_rr.actual_speed_rpm;

    pid_calculate_output(&rover.pid_fl, e_fl, &u_fl);
    pid_calculate_output(&rover.pid_fr, e_fr, &u_fr);
    pid_calculate_output(&rover.pid_rl, e_rl, &u_rl);
    pid_calculate_output(&rover.pid_rr, e_rr, &u_rr);

    drive_motor(rover.motor_timer, TIM_PWM_RL, u_rl);
    drive_motor(rover.motor_timer, TIM_PWM_FL, u_fl);
    drive_motor(rover.motor_timer, TIM_PWM_RR, u_rr);
    drive_motor(rover.motor_timer, TIM_PWM_FR, u_fr);
}


int16_t saturate_rpm(int16_t value)
{

    if (value > MAX_RPM) return MAX_RPM;
    if (value < MIN_RPM) return MIN_RPM;
    return value;
}


Rover_StatusTypeDef rover_enc_can_tx_step(void){
	Rover_StatusTypeDef status = ROVER_ERROR;
	can_msg_t message;
	can_messages_encoder_rpms_t enc_msg;
	if (motor_control_step() == MOTOR_OK) {
		#ifdef USE_CAN_TX
		if ((can_msg_encoder_rpms_init(&enc_msg) == CAN_MESSAGES_OK) &&
			(can_msg_encoder_rpms_set_phys(&enc_msg,
			                               rover.encoder_fl.actual_speed_rpm,
			                               rover.encoder_rl.actual_speed_rpm,
			                               rover.encoder_fr.actual_speed_rpm,
			                               rover.encoder_rr.actual_speed_rpm) == CAN_MESSAGES_OK) &&
			(can_msg_pack_encoder_rpms_frame(&enc_msg, message.msg) == CAN_MESSAGES_OK))
		{
			message.id = CAN_MESSAGES_ENCODER_RPMS_MSG_ID;
			if (can_sender_enqueue_msg(&rover.can_sender, &message) == CAN_SENDER_OK){
				status = ROVER_OK;
			}
		}
		#else
			status = ROVER_OK;
		#endif
	}
	return status;
}

Rover_StatusTypeDef rover_imu_can_tx_step(void){
	Rover_StatusTypeDef status = ROVER_OK;
	#ifdef USE_MPU
	status = ROVER_ERROR;
	can_msg_t gyro_msg, accel_msg, z_msg;
	uint8_t gyro_frame[IMU_FRAME_LENGTH_IN_BYTE];
	uint8_t accel_frame[IMU_FRAME_LENGTH_IN_BYTE];
	uint8_t z_frame[IMU_FRAME_LENGTH_IN_BYTE];


	if (
	    MPU60X0_get_gyro_value(&rover.mpu, &rover.mpu.gyro) == MPU60X0_OK &&
	    MPU60X0_get_accel_value(&rover.mpu, &rover.mpu.accel) == MPU60X0_OK &&
	    IMUFeedback_createGyroXYFrame(&rover.mpu.gyro, gyro_frame) == IMU_FEEDBACK_OK &&
	    IMUFeedback_createAccelXYFrame(&rover.mpu.accel, accel_frame) == IMU_FEEDBACK_OK &&
	    IMUFeedback_createZFrame(&rover.mpu.gyro, &rover.mpu.accel, z_frame) == IMU_FEEDBACK_OK
	) {
	    memcpy(gyro_msg.msg, gyro_frame, IMU_FRAME_LENGTH_IN_BYTE);
	    gyro_msg.id = IMU_GYRO_XY_FEEDBACK_MSG_ID;

	    memcpy(accel_msg.msg, accel_frame, IMU_FRAME_LENGTH_IN_BYTE);
	    accel_msg.id = IMU_ACCEL_XY_FEEDBACK_MSG_ID;

	    memcpy(z_msg.msg, z_frame, IMU_FRAME_LENGTH_IN_BYTE);
	    z_msg.id = IMU_Z_FEEDBACK_MSG_ID;

	    if (
	        can_sender_enqueue_msg(&rover.can_sender, &gyro_msg) == CAN_SENDER_OK &&
	        can_sender_enqueue_msg(&rover.can_sender, &accel_msg) == CAN_SENDER_OK &&
	        can_sender_enqueue_msg(&rover.can_sender, &z_msg) == CAN_SENDER_OK
	    ) {
	        status = ROVER_OK;
	    }
	}
	#endif // USE_MPU
	return status;
}

Rover_StatusTypeDef rover_can_tx_step(void){
	Rover_StatusTypeDef status = ROVER_OK;
	#ifdef USE_CAN_TX
	status = ROVER_ERROR;
	if(can_sender_dequeue_msg(&rover.can_sender) == CAN_SENDER_OK){
		status = ROVER_OK;
	}
	#endif
	return status;
}
