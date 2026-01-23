/**
 * @file can_messages.h
 * @brief CAN message packing/unpacking for rover signals (Reference, encoder RPMs and IMU), DBC-aligned.
 *
 * This module provides helpers aligned to the following DBC messages:
 * - BO_ 10 Reference     (DLC=8) : 4 x int16 (rpm)    [RX on Rover]
 * - BO_ 11 EncoderRpms   (DLC=8) : 4 x int16 (raw)    [TX from Rover]
 *   Physical RPM = raw_value * (1/182).
 * - BO_ 12 ImuGyroXy     (DLC=8) : 2 x 32-bit         [TX from Rover]
 * - BO_ 13 ImuAccelXy    (DLC=8) : 2 x 32-bit         [TX from Rover]
 * - BO_ 14 ImuZ          (DLC=8) : 2 x 32-bit         [TX from Rover]
 *
 * Endianness:
 * The DBC uses '@1' for all signals, i.e., little-endian (Intel) bit/byte ordering.
 *
 * IMU 32-bit representation policy:
 * The DBC specifies 32-bit signed signals with scaling (1,0) but does not
 * explicitly state whether these are IEEE-754 float32 bit patterns or int32
 * physical integers. This module treats the 32-bit representation as
 * implementation-defined and documents it accordingly. The source implementation
 * shall match the receiver expectations.
 *
 * @note
 * This header defines the public API only. The packing logic is implemented in
 * can_messages.c (or equivalent).
 *
 * @author Alessio Guarini, Miriam Vitolo
 * @date June 13, 2025
 */

#ifndef CAN_MESSAGES_H_
#define CAN_MESSAGES_H_

#include <stdint.h>
#include <stdlib.h>
#include "geometry.h"

/*==============================================================================
 * Public status type (the only PascalCase exception)
 *============================================================================*/

/**
 * @brief Status type for CAN message operations.
 *
 * @details
 * A single common status type is used across the entire module, to keep the API
 * uniform and avoid accidental mismatches between message families.
 */
typedef uint8_t CANMessages_StatusTypeDef;

/** @brief Operation completed successfully. */
#define CAN_MESSAGES_OK   ((CANMessages_StatusTypeDef)0U)

/** @brief Operation failed (e.g., NULL pointer, invalid argument). */
#define CAN_MESSAGES_ERR  ((CANMessages_StatusTypeDef)1U)

/*==============================================================================
 * CAN Identifiers (DBC aligned)
 *============================================================================*/

/**
 * @name CAN message identifiers (standard 11-bit)
 * @brief Message identifiers as defined in the DBC.
 * @{
 */
#define CAN_MESSAGES_REFERENCE_MSG_ID        (10U)  /**< BO_ 10 Reference   */
#define CAN_MESSAGES_ENCODER_RPMS_MSG_ID     (11U)  /**< BO_ 11 EncoderRpms */
#define CAN_MESSAGES_IMU_GYRO_XY_MSG_ID      (12U)  /**< BO_ 12 ImuGyroXy   */
#define CAN_MESSAGES_IMU_ACCEL_XY_MSG_ID     (13U)  /**< BO_ 13 ImuAccelXy  */
#define CAN_MESSAGES_IMU_Z_MSG_ID            (14U)  /**< BO_ 14 ImuZ        */
/** @} */

/*==============================================================================
 * Frame lengths (DLC)
 *============================================================================*/

/**
 * @name Frame payload lengths (DLC)
 * @brief Payload sizes for the CAN messages handled by this module.
 * @{
 */
#define CAN_MESSAGES_REFERENCE_DLC_BYTES       (8U)  /**< BO_ 10 DLC=8  */
#define CAN_MESSAGES_ENCODER_RPMS_DLC_BYTES    (8U)  /**< BO_ 11 DLC=8  */
#define CAN_MESSAGES_IMU_DLC_BYTES             (8U)  /**< BO_ 12..14 DLC=8 */
/** @} */

/*==============================================================================
 * Encoder scaling (DBC aligned)
 *============================================================================*/

/**
 * @name Encoder RPM scaling (BO_11)
 * @brief DBC scaling for BO_ 11 EncoderRpms.
 *
 * DBC comment: "Raw encoder RPMs. Physical RPM = raw_value * (1/182)."
 * Hence: raw_value = physical_rpm * 182.
 * @{
 */
#define CAN_MESSAGES_ENCODER_RPM_RAW_SCALE     (182.0)   /**< raw = physical * 182 */
#define CAN_MESSAGES_ENCODER_RPM_PHYS_MIN      (-180.0)  /**< Physical RPM min (DBC) */
#define CAN_MESSAGES_ENCODER_RPM_PHYS_MAX      (180.0)   /**< Physical RPM max (DBC) */
/** @} */

/*==============================================================================
 * Signal layout descriptors (start bit / length / endianness) - DBC aligned
 *============================================================================*/

/**
 * @name BO_ 10 Reference signal layout (4 x int16, signed)
 * @brief Reference RPMs layout (FrontLeft, RearLeft, FrontRight, RearRight).
 * @{
 */
#define CAN_MESSAGES_REF_FL_START_BIT          (0U)
#define CAN_MESSAGES_REF_FL_LENGTH             (16U)
#define CAN_MESSAGES_REF_FL_ENDIANNESS         (1U)

#define CAN_MESSAGES_REF_RL_START_BIT          (16U)
#define CAN_MESSAGES_REF_RL_LENGTH             (16U)
#define CAN_MESSAGES_REF_RL_ENDIANNESS         (1U)

#define CAN_MESSAGES_REF_FR_START_BIT          (32U)
#define CAN_MESSAGES_REF_FR_LENGTH             (16U)
#define CAN_MESSAGES_REF_FR_ENDIANNESS         (1U)

#define CAN_MESSAGES_REF_RR_START_BIT          (48U)
#define CAN_MESSAGES_REF_RR_LENGTH             (16U)
#define CAN_MESSAGES_REF_RR_ENDIANNESS         (1U)
/** @} */

/**
 * @name BO_ 11 EncoderRpms signal layout (4 x int16, signed, raw)
 * @{
 */
#define CAN_MESSAGES_ENC_FL_START_BIT          (0U)
#define CAN_MESSAGES_ENC_FL_LENGTH             (16U)
#define CAN_MESSAGES_ENC_FL_ENDIANNESS         (1U)

#define CAN_MESSAGES_ENC_RL_START_BIT          (16U)
#define CAN_MESSAGES_ENC_RL_LENGTH             (16U)
#define CAN_MESSAGES_ENC_RL_ENDIANNESS         (1U)

#define CAN_MESSAGES_ENC_FR_START_BIT          (32U)
#define CAN_MESSAGES_ENC_FR_LENGTH             (16U)
#define CAN_MESSAGES_ENC_FR_ENDIANNESS         (1U)

#define CAN_MESSAGES_ENC_RR_START_BIT          (48U)
#define CAN_MESSAGES_ENC_RR_LENGTH             (16U)
#define CAN_MESSAGES_ENC_RR_ENDIANNESS         (1U)
/** @} */

/**
 * @name BO_ 12 ImuGyroXy signal layout (2 x 32-bit)
 * @{
 */
#define CAN_MESSAGES_GYRO_X_START_BIT          (0U)
#define CAN_MESSAGES_GYRO_X_LENGTH             (32U)
#define CAN_MESSAGES_GYRO_X_ENDIANNESS         (1U)

#define CAN_MESSAGES_GYRO_Y_START_BIT          (32U)
#define CAN_MESSAGES_GYRO_Y_LENGTH             (32U)
#define CAN_MESSAGES_GYRO_Y_ENDIANNESS         (1U)
/** @} */

/**
 * @name BO_ 13 ImuAccelXy signal layout (2 x 32-bit)
 * @{
 */
#define CAN_MESSAGES_ACCEL_X_START_BIT         (0U)
#define CAN_MESSAGES_ACCEL_X_LENGTH            (32U)
#define CAN_MESSAGES_ACCEL_X_ENDIANNESS        (1U)

#define CAN_MESSAGES_ACCEL_Y_START_BIT         (32U)
#define CAN_MESSAGES_ACCEL_Y_LENGTH            (32U)
#define CAN_MESSAGES_ACCEL_Y_ENDIANNESS        (1U)
/** @} */

/**
 * @name BO_ 14 ImuZ signal layout (2 x 32-bit)
 * @{
 */
#define CAN_MESSAGES_GYRO_Z_START_BIT          (0U)
#define CAN_MESSAGES_GYRO_Z_LENGTH             (32U)
#define CAN_MESSAGES_GYRO_Z_ENDIANNESS         (1U)

#define CAN_MESSAGES_ACCEL_Z_START_BIT         (32U)
#define CAN_MESSAGES_ACCEL_Z_LENGTH            (32U)
#define CAN_MESSAGES_ACCEL_Z_ENDIANNESS        (1U)
/** @} */

/*==============================================================================
 * Data containers (snake_case)
 *============================================================================*/

/**
 * @brief BO_ 10 Reference container (RX on Rover).
 *
 * @details
 * Represents BO_ 10 "Reference" signals as defined by the DBC:
 * FrontLeft, RearLeft, FrontRight, RearRight as signed 16-bit RPM values.
 *
 * @note DBC DLC is 8 bytes.
 */
typedef struct
{
    int16_t front_left_rpm;   /**< SG_ FrontLeft  : 0|16  signed, rpm */
    int16_t rear_left_rpm;    /**< SG_ RearLeft   : 16|16 signed, rpm */
    int16_t front_right_rpm;  /**< SG_ FrontRight : 32|16 signed, rpm */
    int16_t rear_right_rpm;   /**< SG_ RearRight  : 48|16 signed, rpm */
} can_messages_reference_t;

/**
 * @brief BO_ 11 EncoderRpms container (TX from Rover).
 *
 * @details
 * Stores raw encoder RPM values as transported on CAN (signed 16-bit).
 * Physical RPM is computed as:
 * - physical_rpm = raw_value / 182
 *
 * For TX generation from physical RPM:
 * - raw_value = physical_rpm * 182 (with rounding and saturation).
 *
 * @note Signal order matches the DBC layout (FrontLeft, RearLeft, FrontRight, RearRight).
 */
typedef struct
{
    int16_t front_left_raw;   /**< SG_ FrontLeft  : 0|16  signed raw */
    int16_t rear_left_raw;    /**< SG_ RearLeft   : 16|16 signed raw */
    int16_t front_right_raw;  /**< SG_ FrontRight : 32|16 signed raw */
    int16_t rear_right_raw;   /**< SG_ RearRight  : 48|16 signed raw */
} can_messages_encoder_rpms_t;

/*==============================================================================
 * Public API (snake_case)
 *============================================================================*/

/**
 * @brief Initialize a can_messages_encoder_rpms_t instance.
 *
 * @details Sets all fields to zero and validates the input pointer.
 *
 * @param[in,out] enc Pointer to the encoder container to initialize.
 * @retval CAN_MESSAGES_OK  Initialization completed successfully.
 * @retval CAN_MESSAGES_ERR The input pointer is NULL.
 */
CANMessages_StatusTypeDef can_msg_encoder_rpms_init(can_messages_encoder_rpms_t * enc);

/**
 * @brief Set encoder RPMs from physical RPM values (BO_ 11).
 *
 * @details
 * Converts physical RPM to raw values using:
 * raw = physical * 182
 * and applies deterministic rounding and saturation to int16_t.
 *
 * The physical input is expected to be within the DBC range
 * [CAN_MESSAGES_ENCODER_RPM_PHYS_MIN, CAN_MESSAGES_ENCODER_RPM_PHYS_MAX].
 * Values outside the range are saturated.
 *
 * @param[in,out] enc Pointer to encoder container to update.
 * @param[in] fl_rpm  FrontLeft physical RPM.
 * @param[in] rl_rpm  RearLeft physical RPM.
 * @param[in] fr_rpm  FrontRight physical RPM.
 * @param[in] rr_rpm  RearRight physical RPM.
 * @retval CAN_MESSAGES_OK  Values converted and stored successfully.
 * @retval CAN_MESSAGES_ERR The input pointer is NULL.
 */
CANMessages_StatusTypeDef can_msg_encoder_rpms_set_phys(can_messages_encoder_rpms_t * enc,
                                                       double fl_rpm,
                                                       double rl_rpm,
                                                       double fr_rpm,
                                                       double rr_rpm);

/**
 * @brief Pack BO_ 11 EncoderRpms CAN payload (DLC=8), TX from Rover.
 *
 * @details
 * Packs the four int16 raw encoder values into the output payload buffer
 * according to the DBC bit layout (little-endian, signed).
 *
 * @param[in] enc Pointer to the encoder container.
 * @param[out] can_data Pointer to an output buffer of at least
 *                      CAN_MESSAGES_ENCODER_RPMS_DLC_BYTES bytes.
 * @retval CAN_MESSAGES_OK  Payload packed successfully.
 * @retval CAN_MESSAGES_ERR At least one input pointer is NULL.
 */
CANMessages_StatusTypeDef can_msg_pack_encoder_rpms_frame(const can_messages_encoder_rpms_t * enc,
                                                         uint8_t * can_data);

/**
 * @brief Unpack BO_ 10 Reference CAN payload (DLC=8), RX on Rover.
 *
 * @details
 * Decodes the four int16 reference RPM values from the input payload buffer
 * according to the DBC bit layout (little-endian, signed).
 *
 * @param[in] can_data Pointer to the input buffer containing the BO_10 payload.
 * @param[out] ref Pointer to the output reference container to be filled.
 * @retval CAN_MESSAGES_OK  Payload unpacked successfully.
 * @retval CAN_MESSAGES_ERR At least one input pointer is NULL.
 *
 * @warning
 * The caller must provide a valid input buffer of at least
 * CAN_MESSAGES_REFERENCE_DLC_BYTES bytes.
 */
CANMessages_StatusTypeDef can_msg_unpack_reference_frame(const uint8_t * can_data,
                                                        can_messages_reference_t * ref);

/**
 * @brief Pack BO_ 12 ImuGyroXy CAN payload (DLC=8), TX from Rover.
 *
 * @details
 * Packs GyroX and GyroY into the payload according to the DBC layout.
 * The 32-bit representation is implementation-defined (e.g., IEEE-754 float32
 * serialized as little-endian, or int32 physical).
 *
 * @param[in] gyro_data Pointer to Cartesian3D containing gyroscope data (X and Y used).
 * @param[out] can_data Pointer to an output buffer of at least CAN_MESSAGES_IMU_DLC_BYTES bytes.
 * @retval CAN_MESSAGES_OK  Payload packed successfully.
 * @retval CAN_MESSAGES_ERR At least one input pointer is NULL.
 */
CANMessages_StatusTypeDef can_msg_imu_pack_gyro_xy_frame(const Cartesian3D * gyro_data,
                                                        uint8_t * can_data);

/**
 * @brief Pack BO_ 13 ImuAccelXy CAN payload (DLC=8), TX from Rover.
 *
 * @details
 * Packs AccelX and AccelY into the payload according to the DBC layout.
 * The 32-bit representation is implementation-defined (e.g., IEEE-754 float32
 * serialized as little-endian, or int32 physical).
 *
 * @param[in] accel_data Pointer to Cartesian3D containing accelerometer data (X and Y used).
 * @param[out] can_data Pointer to an output buffer of at least CAN_MESSAGES_IMU_DLC_BYTES bytes.
 * @retval CAN_MESSAGES_OK  Payload packed successfully.
 * @retval CAN_MESSAGES_ERR At least one input pointer is NULL.
 */
CANMessages_StatusTypeDef can_msg_imu_pack_accel_xy_frame(const Cartesian3D * accel_data,
                                                         uint8_t * can_data);

/**
 * @brief Pack BO_ 14 ImuZ CAN payload (DLC=8), TX from Rover.
 *
 * @details
 * Packs GyroZ and AccelZ into the payload according to the DBC layout.
 * The 32-bit representation is implementation-defined (e.g., IEEE-754 float32
 * serialized as little-endian, or int32 physical).
 *
 * @param[in] gyro_data Pointer to Cartesian3D containing gyroscope data (Z used).
 * @param[in] accel_data Pointer to Cartesian3D containing accelerometer data (Z used).
 * @param[out] can_data Pointer to an output buffer of at least CAN_MESSAGES_IMU_DLC_BYTES bytes.
 * @retval CAN_MESSAGES_OK  Payload packed successfully.
 * @retval CAN_MESSAGES_ERR At least one input pointer is NULL.
 */
CANMessages_StatusTypeDef can_msg_imu_pack_z_frame(const Cartesian3D * gyro_data,
                                                  const Cartesian3D * accel_data,
                                                  uint8_t * can_data);

#endif /* CAN_MESSAGES_H_ */
