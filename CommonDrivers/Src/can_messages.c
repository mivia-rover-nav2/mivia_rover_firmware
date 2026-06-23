/**
 * @file can_messages.c
 * @brief Implementation of CAN message packing/unpacking for rover signals (Reference, encoder RPMs and IMU), DBC-aligned.
 *
 * This module reuses:
 * - CanParser_encode_can_frame() / CanParser_decode_can_frame() for bitfield operations
 * - clamp_double() from common_drivers for deterministic range limiting
 *
 * Authors: Alessio Guarini, Miriam Vitolo
 */

#include "can_messages.h"
#include "canParser.h"
#include "common_drivers.h"

#include <string.h>

/*==============================================================================
 * Local helpers
 *============================================================================*/

/**
 * @brief Deterministic rounding to nearest integer (ties away from zero).
 * @param[in] x Input value.
 * @return Rounded value as int32_t.
 */
static int32_t round_double_to_int32(double x)
{
    double y = x;

    if (y >= 0.0)
    {
        y = y + 0.5;
    }
    else
    {
        y = y - 0.5;
    }

    return (int32_t)y;
}

/**
 * @brief Saturate an int32_t value to the int16_t representable range.
 * @param[in] x Input value.
 * @return Saturated value as int16_t.
 */
static int16_t saturate_int32_to_int16(int32_t x)
{
    int32_t y = x;

    if (y > (int32_t)INT16_MAX)
    {
        y = (int32_t)INT16_MAX;
    }
    else if (y < (int32_t)INT16_MIN)
    {
        y = (int32_t)INT16_MIN;
    }
    else
    {
        /* No action required. */
    }

    return (int16_t)y;
}

/**
 * @brief Wrapper for CanParser encoding with module status mapping.
 */
static CANMessages_StatusTypeDef encode_u32(uint8_t * can_data,
                                           uint32_t value,
                                           uint8_t start_bit,
                                           uint8_t length,
                                           uint8_t endianness)
{
    CANMessages_StatusTypeDef status = CAN_MESSAGES_ERR;

    if (CanParser_encode_can_frame(can_data, value, start_bit, length, endianness) != CAN_PARSER_STATUS_ERR)
    {
        status = CAN_MESSAGES_OK;
    }
    else
    {
        /* Keep ERR. */
    }

    return status;
}

/**
 * @brief Wrapper for CanParser decoding with module status mapping.
 */
static CANMessages_StatusTypeDef decode_signal(const uint8_t * can_data,
                                              void * result,
                                              uint8_t start_bit,
                                              uint8_t length,
                                              uint8_t endianness,
                                              uint8_t result_size)
{
    CANMessages_StatusTypeDef status = CAN_MESSAGES_ERR;

    if (CanParser_decode_can_frame(can_data, result, start_bit, length, endianness, result_size) != CAN_PARSER_STATUS_ERR)
    {
        status = CAN_MESSAGES_OK;
    }
    else
    {
        /* Keep ERR. */
    }

    return status;
}

/*==============================================================================
 * Public API
 *============================================================================*/

CANMessages_StatusTypeDef can_msg_encoder_rpms_init(can_messages_encoder_rpms_t * enc)
{
    CANMessages_StatusTypeDef status = CAN_MESSAGES_ERR;

    if (enc != NULL)
    {
        enc->front_left_raw  = 0;
        enc->rear_left_raw   = 0;
        enc->front_right_raw = 0;
        enc->rear_right_raw  = 0;
        status = CAN_MESSAGES_OK;
    }
    else
    {
        /* Keep ERR. */
    }

    return status;
}

CANMessages_StatusTypeDef can_msg_encoder_rpms_set_phys(can_messages_encoder_rpms_t * enc,
                                                       double fl_rpm,
                                                       double rl_rpm,
                                                       double fr_rpm,
                                                       double rr_rpm)
{
    CANMessages_StatusTypeDef status = CAN_MESSAGES_ERR;

    if (enc != NULL)
    {
        /* Clamp physical RPM to DBC range (double). */
        const double fl_c = clamp_double(fl_rpm, CAN_MESSAGES_ENCODER_RPM_PHYS_MIN, CAN_MESSAGES_ENCODER_RPM_PHYS_MAX);
        const double rl_c = clamp_double(rl_rpm, CAN_MESSAGES_ENCODER_RPM_PHYS_MIN, CAN_MESSAGES_ENCODER_RPM_PHYS_MAX);
        const double fr_c = clamp_double(fr_rpm, CAN_MESSAGES_ENCODER_RPM_PHYS_MIN, CAN_MESSAGES_ENCODER_RPM_PHYS_MAX);
        const double rr_c = clamp_double(rr_rpm, CAN_MESSAGES_ENCODER_RPM_PHYS_MIN, CAN_MESSAGES_ENCODER_RPM_PHYS_MAX);

        /* raw = physical * 182 (DBC); deterministic rounding. */
        const int32_t fl_raw32 = round_double_to_int32(fl_c * CAN_MESSAGES_ENCODER_RPM_RAW_SCALE);
        const int32_t rl_raw32 = round_double_to_int32(rl_c * CAN_MESSAGES_ENCODER_RPM_RAW_SCALE);
        const int32_t fr_raw32 = round_double_to_int32(fr_c * CAN_MESSAGES_ENCODER_RPM_RAW_SCALE);
        const int32_t rr_raw32 = round_double_to_int32(rr_c * CAN_MESSAGES_ENCODER_RPM_RAW_SCALE);

        /* Defensive saturation to int16_t. */
        enc->front_left_raw  = saturate_int32_to_int16(fl_raw32);
        enc->rear_left_raw   = saturate_int32_to_int16(rl_raw32);
        enc->front_right_raw = saturate_int32_to_int16(fr_raw32);
        enc->rear_right_raw  = saturate_int32_to_int16(rr_raw32);

        status = CAN_MESSAGES_OK;
    }
    else
    {
        /* Keep ERR. */
    }

    return status;
}

CANMessages_StatusTypeDef can_msg_pack_encoder_rpms_frame(const can_messages_encoder_rpms_t * enc,
                                                         uint8_t * can_data)
{
    CANMessages_StatusTypeDef status = CAN_MESSAGES_ERR;

    if ((enc != NULL) && (can_data != NULL))
    {
        (void)memset(can_data, 0, (size_t)CAN_MESSAGES_ENCODER_RPMS_DLC_BYTES);

        /* DBC: signed 16-bit little-endian. Provide two's complement as unsigned for the encoder. */
        const uint32_t fl_u32 = (uint32_t)(uint16_t)enc->front_left_raw;
        const uint32_t rl_u32 = (uint32_t)(uint16_t)enc->rear_left_raw;
        const uint32_t fr_u32 = (uint32_t)(uint16_t)enc->front_right_raw;
        const uint32_t rr_u32 = (uint32_t)(uint16_t)enc->rear_right_raw;

        if ((encode_u32(can_data, fl_u32,
                        (uint8_t)CAN_MESSAGES_ENC_FL_START_BIT,
                        (uint8_t)CAN_MESSAGES_ENC_FL_LENGTH,
                        (uint8_t)CAN_MESSAGES_ENC_FL_ENDIANNESS) == CAN_MESSAGES_OK) &&
            (encode_u32(can_data, rl_u32,
                        (uint8_t)CAN_MESSAGES_ENC_RL_START_BIT,
                        (uint8_t)CAN_MESSAGES_ENC_RL_LENGTH,
                        (uint8_t)CAN_MESSAGES_ENC_RL_ENDIANNESS) == CAN_MESSAGES_OK) &&
            (encode_u32(can_data, fr_u32,
                        (uint8_t)CAN_MESSAGES_ENC_FR_START_BIT,
                        (uint8_t)CAN_MESSAGES_ENC_FR_LENGTH,
                        (uint8_t)CAN_MESSAGES_ENC_FR_ENDIANNESS) == CAN_MESSAGES_OK) &&
            (encode_u32(can_data, rr_u32,
                        (uint8_t)CAN_MESSAGES_ENC_RR_START_BIT,
                        (uint8_t)CAN_MESSAGES_ENC_RR_LENGTH,
                        (uint8_t)CAN_MESSAGES_ENC_RR_ENDIANNESS) == CAN_MESSAGES_OK))
        {
            status = CAN_MESSAGES_OK;
        }
        else
        {
            /* Keep ERR. */
        }
    }
    else
    {
        /* Keep ERR. */
    }

    return status;
}

CANMessages_StatusTypeDef can_msg_unpack_reference_frame(const uint8_t * can_data,
                                                        can_messages_reference_t * ref)
{
    CANMessages_StatusTypeDef status = CAN_MESSAGES_ERR;

    if ((can_data != NULL) && (ref != NULL))
    {
        int16_t fl = 0;
        int16_t rl = 0;
        int16_t fr = 0;
        int16_t rr = 0;

        if ((decode_signal(can_data, &fl,
                           (uint8_t)CAN_MESSAGES_REF_FL_START_BIT,
                           (uint8_t)CAN_MESSAGES_REF_FL_LENGTH,
                           (uint8_t)CAN_MESSAGES_REF_FL_ENDIANNESS,
                           (uint8_t)sizeof(fl)) == CAN_MESSAGES_OK) &&
            (decode_signal(can_data, &rl,
                           (uint8_t)CAN_MESSAGES_REF_RL_START_BIT,
                           (uint8_t)CAN_MESSAGES_REF_RL_LENGTH,
                           (uint8_t)CAN_MESSAGES_REF_RL_ENDIANNESS,
                           (uint8_t)sizeof(rl)) == CAN_MESSAGES_OK) &&
            (decode_signal(can_data, &fr,
                           (uint8_t)CAN_MESSAGES_REF_FR_START_BIT,
                           (uint8_t)CAN_MESSAGES_REF_FR_LENGTH,
                           (uint8_t)CAN_MESSAGES_REF_FR_ENDIANNESS,
                           (uint8_t)sizeof(fr)) == CAN_MESSAGES_OK) &&
            (decode_signal(can_data, &rr,
                           (uint8_t)CAN_MESSAGES_REF_RR_START_BIT,
                           (uint8_t)CAN_MESSAGES_REF_RR_LENGTH,
                           (uint8_t)CAN_MESSAGES_REF_RR_ENDIANNESS,
                           (uint8_t)sizeof(rr)) == CAN_MESSAGES_OK))
        {
            ref->front_left_rpm  = fl;
            ref->rear_left_rpm   = rl;
            ref->front_right_rpm = fr;
            ref->rear_right_rpm  = rr;

            status = CAN_MESSAGES_OK;
        }
        else
        {
            /* Keep ERR. */
        }
    }
    else
    {
        /* Keep ERR. */
    }

    return status;
}

CANMessages_StatusTypeDef can_msg_imu_pack_gyro_xy_frame(const Cartesian3D * gyro_data,
                                                        uint8_t * can_data)
{
    CANMessages_StatusTypeDef status = CAN_MESSAGES_ERR;

    if ((gyro_data != NULL) && (can_data != NULL))
    {
        (void)memset(can_data, 0, (size_t)CAN_MESSAGES_IMU_DLC_BYTES);

        uint32_t x_bits = 0U;
        uint32_t y_bits = 0U;

        /* Legacy behavior: pack float32 bit patterns. */
        (void)memcpy(&x_bits, &gyro_data->x, sizeof(float));
        (void)memcpy(&y_bits, &gyro_data->y, sizeof(float));

        if ((encode_u32(can_data, x_bits,
                        (uint8_t)CAN_MESSAGES_GYRO_X_START_BIT,
                        (uint8_t)CAN_MESSAGES_GYRO_X_LENGTH,
                        (uint8_t)CAN_MESSAGES_GYRO_X_ENDIANNESS) == CAN_MESSAGES_OK) &&
            (encode_u32(can_data, y_bits,
                        (uint8_t)CAN_MESSAGES_GYRO_Y_START_BIT,
                        (uint8_t)CAN_MESSAGES_GYRO_Y_LENGTH,
                        (uint8_t)CAN_MESSAGES_GYRO_Y_ENDIANNESS) == CAN_MESSAGES_OK))
        {
            status = CAN_MESSAGES_OK;
        }
        else
        {
            /* Keep ERR. */
        }
    }
    else
    {
        /* Keep ERR. */
    }

    return status;
}

CANMessages_StatusTypeDef can_msg_imu_pack_accel_xy_frame(const Cartesian3D * accel_data,
                                                         uint8_t * can_data)
{
    CANMessages_StatusTypeDef status = CAN_MESSAGES_ERR;

    if ((accel_data != NULL) && (can_data != NULL))
    {
        (void)memset(can_data, 0, (size_t)CAN_MESSAGES_IMU_DLC_BYTES);

        uint32_t x_bits = 0U;
        uint32_t y_bits = 0U;

        (void)memcpy(&x_bits, &accel_data->x, sizeof(float));
        (void)memcpy(&y_bits, &accel_data->y, sizeof(float));

        if ((encode_u32(can_data, x_bits,
                        (uint8_t)CAN_MESSAGES_ACCEL_X_START_BIT,
                        (uint8_t)CAN_MESSAGES_ACCEL_X_LENGTH,
                        (uint8_t)CAN_MESSAGES_ACCEL_X_ENDIANNESS) == CAN_MESSAGES_OK) &&
            (encode_u32(can_data, y_bits,
                        (uint8_t)CAN_MESSAGES_ACCEL_Y_START_BIT,
                        (uint8_t)CAN_MESSAGES_ACCEL_Y_LENGTH,
                        (uint8_t)CAN_MESSAGES_ACCEL_Y_ENDIANNESS) == CAN_MESSAGES_OK))
        {
            status = CAN_MESSAGES_OK;
        }
        else
        {
            /* Keep ERR. */
        }
    }
    else
    {
        /* Keep ERR. */
    }

    return status;
}

CANMessages_StatusTypeDef can_msg_imu_pack_z_frame(const Cartesian3D * gyro_data,
                                                  const Cartesian3D * accel_data,
                                                  uint8_t * can_data)
{
    CANMessages_StatusTypeDef status = CAN_MESSAGES_ERR;

    if ((gyro_data != NULL) && (accel_data != NULL) && (can_data != NULL))
    {
        (void)memset(can_data, 0, (size_t)CAN_MESSAGES_IMU_DLC_BYTES);

        uint32_t gz_bits = 0U;
        uint32_t az_bits = 0U;

        (void)memcpy(&gz_bits, &gyro_data->z, sizeof(float));
        (void)memcpy(&az_bits, &accel_data->z, sizeof(float));

        if ((encode_u32(can_data, gz_bits,
                        (uint8_t)CAN_MESSAGES_GYRO_Z_START_BIT,
                        (uint8_t)CAN_MESSAGES_GYRO_Z_LENGTH,
                        (uint8_t)CAN_MESSAGES_GYRO_Z_ENDIANNESS) == CAN_MESSAGES_OK) &&
            (encode_u32(can_data, az_bits,
                        (uint8_t)CAN_MESSAGES_ACCEL_Z_START_BIT,
                        (uint8_t)CAN_MESSAGES_ACCEL_Z_LENGTH,
                        (uint8_t)CAN_MESSAGES_ACCEL_Z_ENDIANNESS) == CAN_MESSAGES_OK))
        {
            status = CAN_MESSAGES_OK;
        }
        else
        {
            /* Keep ERR. */
        }
    }
    else
    {
        /* Keep ERR. */
    }

    return status;
}
