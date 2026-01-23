/**
 * @file common_drivers.h
 * @brief Header file for Common Driver Utilities.
 *
 * This file contains the definitions and prototypes for common utility functions
 * used across different modules, including mathematical operations and value transformations.
 *
 * Created on: Jun 25, 2024
 * Author: Alessio Guarini, Antonio Vitale
 */

#ifndef INC_COMMON_DRIVERS_H_
#define INC_COMMON_DRIVERS_H_

#include "stdint.h"
#include "math.h"

/**
 * @brief Unsigned 8-bit boolean type.
 *
 * This typedef defines an unsigned 8-bit integer to be used as a boolean type.
 */
typedef uint8_t bool8u;

/**
 * @brief Boolean true value.
 *
 * This macro defines the true value for the `bool8u` type.
 */
#define CD_TRUE ((bool8u) 1U)
/**
 * @brief Boolean false value.
 *
 * This macro defines the false value for the `bool8u` type.
 */
#define CD_FALSE ((bool8u) 0U)

/**
 * Clamps a float value to a specified range.
 *
 * @param[in] x The value to clamp.
 * @param[in] min The minimum allowable value.
 * @param[in] max The maximum allowable value.
 * @return The clamped value.
 *
 * @note This function assumes that min is less than or equal to max.
 *       No validation is done inside the function to check this.
 */
float clamp_float(float x, float min, float max);

/**
 * Clamps a double value to a specified range.
 *
 * @param[in] x The value to clamp.
 * @param[in] min The minimum allowable value.
 * @param[in] max The maximum allowable value.
 * @return The clamped value.
 *
 * @note This function assumes that min is less than or equal to max.
 *       No validation is done inside the function to check this.
 */
double clamp_double(double x, double min, double max);


/**
 * Maps a float value from one range to another.
 *
 * @param[in] x The value to map.
 * @param[in] in_min The minimum value of the input range.
 * @param[in] in_max The maximum value of the input range.
 * @param[in] out_min The minimum value of the output range.
 * @param[in] out_max The maximum value of the output range.
 * @return The mapped value.
 *
 */
float map_value_float(float x, float in_min, float in_max, float out_min,
		float out_max);

/**
 * @brief Calculates a new smoothed value.
 *
 * This function calculates a new smoothed value based on the current value,
 * set point, maximum increment, and maximum decrement. It adjusts the current
 * value towards the set point within the given constraints.
 *
 * @param[in] current_value The current value.
 * @param[in] set_point The target value.
 * @param[in] max_increment The maximum increment value.
 * @param[in] max_decrement The maximum decrement value.
 * @return The new smoothed value.
 */
float calculate_new_smoothed_value(float current_value, float set_point,
		float max_increment, float max_decrement);

#endif /* INC_COMMON_DRIVERS_H_ */
