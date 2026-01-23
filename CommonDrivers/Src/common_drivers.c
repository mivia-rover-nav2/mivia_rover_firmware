/**
 * @file common_drivers.c
 * @brief Implementation of Common Driver Utilities.
 *
 * This source file provides the implementation for common utility functions
 * defined in common_drivers.h.
 *
 * Created on: Jun 25, 2024
 * Author: Alessio Guarini, Antonio Vitale
 */

#include "common_drivers.h"

float clamp_float(float x, float min, float max) {
	float ret_val;
	if (x < min) {
		ret_val = min;
	} else if (x > max) {
		ret_val = max;
	} else {
		ret_val = x;
	}
	return ret_val;
}

double clamp_double(double x, double min, double max)
{
    double ret_val;

    if (x < min)
    {
        ret_val = min;
    }
    else if (x > max)
    {
        ret_val = max;
    }
    else
    {
        ret_val = x;
    }

    return ret_val;
}


float map_value_float(float x, float in_min, float in_max, float out_min,
		float out_max) {
	float ret_val;
	if (in_max == in_min) {
		ret_val = out_min;
	} else {
		ret_val = clamp_float(x, in_min, in_max);
		ret_val = (ret_val - in_min) * (out_max - out_min) / (in_max - in_min)
				+ out_min;
	}
	return ret_val;
}

float calculate_new_smoothed_value(float current_value, float set_point,
		float max_increment, float max_decrement) {
	float error = set_point - current_value;
	float new_smoothed = 0.0f;
	if (error != 0.0f) {
		static const float positive_sign = 1.0f;
		static const float negative_sign = -1.0f;
		float error_sign = (error < 0.0f) ? negative_sign : positive_sign;
		float adjustment = 0.0f;
		if (((current_value > 0.0f) && (error < 0.0f))
				|| ((current_value < 0.0f) && (error > 0.0f))) {
			adjustment =
					(fabs(current_value) <= max_decrement) ?
							-current_value : (error_sign * max_decrement);
		} else {
			float abs_error = fabs(error);
			adjustment = error_sign
					* ((abs_error > max_increment) ? max_increment : abs_error);
		}
		new_smoothed = current_value + adjustment;
		if (((adjustment > 0.0f) && (new_smoothed > set_point))
				|| ((adjustment < 0.0f) && (new_smoothed < set_point))) {
			new_smoothed = set_point;
		}
	} else {
		new_smoothed = set_point;
	}
	return (float) new_smoothed;
}

