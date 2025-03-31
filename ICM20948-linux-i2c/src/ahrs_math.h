#pragma once


#include "imu_data.h"

/**
 * Fast inverse square root algorithm from http://en.wikipedia.org/wiki/Fast_inverse_square_root
 *  @param x input data
 *  @return the result
 */
float invSqrt(const float x);

/**
 * Converts quaternion to Euler angles.
 *  @param[in/out] data the IMU data structure with quaternion and Euler angles.
 */
void quatToAngles(IMUData& data);
