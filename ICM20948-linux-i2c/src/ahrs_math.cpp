#include <cmath>
#include "ahrs_math.h"

/** Scale to convert angles from radians to degrees. */
static const float RAD_TO_DEG = 180.0f / M_PI;

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
float invSqrt(const float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

void quatToAngles(IMUData& data)
{
    data.mAngles[1] = 2 * (data.mQuat[0] * data.mQuat[2] - data.mQuat[1] * data.mQuat[3]);
    if (fabs(data.mAngles[1]) >= 1)
    {
        data.mAngles[1] = std::copysign(M_PI / 2, data.mAngles[1]) * RAD_TO_DEG;
    }
    else
    {
        data.mAngles[1] = asin(data.mAngles[1]) * RAD_TO_DEG;
    }
    data.mAngles[0] = atan2(2 * (data.mQuat[0] * data.mQuat[1] + data.mQuat[2] * data.mQuat[3]), 1 - 2 * (data.mQuat[1] * data.mQuat[1] + data.mQuat[2] * data.mQuat[2])) * RAD_TO_DEG;
    data.mAngles[2] = atan2(2 * (data.mQuat[0] * data.mQuat[3] + data.mQuat[1] * data.mQuat[2]), 1 - 2 * (data.mQuat[2] * data.mQuat[2] + data.mQuat[3] * data.mQuat[3])) * RAD_TO_DEG;
}
