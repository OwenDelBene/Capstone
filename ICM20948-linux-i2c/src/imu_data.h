#pragma once


/**
 * A structure to hold all data from IMU.
 */
struct IMUData
{
    /** The period at which IMU data is being pulled from the sensor. */
    float mUpdatePeriod;
    /** The temperature in Celsius degrees. */
    float mTemp;
    /** Raw angular rate from gyroscope in rad/s. */
    float mGyro[3];
    /** Raw data from accelerometer in G's. */
    float mAcc[3];
    /** Raw data from compass in uT. */
    float mMag[3];
    /** Quaternion representation of IMU orientation. */
    float mQuat[4];
    /** Euler angles representation of IMU orientation in degrees. */
    float mAngles[3];

    /**
     * Basic constructor which initialises all variables with default values.
     */
    IMUData() : mUpdatePeriod(0.0f), mTemp(0.0f)
    {
        mGyro[0] = mGyro[1] = mGyro[2] = 0.0f;
        mAcc[0] = mAcc[1] = mAcc[2] = 0.0f;
        mMag[0] = mMag[1] = mMag[2] = 0.0f;
        mQuat[0] = 1.0;
        mQuat[1] = mQuat[2] = mQuat[3] = 0.0f;
        mAngles[0] = mAngles[1] = mAngles[2] = 0.0f;
    };
};
