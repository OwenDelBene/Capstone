#include <cstring>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <syslog.h>
#include <unistd.h>
#include "i2c.h"


I2C::I2C(): mSerial(-1), mCurrentDevice(0)
{
}

I2C::~I2C()
{
    closeSerialPort();
}

bool I2C::openSerialPort(const char* device)
{
    // make sure the device is closed before opening a new one
    closeSerialPort();
    mSerial = open(device, O_RDWR);
    return mSerial >= 0;
}

void I2C::closeSerialPort()
{
    if (mSerial >= 0)
    {
        close(mSerial);
        mSerial = -1;
        mCurrentDevice = 0;
    }
}

uint8_t I2C::readByte(const uint8_t deviceAddr, const uint8_t regAddr) const
{
    uint8_t u8Ret = 0;
    if (connectToDevice(deviceAddr))
    {
        if (1 == write(mSerial, &regAddr, 1))
        {
            if (1 != read(mSerial, &u8Ret, 1))
            {
                syslog(LOG_ERR, "Failed to read data from device %02x, address: %02x.", deviceAddr, regAddr);
                u8Ret = 0;
            }
        }
        else
        {
            syslog(LOG_ERR, "Failed to write request to device: %02x, address: %02x in order to obtain data.", deviceAddr, regAddr);
        }
    }
    return u8Ret;
}

void I2C::readNBytes(const uint8_t deviceAddr, const uint8_t regAddr, const uint8_t length, uint8_t data[]) const
{
    if (connectToDevice(deviceAddr))
    {
        if (1 == write(mSerial, &regAddr, 1))
        {
            if (length != read(mSerial, data, length))
            {
                syslog(LOG_ERR, "Failed to read data from device %02x, address: %02x.", deviceAddr, regAddr);
                memset(data, 0, length * sizeof(uint8_t));
            }
        }
        else
        {
            syslog(LOG_ERR, "Failed to write request to device: %02x, address: %02x in order to obtain data.", deviceAddr, regAddr);
        }
    }
}

void I2C::writeByte(const uint8_t deviceAddr, const uint8_t regAddr, const uint8_t value) const
{
    uint8_t buf[2] = {regAddr, value};
    if (connectToDevice(deviceAddr))
    {
        if (2 != write(mSerial, buf, 2))
        {
            syslog(LOG_ERR, "Failed to write %02x to device: %02x, address: %02x.", value, deviceAddr, regAddr);
        }
    }
}

void I2C::writeData(const uint8_t deviceAddr, const uint8_t length, const uint8_t data[]) const
{
    if (connectToDevice(deviceAddr))
    {
        if (length != write(mSerial, data, length))
        {
            syslog(LOG_ERR, "Failed to write %d bytes to device: %02x, address: %02x.", length, deviceAddr, data[0]);
        }
    }
}

bool I2C::connectToDevice(const uint8_t deviceAddr) const
{
    if (deviceAddr != mCurrentDevice)
    {
        if (ioctl(mSerial, I2C_SLAVE, deviceAddr) >= 0)
        {
            mCurrentDevice = deviceAddr;
        }
        else
        {
            syslog(LOG_ERR, "Failed to connect to device: %02x.", deviceAddr);
            return false;
        }
    }
    return true;
}
