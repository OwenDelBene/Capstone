#include <sys/stat.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <stdio.h>      /* Standard I/O functions */
#include <fcntl.h>
#include <syslog.h>		/* Syslog functionallity */
#include <inttypes.h>
#include <errno.h>
#include <math.h>

#include "PCA9685.h"

//! Constructor takes bus and address arguments
/*!
 \param bus the bus to use in /dev/i2c-%d.
 \param address the device address on bus
 */
PCA9685::PCA9685(char* bus, int address) {
  this->address = address;
  if (i2c.openSerialPort(bus)	)
  {
  reset();
	setPWMFreq(400);

  }
  else {
    printf("Failed to open device\n");

  }
}

PCA9685::~PCA9685() {
  i2c.closeSerialPort();
}
//! Sets PCA9685 mode to 00
void PCA9685::reset() {

		i2c.writeByte(address, MODE1, 0x00); //Normal mode
		i2c.writeByte(address, MODE2, 0x04); //totem pole

}
//! Set the frequency of PWM
/*!
 \param freq desired frequency. 40Hz to 1000Hz using internal 25MHz oscillator.
 */
void PCA9685::setPWMFreq(int freq) {

		uint8_t prescale_val = (CLOCK_FREQ / 4096 / freq)  - 1;
		i2c.writeByte(address, MODE1, 0x10); //sleep
		i2c.writeByte(address, PRE_SCALE, prescale_val); // multiplyer for PWM frequency
		i2c.writeByte(address, MODE1, 0x80); //restart
		i2c.writeByte(address, MODE2, 0x04); //totem pole (default)
}

//! PWM a single channel
/*!
 \param led channel (1-16) to set PWM value for
 \param value 0-4095 value for PWM
 */
void PCA9685::setPWM(uint8_t led, int value) {
	setPWM(led, 0, value);
}
//! PWM a single channel with custom on time
/*!
 \param led channel (1-16) to set PWM value for
 \param on_value 0-4095 value to turn on the pulse
 \param off_value 0-4095 value to turn off the pulse
 */
void PCA9685::setPWM(uint8_t led, int on_value, int off_value) {
		i2c.writeByte(address, LED0_ON_L + LED_MULTIPLYER * (led - 1), on_value & 0xFF);
		i2c.writeByte(address, LED0_ON_H + LED_MULTIPLYER * (led - 1), on_value >> 8);
		i2c.writeByte(address, LED0_OFF_L + LED_MULTIPLYER * (led - 1), off_value & 0xFF);
		i2c.writeByte(address, LED0_OFF_H + LED_MULTIPLYER * (led - 1), off_value >> 8);
}

//! Get current PWM value
/*!
 \param led channel (1-16) to get PWM value from
 */
int PCA9685::getPWM(uint8_t led){
	int ledval = 0;
	ledval = i2c.readByte(address, LED0_OFF_H + LED_MULTIPLYER * (led-1));
	ledval = ledval & 0xf;
	ledval <<= 8;
	ledval += i2c.readByte(address, LED0_OFF_L + LED_MULTIPLYER * (led-1));
	return ledval;
}
