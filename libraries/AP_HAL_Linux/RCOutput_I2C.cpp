
#include <AP_HAL.h>

//#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include "RCOutput_I2C.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <dirent.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <sys/mman.h>
#include <signal.h>
#include <math.h>

using namespace Linux;

extern const AP_HAL::HAL& hal;

LinuxRCOutput_I2C::LinuxRCOutput_I2C()
: _devAddr(PCA9685_DEFAULT_ADDRESS),
  _frequency(0.0),
  _debug(true)
{}

uint8_t LinuxRCOutput_I2C::writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data)
{
	uint8_t b;
	hal.i2c->readRegister(devAddr, regAddr, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return hal.i2c->writeRegister(devAddr, regAddr, b);
}

void LinuxRCOutput_I2C::setPWM(uint8_t channel, uint16_t offset, uint16_t length)
{
	if (_debug)
		hal.console->printf("LinuxRCOutput_I2C::setPWM %d %d %d\n", channel, offset, length);

	// FIXME we have not validate writeRegisters on mbmd
//    uint8_t data[4] = {offset & 0xFF, offset >> 8, length & 0xFF, length >> 8};
//    hal.i2c->writeRegisters(_devAddr, PCA9685_RA_LED0_ON_L + 4 * channel, 4, data);

	hal.i2c->writeRegister(_devAddr, PCA9685_RA_LED0_ON_L + 4 * channel, offset & 0xff);
	hal.i2c->writeRegister(_devAddr, PCA9685_RA_LED0_ON_H + 4 * channel, offset >> 8);
	hal.i2c->writeRegister(_devAddr, PCA9685_RA_LED0_OFF_L + 4 * channel, length & 0xff);
	hal.i2c->writeRegister(_devAddr, PCA9685_RA_LED0_OFF_H + 4 * channel, length >> 8);
}

void LinuxRCOutput_I2C::setPWM(uint8_t channel, uint16_t length)
{
	setPWM(channel, 0, length);
}

float LinuxRCOutput_I2C::getFrequency() {
    uint8_t data;
    hal.i2c->readRegister(_devAddr, PCA9685_RA_PRE_SCALE, &data);
    return 25000000.f / 4096.f / (data + 1);
}

void LinuxRCOutput_I2C::restart() {
	writeBit(_devAddr, PCA9685_RA_MODE1, PCA9685_MODE1_SLEEP_BIT, 0);
	writeBit(_devAddr, PCA9685_RA_MODE1, PCA9685_MODE1_RESTART_BIT, 1);
}

void LinuxRCOutput_I2C::sleep() {
    writeBit(_devAddr, PCA9685_RA_MODE1, PCA9685_MODE1_SLEEP_BIT, 1);
}

void LinuxRCOutput_I2C::init(void* implspecific)
{
	writeBit(_devAddr, PCA9685_RA_MODE1, PCA9685_MODE1_AI_BIT, 1);
	_frequency = getFrequency();
	restart();

	if (_debug)
		hal.console->printf("LinuxRCOutput_I2C: base frequency %.2f\n", _frequency);
}

void LinuxRCOutput_I2C::set_freq(uint32_t chmask, uint16_t freq_hz)            //LSB corresponds to CHAN_1
{
    sleep();
    usleep(10000);
    uint8_t prescale = roundf(25000000.f / 4096.f / freq_hz)  - 1;
    hal.i2c->writeRegister(_devAddr, PCA9685_RA_PRE_SCALE, prescale);
    _frequency = getFrequency();
    restart();

    if (_debug)
    	hal.console->printf("LinuxRCOutput_I2C: setting frequency %.2f\n", _frequency);
}

uint16_t LinuxRCOutput_I2C::get_freq(uint8_t ch)
{
    uint8_t data;
    hal.i2c->readRegister(_devAddr, PCA9685_RA_PRE_SCALE, &data);
    return roundf(25000000.f / 4096.f / (data + 1));
}

void LinuxRCOutput_I2C::enable_ch(uint8_t ch)
{
	if (_debug)
		hal.console->printf("WARNING: LinuxRCOutput_I2C::enable_ch(%d) not implemented.\n", ch);
}

void LinuxRCOutput_I2C::disable_ch(uint8_t ch)
{
	if (_debug)
		hal.console->printf("WARNING: LinuxRCOutput_I2C::disable_ch(%d) not implemented.\n", ch);
}

void LinuxRCOutput_I2C::write(uint8_t ch, uint16_t period_us)
{
    setPWM(ch, round((period_us * 4096.f) / (1000000.f / _frequency) - 1));

    if (_debug)
    	hal.console->printf("LinuxRCOutput_I2C::write(%d, %d)\n", ch, period_us);

    // to interpret period as PWM value.
//	setPWM(ch, period_us);
}

void LinuxRCOutput_I2C::write(uint8_t ch, uint16_t* period_us, uint8_t len)
{
	if (_debug)
		hal.console->printf("WARNING: LinuxRCOutput_I2C::write(%d, &period, %d) not implemented.\n", ch, len);
}

uint16_t LinuxRCOutput_I2C::read(uint8_t ch)
{
	uint8_t data[4];
	hal.i2c->readRegisters(_devAddr, PCA9685_RA_LED0_ON_L + 4 * ch, 4, data);
	return 0;
}

void LinuxRCOutput_I2C::read(uint16_t* period_us, uint8_t len)
{
	if (_debug)
		hal.console->printf("WARNING: LinuxRCOutput_I2C::read(&period, %d) not implemented.\n", len);
}

void LinuxRCOutput_I2C::set_failsafe_pwm(uint32_t chmask, uint16_t period_us)
{
	if (_debug)
		hal.console->printf("WARNING: LinuxRCOutput_I2C::set_failsafe_pwm(%d, %d) not implemented.\n", chmask, period_us);
}

//#endif
