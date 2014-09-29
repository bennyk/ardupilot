/*
 *  Example of AP_Motors library.
 *  Code by Randy Mackay. DIYDrones.com
 */

// Libraries
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Param.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_PX4.h>
#include <AP_HAL_Linux.h>
#include <AP_HAL_Empty.h>
#include <AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel.h>     // RC Channel Library
#include <AP_Motors.h>
#include <AP_Curve.h>
#include <AP_Notify.h>
#include <AP_GPS.h>
#include <DataFlash.h>
#include <AP_InertialSensor.h>
#include <AP_ADC.h>
#include <GCS_MAVLink.h>
#include <AP_Baro.h>
#include <Filter.h>
#include <AP_AHRS.h>
#include <AP_Compass.h>
#include <AP_Declination.h>
#include <AP_Airspeed.h>
#include <AP_Vehicle.h>
#include <AP_ADC_AnalogSource.h>
#include <AP_Mission.h>
#include <StorageManager.h>
#include <AP_Terrain.h>
#include <AP_NavEKF.h>

#include <RCOutput_I2C.h>

#include <unistd.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

class PCA9685 {
public:
	PCA9685();

	void init();
	void setFrequency(uint16_t freq_hz);
	float getFrequency();
	void restart();
	void sleep();

	void setPWM(uint8_t channel, uint16_t offset, uint16_t length);
	uint8_t writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data);

private:
    uint8_t _devAddr;
    float _frequency;
};

PCA9685 chip;

PCA9685::PCA9685()
: _devAddr(PCA9685_DEFAULT_ADDRESS), _frequency(0.0)
{}

void PCA9685::sleep() {
    writeBit(_devAddr, PCA9685_RA_MODE1, PCA9685_MODE1_SLEEP_BIT, 1);
}

/*
void PCA9685::setFrequency_(uint16_t freq_hz)            //LSB corresponds to CHAN_1
{
    sleep();
    usleep(10000);
    uint8_t prescale = roundf(25000000.f / 4096.f / freq_hz)  - 1;
    hal.console->printf("PCA9685: computed prescale value %d\n", prescale);
    hal.i2c->writeRegister(_devAddr, PCA9685_RA_PRE_SCALE, prescale);
    _frequency = getFrequency();
    restart();

	hal.console->printf("PCA9685: setting frequency %.2f\n", _frequency);
}
*/

void PCA9685::setFrequency(uint16_t freq_hz)            //LSB corresponds to CHAN_1
{
    sleep();
    usleep(10000);
    uint8_t prescale = roundf(25000000.f / 4096.f / freq_hz)  - 1;
    prescale = floor((double)prescale + 0.5);

    uint8_t oldmode;
    hal.i2c->readRegister(_devAddr, PCA9685_RA_MODE1, &oldmode);
    uint8_t newmode = (oldmode & 0x7F) | 0x10;

    hal.i2c->writeRegister(_devAddr, PCA9685_RA_MODE1, newmode);

    hal.console->printf("PCA9685: computed prescale value %d\n", prescale);
    hal.i2c->writeRegister(_devAddr, PCA9685_RA_PRE_SCALE, prescale);


    hal.i2c->writeRegister(_devAddr, PCA9685_RA_MODE1, oldmode);

    usleep(0.005 * 1e6);

    hal.i2c->writeRegister(_devAddr, PCA9685_RA_MODE1, oldmode | 0x80);

    _frequency = getFrequency();
    restart();

	hal.console->printf("PCA9685: setting frequency %.2f\n", _frequency);
}


float PCA9685::getFrequency() {
    uint8_t data;
    hal.i2c->readRegister(_devAddr, PCA9685_RA_PRE_SCALE, &data);
    return 25000000.f / 4096.f / (data + 1);
}

void PCA9685::restart() {
	writeBit(_devAddr, PCA9685_RA_MODE1, PCA9685_MODE1_SLEEP_BIT, 0);
	writeBit(_devAddr, PCA9685_RA_MODE1, PCA9685_MODE1_RESTART_BIT, 1);
}

uint8_t PCA9685::writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data)
{
	uint8_t b;
	hal.i2c->readRegister(devAddr, regAddr, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    return hal.i2c->writeRegister(devAddr, regAddr, b);
}

void PCA9685::setPWM(uint8_t channel, uint16_t offset, uint16_t length)
{
	hal.console->printf("setPWM %d %d %d\n", channel, offset, length);

//    uint8_t data[4] = {offset & 0xFF, offset >> 8, length & 0xFF, length >> 8};
//    hal.i2c->writeRegisters(_devAddr, PCA9685_RA_LED0_ON_L + 4 * channel, 4, data);

	hal.i2c->writeRegister(_devAddr, PCA9685_RA_LED0_ON_L + 4 * channel, offset & 0xff);
	hal.i2c->writeRegister(_devAddr, PCA9685_RA_LED0_ON_H + 4 * channel, offset >> 8);
	hal.i2c->writeRegister(_devAddr, PCA9685_RA_LED0_OFF_L + 4 * channel, length & 0xff);
	hal.i2c->writeRegister(_devAddr, PCA9685_RA_LED0_OFF_H + 4 * channel, length >> 8);
}

void PCA9685::init()
{
	writeBit(_devAddr, PCA9685_RA_MODE1, PCA9685_MODE1_AI_BIT, 1);
	_frequency = getFrequency();
	restart();

	hal.console->printf("PCA9685: base frequency %.2f\n", _frequency);
}


// setup
void setup()
{
//	chip.init();
	hal.console->printf("setting up\n");

	AP_HAL::Semaphore *i2c_sem = hal.i2c->get_semaphore();

//	hal.i2c->begin();
	chip.setFrequency(30);

}

// loop
void loop()
{

	const int servoMin = 150; // Min pulse length out of 4096
	const int servoMax = 4095;  // Max pulse length out of 4096

    int16_t value;

    // display help
    hal.console->println("Press 't' to run motor orders test, 's' to run stability patch test.  Be careful the motors will spin!");

    // wait for user to enter something
    while( !hal.console->available() ) {
        hal.scheduler->delay(20);
    }

    // get character from user
    value = hal.console->read();

    while (true) {
    	chip.setPWM(0, 0, servoMin);
        usleep(1000000);
    	chip.setPWM(0, 0, servoMax);
    	usleep(1000000);

    	chip.setPWM(1, 0, servoMin);
        usleep(1000000);
    	chip.setPWM(1, 0, servoMax);
    	usleep(1000000);

    	chip.setPWM(2, 0, servoMin);
        usleep(1000000);
    	chip.setPWM(2, 0, servoMax);
    	usleep(1000000);

    	chip.setPWM(3, 0, servoMin);
        usleep(1000000);
    	chip.setPWM(3, 0, servoMax);
    	usleep(1000000);

    }

}


AP_HAL_MAIN();
