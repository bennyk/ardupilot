
#include "AP_InertialSensor_MPU60X0.h"

extern const AP_HAL::HAL& hal;

// Gyroscope scale (uncertain where the 0.01745 value comes from)
#define MPU9150_GYRO_SCALE_2000  (0.0174532f / 16.4f)
#define MPU9150_GYRO_SCALE_1000  (0.0174532f / 32.8f)
#define MPU9150_GYRO_SCALE_500  (0.0174532f / 65.5f)
#define MPU9150_GYRO_SCALE_250  (0.0174532f / 131f)

// Accelerometer scale adjustment
#define MPU9150_ACCEL_SCALE_16G    (GRAVITY_MSS / 2048.0f)
#define MPU9150_ACCEL_SCALE_8G    (GRAVITY_MSS / 4096.0f)
#define MPU9150_ACCEL_SCALE_4G    (GRAVITY_MSS / 8192.0f)
#define MPU9150_ACCEL_SCALE_2G    (GRAVITY_MSS / 16384.0f)

//#define MPU60X0_ENABLE_FIFO

AP_InertialSensor_MPU60X0::AP_InertialSensor_MPU60X0() :
    AP_InertialSensor(),
    _accel_filter_x(800, 10),
    _accel_filter_y(800, 10),
    _accel_filter_z(800, 10),
    _gyro_filter_x(800, 10),
    _gyro_filter_y(800, 10),
    _gyro_filter_z(800, 10),

    _last_sample_timestamp(0)
{
}

uint16_t AP_InertialSensor_MPU60X0::_init_sensor( Sample_rate sample_rate )
{
//	_accelgyro.initialize();
    hal.console->println("init MPU60X0 driver...");

    switch (sample_rate) {
    case RATE_50HZ:
        _default_filter_hz = 10;
        _sample_period_usec = (1000*1000) / 50;
        break;
    case RATE_100HZ:
        _default_filter_hz = 20;
        _sample_period_usec = (1000*1000) / 100;
        break;
    case RATE_200HZ:
        _default_filter_hz = 20;
        _sample_period_usec = 5000;
        break;
    case RATE_400HZ:
    default:
        _default_filter_hz = 20;
        _sample_period_usec = 2500;
        break;
    }

    // get pointer to i2c bus semaphore
    AP_HAL::Semaphore* i2c_sem = hal.i2c->get_semaphore();

    // take i2c bus semaphore
    if (!i2c_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)){
        return -1;
    }

	_accelgyro.reset();
    hal.scheduler->delay(100);

#ifdef HMC5843_IN_MPU60X0_I2CAUX
    // enabling the MPU60X0 auxport
    _accelgyro.setI2CMasterModeEnabled(false);
    _accelgyro.setI2CBypassEnabled(true);
#endif

    _accelgyro.setSleepEnabled(false);
    _accelgyro.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
    _accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    _accelgyro.setClockSource(MPU6050_CLOCK_PLL_XGYRO);

    mpu_set_lpf(_default_filter_hz);
    mpu_set_sample_rate(800);

#ifdef MPU60X0_ENABLE_FIFO
    _accelgyro.setFIFOEnabled(false);
    _accelgyro.resetFIFO();
    hal.scheduler->delay(100);

    _accelgyro.setTempFIFOEnabled(false);
    _accelgyro.setSlave0FIFOEnabled(false);
    _accelgyro.setSlave1FIFOEnabled(false);
    _accelgyro.setSlave2FIFOEnabled(false);
    _accelgyro.setSlave3FIFOEnabled(false);

    _accelgyro.setAccelFIFOEnabled(true);
    _accelgyro.setXGyroFIFOEnabled(true);
    _accelgyro.setYGyroFIFOEnabled(true);
    _accelgyro.setZGyroFIFOEnabled(true);

    _accelgyro.setInterruptMode(false);
    _accelgyro.setFIFOEnabled(true);

    // clear interrupt flags
    _accelgyro.getIntStatus();

#endif

    _set_filter_frequency(_mpu6000_filter);

    i2c_sem->give();

    hal.scheduler->register_timer_process(AP_HAL_MEMBERPROC(&AP_InertialSensor_MPU60X0::_accumulate));

	return 0;
}

#ifdef MPU60X0_ENABLE_FIFO

void AP_InertialSensor_MPU60X0::_accumulate(void){

	// get pointer to i2c bus semaphore
	AP_HAL::Semaphore* i2c_sem = hal.i2c->get_semaphore();

	// take i2c bus semaphore
	if (!i2c_sem->take_nonblocking()){
		return;
	}

    int i = 0, index = 0;
	
    int16_t ax, ay, az;
	int16_t gx, gy, gz;
    
    const int packetSize = 12;
    uint8_t data[packetSize];

    uint16_t fifoCount = _accelgyro.getFIFOCount();
    uint8_t status = _accelgyro.getIntStatus();
    
    if (true && ((status & 0x10) || fifoCount == 1024)) {
        _accelgyro.resetFIFO();
        hal.console->printf("FIFO overflown! %d bytes\n", fifoCount);
        i2c_sem->give();
        return;
    }
    else if (status & 0x01) {
        int remainingBytes = fifoCount;
        while (remainingBytes > packetSize)
        {
            _accelgyro.getFIFOBytes(data, packetSize);
            i++;
            //hal.console->printf("got %d samples\n", fifoCount);

            index = 0;
            ax = (data[index+0] << 8) | data[index+1];
            index += 2;
            ay = (data[index+0] << 8) | data[index+1];
            index += 2;
            az = (data[index+0] << 8) | data[index+1];
            index += 2;
            gx = (data[index+0] << 8) | data[index+1];
            index += 2;
            gy = (data[index+0] << 8) | data[index+1];
            index += 2;
            gz = (data[index+0] << 8) | data[index+1];

            remainingBytes -= packetSize;
        
            //hal.console->printf("a/g: %d %6hd %6hd %6hd   %6hd %6hd %6hd\n",i, ax,ay,az,gx,gy,gz);
	
            _accel_filtered = Vector3f(
                _accel_filter_x.apply(ax),
                _accel_filter_y.apply(ay),
                _accel_filter_z.apply(az));

            _gyro_filtered = Vector3f(
                _gyro_filter_x.apply(gx),
                _gyro_filter_y.apply(gy),
                _gyro_filter_z.apply(gz));

            _gyro_samples_available++;
       }
    }
    i2c_sem->give();

}

#else
void AP_InertialSensor_MPU60X0::_accumulate(void){

	// get pointer to i2c bus semaphore
	AP_HAL::Semaphore* i2c_sem = hal.i2c->get_semaphore();

	// take i2c bus semaphore
	if (!i2c_sem->take_nonblocking()){
		return;
	}

	int16_t ax, ay, az;
	int16_t gx, gy, gz;

    _accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    //hal.console->printf("a/g: %d %d %d %d %d %d\n", ax, ay, az, gx, gy, gz);

    _accel_filtered = Vector3f(
        _accel_filter_x.apply(ax),
        _accel_filter_y.apply(ay),
        _accel_filter_z.apply(az));

    _gyro_filtered = Vector3f(
        _gyro_filter_x.apply(gx),
        _gyro_filter_y.apply(gy),
        _gyro_filter_z.apply(gz));

    _gyro_samples_available++;

	i2c_sem->give();

}
#endif

bool AP_InertialSensor_MPU60X0::_sample_available(void)
{
    uint64_t tnow =  hal.scheduler->micros();
    while (tnow - _last_sample_timestamp > _sample_period_usec) {
        _have_sample_available = true;
        _last_sample_timestamp += _sample_period_usec;
    }
    return _have_sample_available;
}

void AP_InertialSensor_MPU60X0::_set_filter_frequency(uint8_t filter_hz)
{
    if (filter_hz == 0)
        filter_hz = _default_filter_hz;

    _accel_filter_x.set_cutoff_frequency(800, filter_hz);
    _accel_filter_y.set_cutoff_frequency(800, filter_hz);
    _accel_filter_z.set_cutoff_frequency(800, filter_hz);
    _gyro_filter_x.set_cutoff_frequency(800, filter_hz);
    _gyro_filter_y.set_cutoff_frequency(800, filter_hz);
    _gyro_filter_z.set_cutoff_frequency(800, filter_hz);
}

bool AP_InertialSensor_MPU60X0::wait_for_sample(uint16_t timeout_ms)
{
    if (_sample_available()) {
        return true;
    }
    uint32_t start = hal.scheduler->millis();
    while ((hal.scheduler->millis() - start) < timeout_ms) {
        uint64_t tnow = hal.scheduler->micros();
        // we spin for the last timing_lag microseconds. Before that
        // we yield the CPU to allow IO to happen
        const uint16_t timing_lag = 400;
        if (_last_sample_timestamp + _sample_period_usec > tnow+timing_lag) {
            hal.scheduler->delay_microseconds(_last_sample_timestamp + _sample_period_usec - (tnow+timing_lag));
        }
        if (_sample_available()) {
            return true;
        }
    }
    return false;
}

bool AP_InertialSensor_MPU60X0::update(void)
{
    if (!wait_for_sample(1000)) {
        return false;
    }
    Vector3f accel_scale = _accel_scale[0].get();

    _previous_accel[0] = _accel[0];

    // hal.scheduler->suspend_timer_procs();
    _accel[0] = _accel_filtered;
    _gyro[0] = _gyro_filtered;
    // hal.scheduler->resume_timer_procs();

    // add offsets and rotation
    _accel[0].rotate(_board_orientation);

    // Adjust for chip scaling to get m/s/s
    ////////////////////////////////////////////////
    _accel[0] *= MPU9150_ACCEL_SCALE_2G/_gyro_samples_available;

    // Now the calibration scale factor
    _accel[0].x *= accel_scale.x;
    _accel[0].y *= accel_scale.y;
    _accel[0].z *= accel_scale.z;
    _accel[0]   -= _accel_offset[0];

    _gyro[0].rotate(_board_orientation);

    // Adjust for chip scaling to get radians/sec
    _gyro[0] *= MPU9150_GYRO_SCALE_2000 / _gyro_samples_available;
    _gyro[0] -= _gyro_offset[0];
    ////////////////////////////////////////////////

    _gyro_samples_available = 0;

    if (_last_filter_hz != _mpu6000_filter) {
        _set_filter_frequency(_mpu6000_filter);
        _last_filter_hz = _mpu6000_filter;
    }

    _have_sample_available = false;

    return true;
}

// TODO review to make sure it matches
float AP_InertialSensor_MPU60X0::get_gyro_drift_rate(void)
{
    // 0.5 degrees/second/minute (a guess)
    return ToRad(0.5/60);
}

// TODO review to make sure it matches
float AP_InertialSensor_MPU60X0::get_delta_time(void) const
{
    return _sample_period_usec * 1.0e-6f;
}

void AP_InertialSensor_MPU60X0::mpu_set_lpf(uint16_t lpf)
{
    uint8_t mode;

    if (lpf >= 188){
        mode = MPU6050_DLPF_BW_188;
    }
    else if (lpf >= 98){
        mode = MPU6050_DLPF_BW_98;
    }
    else if (lpf >= 42){
        mode = MPU6050_DLPF_BW_42;
    }
    else if (lpf >= 20){
        mode = MPU6050_DLPF_BW_20;
    }
    else if (lpf >= 10){
        mode = MPU6050_DLPF_BW_10;
    }
    else {
        mode = MPU6050_DLPF_BW_5;
    }

    _accelgyro.setDLPFMode(mode);
}

void AP_InertialSensor_MPU60X0::mpu_set_sample_rate(uint16_t rate)
{
    uint8_t data;
    // uint16_t sample_rate;

    if (rate < 4){
        rate = 4;
    }
    else if (rate > 1000){
        rate = 1000;
    }

    data = 1000 / rate - 1;

    _accelgyro.setRate(data);

    // sample_rate = 1000 / (1 + data);
    // mpu_set_compass_sample_rate(min(sample_rate, MAX_COMPASS_SAMPLE_RATE), rate);
}
