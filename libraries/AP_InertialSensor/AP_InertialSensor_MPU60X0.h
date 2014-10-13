
#ifndef __AP_INERTIAL_SENSOR_MPU60X0__
#define __AP_INERTIAL_SENSOR_MPU60X0__

#include "AP_InertialSensor.h"

#include "MPU6050.h"

class AP_InertialSensor_MPU60X0 : public AP_InertialSensor
{
public:
	AP_InertialSensor_MPU60X0();

public:
    bool            update();
    float        	get_delta_time() const;
    float           get_gyro_drift_rate();
    bool            wait_for_sample(uint16_t timeout_ms);

private:
    uint16_t _init_sensor( Sample_rate sample_rate );
    void _accumulate();
    bool _sample_available();
    void _set_filter_frequency(uint8_t filter_hz);

    void mpu_set_lpf(uint16_t lpf);
    void mpu_set_sample_rate(uint16_t rate);

private:
    MPU6050 _accelgyro;

    Vector3f _accel_filtered;
    Vector3f _gyro_filtered;
    uint32_t _sample_period_usec;
    volatile uint32_t _gyro_samples_available;
    uint64_t _last_sample_timestamp;
    bool _have_sample_available;

    // support for updating filter at runtime
    uint8_t _last_filter_hz;
    uint8_t _default_filter_hz;

    // Low Pass filters for gyro and accel
    LowPassFilter2p _accel_filter_x;
    LowPassFilter2p _accel_filter_y;
    LowPassFilter2p _accel_filter_z;
    LowPassFilter2p _gyro_filter_x;
    LowPassFilter2p _gyro_filter_y;
    LowPassFilter2p _gyro_filter_z;

};

#endif
