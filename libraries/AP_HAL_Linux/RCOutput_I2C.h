
#ifndef __AP_HAL_LINUX_RCOUTPUT_I2C_H__
#define __AP_HAL_LINUX_RCOUTPUT_I2C_H__

#include <AP_HAL_Linux.h>

#define PCA9685_DEFAULT_ADDRESS     0x40 // All address pins low, Navio default

#define PCA9685_RA_MODE1            0x00
#define PCA9685_RA_MODE2            0x01
#define PCA9685_RA_LED0_ON_L        0x06
#define PCA9685_RA_LED0_ON_H        0x07
#define PCA9685_RA_LED0_OFF_L       0x08
#define PCA9685_RA_LED0_OFF_H       0x09
#define PCA9685_RA_ALL_LED_ON_L     0xFA
#define PCA9685_RA_ALL_LED_ON_H     0xFB
#define PCA9685_RA_ALL_LED_OFF_L    0xFC
#define PCA9685_RA_ALL_LED_OFF_H    0xFD
#define PCA9685_RA_PRE_SCALE        0xFE

#define PCA9685_MODE1_RESTART_BIT   7  // 0x40
#define PCA9685_MODE1_EXTCLK_BIT    6  // 0x20
#define PCA9685_MODE1_AI_BIT        5  // 0x10
#define PCA9685_MODE1_SLEEP_BIT     4  // 0x8
#define PCA9685_MODE1_SUB1_BIT      3  // 0x4
#define PCA9685_MODE1_SUB2_BIT      2  // 0x2
#define PCA9685_MODE1_SUB3_BIT      1  // 0x1
#define PCA9685_MODE1_ALLCALL_BIT   0  // 0x0

#define PCA9685_MODE2_INVRT_BIT     4  // 0x8
#define PCA9685_MODE2_OCH_BIT       3  // 0x4
#define PCA9685_MODE2_OUTDRV_BIT    2  // 0x2
#define PCA9685_MODE2_OUTNE1_BIT    1  // 0x1
#define PCA9685_MODE2_OUTNE0_BIT    0  // 0x0

#define LED0_ON_L       0x06    // LED0 output and brightness control byte 0
#define LED0_ON_H       0x07    // LED0 output and brightness control byte 1
#define LED0_OFF_L      0x08    // LED0 output and brightness control byte 2
#define LED0_OFF_H      0x09    // LED0 output and brightness control byte 3
#define LED1_ON_L       0x0A    // LED1 output and brightness control byte 0
#define LED1_ON_H       0x0B    // LED1 output and brightness control byte 1
#define LED1_OFF_L      0x0C    // LED1 output and brightness control byte 2
#define LED1_OFF_H      0x0D    // LED1 output and brightness control byte 3
#define LED2_ON_L       0x0E    // LED2 output and brightness control byte 0
#define LED2_ON_H       0x0F    // LED2 output and brightness control byte 1
#define LED2_OFF_L      0x10    // LED2 output and brightness control byte 2
#define LED2_OFF_H      0x11    // LED2 output and brightness control byte 3
#define LED3_ON_L       0x12    // LED3 output and brightness control byte 0
#define LED3_ON_H       0x13    // LED3 output and brightness control byte 1
#define LED3_OFF_L      0x14    // LED3 output and brightness control byte 2
#define LED3_OFF_H      0x15    // LED3 output and brightness control byte 3
#define LED4_ON_L       0x16    // LED4 output and brightness control byte 0
#define LED4_ON_H       0x17    // LED4 output and brightness control byte 1
#define LED4_OFF_L      0x18    // LED4 output and brightness control byte 2
#define LED4_OFF_H      0x19    // LED4 output and brightness control byte 3
#define LED5_ON_L       0x1A    // LED5 output and brightness control byte 0
#define LED5_ON_H       0x1B    // LED5 output and brightness control byte 1
#define LED5_OFF_L      0x1C    // LED5 output and brightness control byte 2
#define LED5_OFF_H      0x1D    // LED5 output and brightness control byte 3
#define LED6_ON_L       0x1E    // LED6 output and brightness control byte 0
#define LED6_ON_H       0x1F    // LED6 output and brightness control byte 1
#define LED6_OFF_L      0x20    // LED6 output and brightness control byte 2
#define LED6_OFF_H      0x21    // LED6 output and brightness control byte 3
#define LED7_ON_L       0x22    // LED7 output and brightness control byte 0
#define LED7_ON_H       0x23    // LED7 output and brightness control byte 1
#define LED7_OFF_L      0x24    // LED7 output and brightness control byte 2
#define LED7_OFF_H      0x25    // LED7 output and brightness control byte 3
#define LED8_ON_L       0x26    // LED8 output and brightness control byte 0
#define LED8_ON_H       0x27    // LED8 output and brightness control byte 1
#define LED8_OFF_L      0x28    // LED8 output and brightness control byte 2
#define LED8_OFF_H      0x29    // LED8 output and brightness control byte 3
#define LED9_ON_L       0x2A    // LED9 output and brightness control byte 0
#define LED9_ON_H       0x2B    // LED9 output and brightness control byte 1
#define LED9_OFF_L      0x2C    // LED9 output and brightness control byte 2
#define LED9_OFF_H      0x2D    // LED9 output and brightness control byte 3
#define LED10_ON_L      0x2E    // LED10 output and brightness control byte 0
#define LED10_ON_H      0x2F    // LED10 output and brightness control byte 1
#define LED10_OFF_L     0x30    // LED10 output and brightness control byte 2
#define LED10_OFF_H     0x31    // LED10 output and brightness control byte 3
#define LED11_ON_L      0x32    // LED11 output and brightness control byte 0
#define LED11_ON_H      0x33    // LED11 output and brightness control byte 1
#define LED11_OFF_L     0x34    // LED11 output and brightness control byte 2
#define LED11_OFF_H     0x35    // LED11 output and brightness control byte 3
#define LED12_ON_L      0x36    // LED12 output and brightness control byte 0
#define LED12_ON_H      0x37    // LED12 output and brightness control byte 1
#define LED12_OFF_L     0x38    // LED12 output and brightness control byte 2
#define LED12_OFF_H     0x39    // LED12 output and brightness control byte 3
#define LED13_ON_L      0x3A    // LED13 output and brightness control byte 0
#define LED13_ON_H      0x3B    // LED13 output and brightness control byte 1
#define LED13_OFF_L     0x3C    // LED13 output and brightness control byte 2
#define LED13_OFF_H     0x3D    // LED13 output and brightness control byte 3
#define LED14_ON_L      0x3E    // LED14 output and brightness control byte 0
#define LED14_ON_H      0x3F    // LED14 output and brightness control byte 1
#define LED14_OFF_L     0x40    // LED14 output and brightness control byte 2
#define LED14_OFF_H     0x41    // LED14 output and brightness control byte 3
#define LED15_ON_L      0x42    // LED15 output and brightness control byte 0
#define LED15_ON_H      0x43    // LED15 output and brightness control byte 1
#define LED15_OFF_L     0x44    // LED15 output and brightness control byte 2
#define LED15_OFF_H     0x45    // LED15 output and brightness control byte 3
#define ALL_LED_ON_L    0xFA    // load all the LEDn_ON registers, byte 0 (read zero)
#define ALL_LED_ON_H    0xFB    // load all the LEDn_ON registers, byte 1 (read zero)
#define ALL_LED_OFF_L   0xFC    // load all the LEDn_OFF registers, byte 0 (read zero)
#define ALL_LED_OFF_H   0xFD    // load all the LEDn_OFF registers, byte 1 (read zero)
#define PCA9685_PRE_SCALE       0xFE    // prescaler for output frequency
#define PCA9685_TestMode        0xFF    // defines the test mode to be entered

class Linux::LinuxRCOutput_I2C : public AP_HAL::RCOutput {
public:
	LinuxRCOutput_I2C();

public:
    virtual void     init(void* implspecific);
    virtual void     set_freq(uint32_t chmask, uint16_t freq_hz);
    virtual uint16_t get_freq(uint8_t ch);
    virtual void     enable_ch(uint8_t ch);
    virtual void     disable_ch(uint8_t ch);
    virtual void     write(uint8_t ch, uint16_t period_us);
    virtual void     write(uint8_t ch, uint16_t* period_us, uint8_t len);
    virtual uint16_t read(uint8_t ch);
    virtual void     read(uint16_t* period_us, uint8_t len);

    virtual void     set_failsafe_pwm(uint32_t chmask, uint16_t period_us);

private:
    static uint8_t writeBit(uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data);
    void setPWM(uint8_t channel, uint16_t offset, uint16_t length);
    void setPWM(uint8_t channel, uint16_t length);
    void restart();
    void sleep();
    float getFrequency();

private:
    uint8_t _devAddr;
    float _frequency;
};

#endif // __AP_HAL_LINUX_RCOUTPUT_I2C_H__
