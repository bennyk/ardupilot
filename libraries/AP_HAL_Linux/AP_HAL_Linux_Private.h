
#ifndef __AP_HAL_LINUX_PRIVATE_H__
#define __AP_HAL_LINUX_PRIVATE_H__

/* Umbrella header for all private headers of the AP_HAL_Linux module.
 * Only import this header from inside AP_HAL_Linux
 */

#include "UARTDriver.h"
#include "I2CDriver.h"
#include "SPIDriver.h"
#include "AnalogIn.h"
#include "Storage.h"
#include "GPIO.h"
#include "RCInput.h"

#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_MBMD
#include "RCOutput_I2C.h"
#endif

#include "RCInput_Navio.h"
#include "RCOutput_PRU.h"
#include "RCOutput_Navio.h"
#include "RCOutput_ZYNQ.h"
#include "Semaphores.h"
#include "Scheduler.h"
#include "ToneAlarmDriver.h"
#include "Util.h"

#endif // __AP_HAL_LINUX_PRIVATE_H__

