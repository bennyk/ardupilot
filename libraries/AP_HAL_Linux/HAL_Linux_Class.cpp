#include <AP_HAL.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX

#include "HAL_Linux_Class.h"
#include "AP_HAL_Linux_Private.h"
#include "i2cbusses.h"

#include <getopt.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>

#include <AP_HAL_Empty.h>
#include <AP_HAL_Empty_Private.h>

extern const AP_HAL::HAL& hal;

using namespace Linux;

// 3 serial ports on Linux for now
static LinuxUARTDriver uartADriver(true);
static LinuxUARTDriver uartBDriver(false);
static LinuxUARTDriver uartCDriver(false);

static LinuxSemaphore  i2cSemaphore;
static LinuxI2CDriver  i2cDriver(&i2cSemaphore, "/dev/i2c-1");
static LinuxSPIDeviceManager spiDeviceManager;
static LinuxAnalogIn analogIn;
static LinuxStorage storageDriver;
static LinuxGPIO gpioDriver;

/*
  use the PRU based RCInput driver on ERLE and PXF
 */
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXF || CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ERLE
static LinuxRCInput_PRU rcinDriver;
#else
static LinuxRCInput rcinDriver;
#endif

/*
  use the PRU based RCOutput driver on ERLE and PXF
 */
#if CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXF || CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ERLE
static LinuxRCOutput_PRU rcoutDriver;
#elif CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_MBMD
static LinuxRCOutput_I2C rcoutDriver;
#else
static Empty::EmptyRCOutput rcoutDriver;
#endif
static LinuxScheduler schedulerInstance;
static LinuxUtil utilInstance;

HAL_Linux::HAL_Linux() :
    AP_HAL::HAL(
        &uartADriver,
        &uartBDriver,
        &uartCDriver,
        NULL,            /* no uartD */
        NULL,            /* no uartE */
        &i2cDriver,
        &spiDeviceManager,
        &analogIn,
        &storageDriver,
        &uartADriver,
        &gpioDriver,
        &rcinDriver,
        &rcoutDriver,
        &schedulerInstance,
        &utilInstance)
{}

void _usage(void)
{
    printf("Usage: -A uartAPath -B uartBPath -C uartCPath\n");
    printf("Options:\n");
    printf("\t-serial:          -A /dev/ttyO4\n");
    printf("\t                  -B /dev/ttyS1\n");    
    printf("\t-tcp:             -C tcp:192.168.2.15:1243:wait\n");
    printf("\t                  -A tcp:11.0.0.2:5678\n");    
}

void HAL_Linux::init(int argc,char* const argv[]) const
{
    int opt;
    /*
      parse command line options
     */
    while ((opt = getopt(argc, argv, "A:B:C:h")) != -1) {
        switch (opt) {
        case 'A':
            uartADriver.set_device_path(optarg);
            break;
        case 'B':
            uartBDriver.set_device_path(optarg);
            break;
        case 'C':
            uartCDriver.set_device_path(optarg);
            break;
        case 'h':
            _usage();
            exit(0);
        default:
            printf("Unknown option '%c'\n", (char)opt);
            exit(1);
        }
    }

    scheduler->init(NULL);
    gpio->init();
    rcout->init(NULL);
    rcin->init(NULL);
    uartA->begin(115200);
    i2c->begin();
    spi->init(NULL);
    utilInstance.init(argc, argv);
}

void HAL_Linux::scan_i2cbus()
{
	struct i2c_adap *adapters;
	int count;
	
	hal.console->println("scanning i2c bus ...");

	adapters = gather_i2c_busses();
	if (adapters == NULL) {
		fprintf(stderr, "Error: Out of memory!\n");
		return;
	}

	bool found = false;
	char dev[30];

	// search for i2c adapter with the name "i2c-designware-pci"
	for (count = 0; adapters[count].name; count++) {

		printf("i2c-%d\t%-10s\t%-32s\t%s\n",
			adapters[count].nr, adapters[count].funcs,
			adapters[count].name, adapters[count].algo);

		if (strcmp(adapters[count].name, "i2c-designware-pci") != 0) {
			continue;
		}

		sprintf(dev, "/dev/i2c-%d", adapters[count].nr);

		LinuxI2CDriver *i2cdev = new LinuxI2CDriver(&i2cSemaphore, dev);
		i2cdev->begin();


		// 0x68 - test the compass device address
		uint8_t byte;
		if (i2cdev->readRegister(0x77, 0, &byte) == 0) {
			// good
			hal.console->printf("i2c bus %d is good!\n", adapters[count].nr);
			i2c = i2cdev;
			found = true;
			break;
		}

		// close and free the memory
		i2cdev->end();
		delete i2cdev;
	}

	if (!found) {
		hal.scheduler->panic("Failed to locate an active i2c bus!");
	}
}


const HAL_Linux AP_HAL_Linux;

#endif
