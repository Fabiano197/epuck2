#include <main.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <i2c_bus.h>

#include "ch.h"
#include "chprintf.h"
#include "hal.h"
#include "shell.h"
#include "cmd.h"
#include "memory_protection.h"
#include "communication.h"
#include "spi_comm.h"
#include "usbcfg.h"

//Custom Includes
#include "measurements.h"
#include "control.h"
#include "ekf.h"
#include "communications.h"
#include "landmarks.h"


messagebus_t bus;


int main(void)
{
	//System Initializations
    halInit();
    chSysInit();
    mpu_init();


    // Init custom libraries.
    communications_init();
    ekf_init();
    //measurements_start();

    for(uint16_t i = 0; i < 50; i++){
        landmark_t test = {100-i, 2*i, 100-10*i};
        find_landmark(test);
    }

    /* Infinite loop. */
    while (1) {

    }

}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void)
{
    chSysHalt("Stack smashing detected");
}
