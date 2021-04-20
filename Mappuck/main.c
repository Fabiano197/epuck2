#include <main.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "ch.h"
#include "chprintf.h"
#include "hal.h"
#include "shell.h"

#include "cmd.h"
#include "memory_protection.h"
#include "communication.h"

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


    // Init the peripherals.
    measurements_start();


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
