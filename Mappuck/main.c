#include <main.h>
#include <mapping.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <motor_control.h>
#include "memory_protection.h"


//Custom Includes
#include "measurements.h"
#include "communications.h"
#include "landmarks.h"
#include "sensors/VL53L0X/VL53L0X.h"

messagebus_t bus;


int main(void)
{
	//System Initializations
    halInit();
    chSysInit();
    mpu_init();


    // Init custom libraries.
    mapping_init();
    measurements_start();
    motor_control_init();
    communications_init();
    /* Infinite loop. */
    while (1) {
    }

}

#define STACK_CHK_GUARD 0xe2dee396
uintptr_t __stack_chk_guard = STACK_CHK_GUARD;

void __stack_chk_fail(void){
    chSysHalt("Stack smashing detected");
}
