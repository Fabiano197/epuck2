#include <ch.h>
#include <hal.h>
#include <main.h>

#include "communications.h"

static THD_WORKING_AREA(get_measurements, 512);
static THD_FUNCTION(imu_reader_thd, arg) {
	(void) arg;
	     chRegSetThreadName(__FUNCTION__);
	//TODO Establish Bluetooth communication with computer and send map information on a regular basis
}
