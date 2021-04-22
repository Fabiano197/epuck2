#include <math.h>
#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "positioning.h"
#include "motors.h"
#include "usbcfg.h"

#define PI    3.1415927
#define TWOPI 6.2831853
#define SAMPLE_PERIODE 10 // [ms]
#define PRINT_PERIODE 2000 // [ms]

static thread_t* positioningThd;
static bool positioning_active = false;

static int32_t old_pos_left;
static int32_t old_pos_right;
static Position_t current_pos = {0,0,0,0};

static void print_position_USB(Position_t pos)
{
    chprintf((BaseSequentialStream *)&SDU1, "Position: x= %f\t y= %f\t z= %f\t theta= %f \n\r", pos.x, pos.y, pos.z, pos.theta);
    chprintf((BaseSequentialStream *)&SDU1, "Position Left  Motor: %i \n\r", old_pos_left);
    chprintf((BaseSequentialStream *)&SDU1, "Position Right Motor: %i \n\n\r", old_pos_right);
}

static void print_position_BlueTooth(void)
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"BlaBla", 6);
	//chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&size, sizeof(uint16_t));
	//chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)data, size);
}

static Position_t make_step(Position_t old_pos)
{
	int32_t new_pos_left  = left_motor_get_pos();
	int32_t new_pos_right = right_motor_get_pos();
	int32_t dist_steps = (new_pos_left + new_pos_right - old_pos_left - old_pos_right)/2;

	Position_t new_pos;
	new_pos.x = old_pos.x + dist_steps*cos(old_pos.theta);
	new_pos.y = old_pos.y + dist_steps*sin(old_pos.theta);
	new_pos.z = old_pos.z;
	new_pos.theta = old_pos.theta + atan((float)(new_pos_right-old_pos_right-new_pos_left+old_pos_left)/WHEEL_FULLDIST_STEP);
	if(new_pos.theta >= TWOPI) {new_pos.theta -= TWOPI;}

	old_pos_left  = new_pos_left;
	old_pos_right = new_pos_right;

	return new_pos;
}

static THD_WORKING_AREA(waPOSITIONINGThd, 2048);
static THD_FUNCTION(POSITIONINGThd, arg)
{
	(void)arg;
	chRegSetThreadName("Positioning thread");
	positioning_active = true;

	uint16_t nb_samples = 0;
	uint16_t nb_samp_print = PRINT_PERIODE/SAMPLE_PERIODE;
	while(1){
		if(positioning_active){
			current_pos = make_step(current_pos);
			if(++nb_samples == nb_samp_print){
				print_position_BlueTooth();
				nb_samples = 0;
			}
		}
		chThdSleepMilliseconds(SAMPLE_PERIODE);
	}
}

void positioning_start(void)
{
	if(positioning_active){
		return;
	}
	positioningThd = chThdCreateStatic(waPOSITIONINGThd,
	                     sizeof(waPOSITIONINGThd),
	                     NORMALPRIO + 2,
						 POSITIONINGThd,
	                     NULL);
}

void positioning_stop(void)
{
    chThdTerminate(positioningThd);
    chThdWait(positioningThd);
    positioningThd = NULL;
    positioning_active = false;
}

Position_t get_pos(void)
{
	return current_pos;
}
