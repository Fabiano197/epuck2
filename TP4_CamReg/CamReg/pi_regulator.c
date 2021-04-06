#include "ch.h"
#include "hal.h"
#include <math.h>
#include <usbcfg.h>
#include <chprintf.h>


#include <main.h>
#include <motors.h>
#include <pi_regulator.h>
#include <process_image.h>

static THD_WORKING_AREA(waPiRegulator, 256);
static THD_FUNCTION(PiRegulator, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;
    float Kp_dist = -100.0;
    float Kp_rot = -15.0;
    float target_dist = 15.0;

    systime_t time;

    int16_t speed = 0;
    int16_t rotation = 0;

    while(1){
        time = chVTGetSystemTime();
        float current_dist  = get_distance_cm();
        float current_angle = get_angle_deg();
        if(current_dist < 0){
        	current_dist = target_dist;
        }
        float e = target_dist - current_dist;
        if(e < 0.5 && e > -0.5){
        	e = 0.0;
        }
        speed = Kp_dist * e;
        rotation = Kp_rot * current_angle;
        //chprintf((BaseSequentialStream *)&SDU1, "Rotation:   %f.\n\n\r", current_angle);
        
		right_motor_set_speed(speed+rotation);
		left_motor_set_speed (speed-rotation);

        //100Hz
        chThdSleepUntilWindowed(time, time + MS2ST(10));
    }
}

void pi_regulator_start(void){
	chThdCreateStatic(waPiRegulator, sizeof(waPiRegulator), NORMALPRIO, PiRegulator, NULL);
}
