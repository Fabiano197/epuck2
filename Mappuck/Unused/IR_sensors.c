#include "sensors/proximity.h"
#include "motors.h"

int prox_right;
int dist;
int error;
int K = 4;
int norm_speed = 200;

right_motor_set_speed(norm_speed);
while (1) {
	prox_right = 0;
	dist = 0;
	for(uint8_t i=0; i<10; i++){
		chThdSleepMilliseconds(20);
		prox_right += get_prox(2);
	}
	prox_right /= 10;
	if(prox_right == 0){
		prox_right = 1;
	}
	dist = sqrt(50000/prox_right);
	error = dist - 40;
	if(fabs(error) < 2){
		error = 0;
	}
	if(error > 30){
		error = 30;
	}
	if(error < -30){
		error = -30;
	}
	left_motor_set_speed(norm_speed - K*error);
	right_motor_set_speed(norm_speed + K*error);
	chThdSleepMilliseconds(20);
	left_motor_set_speed(norm_speed + K*error);
	right_motor_set_speed(norm_speed - K*error);
	chThdSleepMilliseconds(20);

	//chprintf((BaseSequentialStream *)&SDU1, "Prox: %i \t Dist: %f mm \n\r", prox_right, dist);
}
