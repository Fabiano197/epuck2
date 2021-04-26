#include <ch.h>
#include <hal.h>
#include <main.h>
#include <math.h>

#include "sensors/imu.h"
#include "sensors/mpu9250.h"
#include "sensors/proximity.h"
#include "sensors/VL53L0X/VL53L0X.h"

#include "measurements.h"

static measurements_msg_t measurements_values;

static THD_WORKING_AREA(get_measurements, 512);
static THD_FUNCTION(get_measurements_thd, arg) {
     (void) arg;
     chRegSetThreadName(__FUNCTION__);

     //To be completed

     // To be adjusted to measurements (ref to imu.c)
	 /*messagebus_topic_t imu_topic;
	 MUTEX_DECL(imu_topic_lock);
	 CONDVAR_DECL(imu_topic_condvar);
	 messagebus_topic_init(&imu_topic, &imu_topic_lock, &imu_topic_condvar, &imu_values, sizeof(imu_values));
	 messagebus_advertise_topic(&bus, &imu_topic, "/imu");*/
}


/****************************PUBLIC FUNCTIONS*************************************/

void measurements_start(void){
	proximity_start();
	imu_start();
	VL53L0X_start();
	//To be completed
}


void measurements_stop(void){
	imu_stop();
	VL53L0X_stop();
	//To be completed
}

uint16_t get_tof_distance(void){
	return VL53L0X_get_dist_mm();
}

uint16_t get_proximity_distance(void){
	int prox = 0;
	for(uint8_t i=0; i<10; i++){
		prox += get_prox(2);
		chThdSleepMilliseconds(20);
	}
	prox /= 10;
	if(prox == 0){
		prox = 1;
	}
	return (uint16_t)(sqrt(50000/prox)); //empirical formula to convert from IR intensity to distance
}

float get_inclination(void){
	float acc_x = get_acceleration(0);
	float acc_y = get_acceleration(1);
	float acc_z = get_acceleration(2);
	float acc_plane = sqrt(acc_x*acc_x + acc_y*acc_y);
	return atan(acc_plane/acc_z);
}
