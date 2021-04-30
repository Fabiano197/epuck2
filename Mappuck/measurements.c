#include <ch.h>
#include <hal.h>
#include <main.h>
#include <math.h>
#include <msgbus/messagebus.h>

#include "sensors/mpu9250.h"
#include "sensors/imu.h"
#include "sensors/proximity.h"
#include "sensors/VL53L0X/VL53L0X.h"

#include "measurements.h"

extern messagebus_t bus;

static measurements_msg_t measurements_values = {0,0,0,0,0};

static thread_t *measurementsThd;
static bool measurements_configured = false;

static uint16_t get_tof_distance(void){
	return VL53L0X_get_dist_mm()-52 ; //remove offset
}

static uint16_t get_proximity_distance(uint8_t sensor){
	uint16_t dist = (uint16_t)(sqrt(200000/get_calibrated_prox(sensor))); //empirical formula to convert from IR intensity to distance
	if(dist == 0) dist = 255;
	return dist;
}

static float get_inclination(void){
	float acc_y = get_acc_filtered(1, 3);
	float acc_z = get_acc_filtered(2, 3);
	return atan(-acc_y/acc_z);
}

static THD_WORKING_AREA(waMeasurements, 512);
static THD_FUNCTION(measurements_thd, arg) {
     (void) arg;
     chRegSetThreadName(__FUNCTION__);

	 messagebus_topic_t measurements_topic;
	 MUTEX_DECL(measurements_topic_lock);
	 CONDVAR_DECL(measurements_topic_condvar);
	 messagebus_topic_init(&measurements_topic, &measurements_topic_lock, &measurements_topic_condvar, &measurements_values, sizeof(measurements_values));
	 messagebus_advertise_topic(&bus, &measurements_topic, "/measurements");

     chThdSleepMilliseconds(2000);

	 while(chThdShouldTerminateX() == false){
		 measurements_values.tof_distance_front = get_tof_distance();
		 measurements_values.proximity_distance_northeast = get_proximity_distance(PROXIMITY_NORTHEAST);
		 measurements_values.proximity_distance_east = get_proximity_distance(PROXIMITY_EAST);
	     measurements_values.proximity_distance_southeast = get_proximity_distance(PROXIMITY_SOUTHEAST);
	     measurements_values.inclination = get_inclination();

	     messagebus_topic_publish(&measurements_topic, &measurements_values, sizeof(measurements_values));
	 }
}


/****************************PUBLIC FUNCTIONS*************************************/

void measurements_start(void){
	if(measurements_configured)return;
	proximity_start();
	imu_start();
	VL53L0X_start();
	calibrate_ir();

	measurementsThd = chThdCreateStatic(waMeasurements, sizeof(waMeasurements), NORMALPRIO, measurements_thd, NULL);
	measurements_configured = true;
}


void measurements_stop(void){
	imu_stop();
	VL53L0X_stop();
	if(!measurements_configured)return;
	measurements_configured = false;
	chThdTerminate(measurementsThd);
}
