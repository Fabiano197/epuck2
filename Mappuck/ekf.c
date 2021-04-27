#include "ekf.h"
#include "measurements.h"
#include "control.h"
#include "landmarks.h"
#include <main.h>
#include <msgbus/messagebus.h>

#define WALLDISTANCE 50
#define EPUCK_RADIUS 35

static position_t pos = {0,0,0,0,0};

static thread_t *efkThd;
static bool efk_configured = false;

static control_command_t u = {0, 10};
static control_command_t error = {0, 0};
static measurements_msg_t measurements_values;

MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

void calculate_u(void){
	error.angle = (measurements_values.proximity_distance_east-WALLDISTANCE) + 4*(measurements_values.proximity_distance_northeast-WALLDISTANCE*3/2);
	error.angle /= 1500;
	if(measurements_values.tof_distance_front<50){
		error.angle = PI/100*(measurements_values.tof_distance_front-50);
	}
	u.angle = error.angle;
}

void estimate_pos(void){
	pos.phi += u.angle;
	if(pos.phi > PI){
		pos.phi-=2*PI;
	}
	else if(pos.phi < PI){
		pos.phi+=2*PI;
	}
	pos.x = pos.x + u.dist*cos(pos.phi);
	pos.y = pos.y + u.dist*sin(pos.phi);
}

void set_landmarks(void){
	landmark_t l;
	l.x = pos.x + (measurements_values.tof_distance_front + EPUCK_RADIUS)*cos(pos.phi);
	l.y = pos.y + (measurements_values.tof_distance_front + EPUCK_RADIUS)*sin(pos.phi);
	l.z = WALL;
	find_landmark(l);
	l.x = pos.x + (measurements_values.proximity_distance_east+EPUCK_RADIUS)*cos(pos.phi+PI/2);
	l.y = pos.y + (measurements_values.proximity_distance_east+EPUCK_RADIUS)*sin(pos.phi+PI/2);
	find_landmark(l);
	l.x = pos.x;
	l.y = pos.y;
	l.z = pos.z;
	find_landmark(l);
}

static THD_WORKING_AREA(waEfk, 1024);
static THD_FUNCTION(efk_thd, arg) {
     (void) arg;
     chRegSetThreadName(__FUNCTION__);

     messagebus_init(&bus, &bus_lock, &bus_condvar);

     messagebus_topic_t *measurements_topic = messagebus_find_topic_blocking(&bus, "/measurements");

     chThdSleepMilliseconds(5000);

	 while(chThdShouldTerminateX() == false){
		 while(motor_is_running())chThdSleepMilliseconds(10);
		 messagebus_topic_wait(measurements_topic, &measurements_values, sizeof(measurements_values));
		 calculate_u();
		 make_step(u);
		 set_landmarks();
		 estimate_pos();
		 /*pos.x = measurements_values.proximity_distance_northeast;
		 pos.y = measurements_values.proximity_distance_east;
		 pos.z = measurements_values.tof_distance_front;
		 pos.phi = measurements_values.inclination;*/

	 }
}

/****************************PUBLIC FUNCTIONS*************************************/


void ekf_init(void){
	if(efk_configured)return;
	efkThd = chThdCreateStatic(waEfk, sizeof(waEfk), NORMALPRIO+2, efk_thd, NULL);
	efk_configured = true;
}

void efk_stop(void){
	if(!efk_configured)return;
	efk_configured = false;
	chThdTerminate(efkThd);
}

position_t* get_position(void){
	return &pos;
}
