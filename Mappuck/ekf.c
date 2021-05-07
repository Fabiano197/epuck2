#include "ekf.h"
#include "measurements.h"
#include "control.h"
#include "landmarks.h"
#include <main.h>
#include <msgbus/messagebus.h>

#define WALL_DISTANCE 50
#define EPUCK_RADIUS 35
#define WHEEL_FULLDIST_STEP 400.0
#define TICK_IN_MM 0.13

static position_t pos = {0,0,0,0,0};

static thread_t *efkThd;
static bool efk_configured = false;

static control_command_t u = {0, 10};
static control_command_t error = {0, 0};
static measurements_msg_t measurements_values;

MUTEX_DECL(bus_lock);
CONDVAR_DECL(bus_condvar);

static void calculate_u(void){
	error.angle = (measurements_values.proximity_distance_east-WALL_DISTANCE) + 4*(measurements_values.proximity_distance_northeast-WALL_DISTANCE*3/2);
	error.angle /= 1300;
	if(measurements_values.tof_distance_front<70){
		error.angle = PI/100*(measurements_values.tof_distance_front-70);
	}
	u.angle = error.angle;
}

static void estimate_pos(void){
	static int32_t old_pos_left;
	static int32_t old_pos_right;
	int32_t new_pos_left  = left_motor_get_pos();
	int32_t new_pos_right = right_motor_get_pos();
	float dist_steps = (float)(new_pos_left + new_pos_right - old_pos_left - old_pos_right)/2;

	pos.phi -= (float)(new_pos_left-old_pos_left - new_pos_right+old_pos_right)/WHEEL_FULLDIST_STEP;
	if(pos.phi > PI)pos.phi -= 2*PI;
	else if(pos.phi <= -PI)pos.phi += 2*PI;
	pos.x += dist_steps*cos(pos.phi);
	pos.y += dist_steps*sin(pos.phi);
	pos.z += dist_steps*sin(pos.theta);
	pos.theta = measurements_values.inclination;

	old_pos_left  = new_pos_left;
	old_pos_right = new_pos_right;

	return;
}

static void set_landmarks(void){
	landmark_t l;
	/*l.x = pos.x + (measurements_values.tof_distance_front + EPUCK_RADIUS)*cos(pos.phi);
	l.y = pos.y + (measurements_values.tof_distance_front + EPUCK_RADIUS)*sin(pos.phi);
	l.z = TOF;
	if(measurements_values.tof_distance_front <= 300){
		find_landmark(l);
	}*/
	if(measurements_values.proximity_distance_east <= 100){
		l.x = pos.x + (measurements_values.proximity_distance_east+EPUCK_RADIUS)*cos(pos.phi-PI/2);
		l.y = pos.y + (measurements_values.proximity_distance_east+EPUCK_RADIUS)*sin(pos.phi-PI/2);
		l.z = IR;
		enter_landmark(l);
	}
	l.x = pos.x;
	l.y = pos.y;
	l.z = pos.z;
	enter_landmark(l);
}

static THD_WORKING_AREA(waEfk, 1024);
static THD_FUNCTION(efk_thd, arg) {
     (void) arg;
     chRegSetThreadName(__FUNCTION__);

     messagebus_init(&bus, &bus_lock, &bus_condvar);

     messagebus_topic_t *measurements_topic = messagebus_find_topic_blocking(&bus, "/measurements");

     chThdSleepMilliseconds(5000);

	 while(chThdShouldTerminateX() == false){
		 while(motor_is_running()) chThdSleepMilliseconds(10);
		 messagebus_topic_wait(measurements_topic, &measurements_values, sizeof(measurements_values));
		 calculate_u();
		 make_step(u);
		 estimate_pos();
		 set_landmarks();

		 //For measurements debugging only
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
