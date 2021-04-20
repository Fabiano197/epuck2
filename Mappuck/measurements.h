#ifndef MEASUREMENT_H
#define MEASUREMENT_H

#include <hal.h>

//Measurements needed for feedback
typedef struct {
	int16_t tof_distance_front; //mm
	int16_t proximity_distance_right; //mm
	float inclination; //rad
} measurements_msg_t;


/**
* @brief   Initializes sensors used for feedback (time of flight, proximity and IMU)
*          Broadcasts a measurements_msg_t message on the /measurements topic
*/
void measurements_start(void);


/**
* @brief   Stops all sensors
*
*/
void measurements_stop(void);

/**
* @brief   returns the distance in the front
*
*/
int16_t get_tof_distance(void);

/**
* @brief   returns the distance on the right of the epuck
*
*/
int16_t get_proximity_distance(void);

/**
* @brief   returns the inclination
*
*/
float get_inclination(void);

#endif
