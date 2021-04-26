#ifndef EKF_H
#define EKF_H

#include <ch.h>
#include <hal.h>
#include <math.h>


#include "landmarks.h"

typedef struct {
	int16_t x; //mm
	int16_t y; //mm
	int16_t z; //mm
	float phi; //rad polar angle compromised between -pi and pi
	float theta; //rad azimuthal angle compromised between 0 and pi

} position_t;

typedef struct {
	float angle;
	uint16_t dist;
} controll_command_t;

/**
* @brief Initializes EFK
*/
void ekf_init(void);

/**
* @brief Stops EFK
*/
void ekf_stop(void);

/**
* @brief Returns covariance matrix TODO specify datatype for that
*/
void get_covariance_matrix(void);

/**
* @brief Returns estimated position of robot
*/
position_t* get_position(void);


#endif
