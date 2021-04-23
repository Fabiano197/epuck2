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
	float angle; //angle in rad compromised between -pi and pi

} position_t;

/**
* @brief Initializes the Extended Kalman Filter
*/
void ekf_init(void);

/**
* @brief Returns covariance matrix TODO specify datatype for that
*/
void get_covariance_matrix(void);

/**
* @brief Returns estimated position of robot
*/
position_t* get_position(void);


#endif
