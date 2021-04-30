#ifndef CONTROL_H
#define CONTROL_H

#include <hal.h>
#include <math.h>
#include "measurements.h"
#include "ekf.h"
#include "motors.h"

/*typedef struct {
	float angle;
	uint16_t dist;
} control_command_t;*/

#define PI 3.141565

/**
* @brief Init control
*/
void control_init(void);

/**
* @brief Stops control
*/
void control_stop(void);

/**
* @brief Turns angle radians and moves distance mm
*/
void make_step(control_command_t u);

/**
* @brief Returns weather motors are running
*/
bool motor_is_running(void);

#endif
