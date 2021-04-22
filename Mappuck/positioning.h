
#ifndef PROJECT_POSITIONING_H_
#define PROJECT_POSITIONING_H_

#include <stdint.h>
#include <hal.h>

typedef struct Position{
	float x;
	float y;
	float z;
	float theta;
} Position_t;

// starts the positioning thread
void positioning_start(void);

// stops the positioning thread
void positioning_stop(void);

// returns the current position of the robot
Position_t get_pos(void);

#endif /* PROJECT_POSITIONING_H_ */
