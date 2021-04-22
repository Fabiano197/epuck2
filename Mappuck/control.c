#include "control.h"

#define PI    3.1415927
#define TWOPI 6.2831853
#define SAMPLE_PERIODE 5 // [ms]
#define PRINT_PERIODE 500 // [ms]

static int32_t old_pos_left = 0;
static int32_t old_pos_right = 0;
static Position_t current_pos = {0,0,0,0};

void control_init(void){
	motors_init();
}

static int16_t sinus(int16_t value)
{
	return value - value*value*value/6 + value*value*value*value*value/120;
}

static int16_t cosinus(int16_t value)
{
	return 1 - value*value/4 + value*value*value*value/24;
}

static float arctan(int16_t value)
{
	return (float)(value - value*value*value/3 + value*value*value*value*value/5);
}

static Position_t make_step(Position_t old_pos)
{
	int32_t new_pos_left  = left_motor_get_pos();
	int32_t new_pos_right = right_motor_get_pos();
	int32_t dist_steps = (new_pos_left + new_pos_right - old_pos_left - old_pos_right)/2;

	Position_t new_pos;
	new_pos.x = old_pos.x + dist_steps*cosinus(old_pos.theta);
	new_pos.y = old_pos.y + dist_steps*sinus(old_pos.theta);
	new_pos.z = old_pos.z;
	new_pos.theta = old_pos.theta + arctan((float)(new_pos_right-old_pos_right-new_pos_left+old_pos_left)/WHEEL_FULLDIST_STEP);
	if(new_pos.theta >= TWOPI) {new_pos.theta -= TWOPI;}

	old_pos_left  = new_pos_left;
	old_pos_right = new_pos_right;

	return new_pos;
}
