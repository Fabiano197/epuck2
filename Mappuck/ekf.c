#include "ekf.h"

static position_t pos = {0,0,0,0};


void ekf_init(void){
	pos = (position_t){32, 46, 20, 5.13};
}

position_t* get_position(void){
	return &pos;
}
