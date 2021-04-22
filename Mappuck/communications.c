#include <ch.h>
#include <hal.h>
#include <main.h>

#include "communications.h"

static THD_WORKING_AREA(get_measurements, 512);
static THD_FUNCTION(imu_reader_thd, arg) {
	(void) arg;
	     chRegSetThreadName(__FUNCTION__);
	//TODO Establish Bluetooth communication with computer and send map information on a regular basis
}

void print_position_USB(Position_t pos)
{
    chprintf((BaseSequentialStream *)&SDU1, "Position: x= %i\t y= %i\t z= %i\t theta= %f \n\r", pos.x, pos.y, pos.z, pos.theta);
}

void send_data_Bluetooth(pos_landmarks_msg_t message)
{
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&message.number_lm, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&message.current_pos, sizeof(Position_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)message.landmarks, sizeof(landmark_t)*message.number_lm );
}
