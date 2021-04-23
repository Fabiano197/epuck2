#include <ch.h>
#include <hal.h>
#include <main.h>
#include "usbcfg.h"
#include "chprintf.h"

#include "communications.h"
#include "ekf.h"

void send_data_Bluetooth(void){
	uint16_t N = get_nb_landmarks();
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)"START", 5);
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)&N, sizeof(uint16_t));
	chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)get_position(), sizeof(position_t));
	landmark_t *landmark_ptr = get_landmark_ptr();
	for(uint16_t i = 0; i < N; i++){
		chSequentialStreamWrite((BaseSequentialStream *)&SD3, (uint8_t*)landmark_ptr, sizeof(landmark_t));
		landmark_ptr++;
	}
	return;
}

static THD_WORKING_AREA(waCommunication, 512);
static THD_FUNCTION(communication_thd, arg) {
	chRegSetThreadName(__FUNCTION__);
	(void) arg;
	while(1){
		send_data_Bluetooth();
		chThdSleepMilliseconds(1000);
	}
}

/****************************PUBLIC FUNCTIONS*************************************/

void communications_init(void){
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};
	sdStart(&SD3, &ser_cfg); // UART3.
	chThdCreateStatic(waCommunication, sizeof(waCommunication), NORMALPRIO, communication_thd, NULL);
}
