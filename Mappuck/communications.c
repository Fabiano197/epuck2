#include <ch.h>
#include <hal.h>
#include <main.h>
#include "usbcfg.h"
#include "chprintf.h"

#include "communications.h"
#include "ekf.h"

static thread_t *comThd;
static bool com_configured = false;

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
	while(chThdShouldTerminateX() == false){
		send_data_Bluetooth();
		chThdSleepMilliseconds(2000);
	}
}

/****************************PUBLIC FUNCTIONS*************************************/

void communications_init(void){
	if(com_configured)return;
	static SerialConfig ser_cfg = {
	    115200,
	    0,
	    0,
	    0,
	};
	sdStart(&SD3, &ser_cfg); // UART3.

	comThd = chThdCreateStatic(waCommunication, sizeof(waCommunication), NORMALPRIO+1, communication_thd, NULL);
	com_configured = true;
}

void communication_stop(void){
	if(!com_configured)return;
	com_configured = false;
	chThdTerminate(comThd);
}
