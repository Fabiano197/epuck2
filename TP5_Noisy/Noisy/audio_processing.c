#include "ch.h"
#include "hal.h"
#include <main.h>
#include <usbcfg.h>
#include <chprintf.h>
#include <chbsem.h>

#include <motors.h>
#include <audio/microphone.h>
#include <audio_processing.h>
#include <communications.h>
#include <fft.h>
#include <arm_math.h>

//semaphore
static BSEMAPHORE_DECL(sendToComputer_sem, TRUE); // @suppress("Field cannot be resolved")

//2 times FFT_SIZE because these arrays contain complex numbers (real + imaginary)
static float micLeft_cmplx_input[2 * FFT_SIZE];
static float micRight_cmplx_input[2 * FFT_SIZE];
static float micFront_cmplx_input[2 * FFT_SIZE];
static float micBack_cmplx_input[2 * FFT_SIZE];
//Arrays containing the computed magnitude of the complex numbers
static float micLeft_output[FFT_SIZE];
static float micRight_output[FFT_SIZE];
static float micFront_output[FFT_SIZE];
static float micBack_output[FFT_SIZE];

static float* micRight_ptr = &micRight_cmplx_input[0];
static float* micLeft_ptr = &micLeft_cmplx_input[0];
static float* micBack_ptr = &micBack_cmplx_input[0];
static float* micFront_ptr = &micFront_cmplx_input[0];

/*
*	Callback called when the demodulation of the four microphones is done.
*	We get 160 samples per mic every 10ms (16kHz)
*	
*	params :
*	int16_t *data			Buffer containing 4 times 160 samples. the samples are sorted by micro
*							so we have [micRight1, micLeft1, micBack1, micFront1, micRight2, etc...]
*	uint16_t num_samples	Tells how many data we get in total (should always be 640)
*/
void processAudioData(int16_t* data, uint16_t num_samples){
	static uint8_t  count = 10;

	for(int i = 0; i < num_samples/4; i++){
		if(micLeft_ptr == &micLeft_cmplx_input[2*FFT_SIZE]){
			micRight_ptr = &micRight_cmplx_input[0];
			micLeft_ptr = &micLeft_cmplx_input[0];
			micBack_ptr = &micBack_cmplx_input[0];
			micFront_ptr = &micFront_cmplx_input[0];

			doFFT_optimized(FFT_SIZE, micRight_ptr);
			doFFT_optimized(FFT_SIZE, micLeft_ptr);
			doFFT_optimized(FFT_SIZE, micBack_ptr);
			doFFT_optimized(FFT_SIZE, micFront_ptr);

			arm_cmplx_mag_f32(micRight_ptr, micRight_output, FFT_SIZE);
			arm_cmplx_mag_f32(micLeft_ptr, micLeft_output, FFT_SIZE);
			arm_cmplx_mag_f32(micBack_ptr, micBack_output, FFT_SIZE);
			arm_cmplx_mag_f32(micFront_ptr, micFront_output, FFT_SIZE);

			if(!count) {
				chBSemSignal(&sendToComputer_sem);
				count = 10;
			}
			else count--;
			command_motor();
			break;
		}
		*micRight_ptr = *data;
		micRight_ptr++;
		*micRight_ptr = 0;
		micRight_ptr++;
		data++;

		*micLeft_ptr = *data;
		micLeft_ptr++;
		*micLeft_ptr = 0;
		micLeft_ptr++;
		data++;

		*micBack_ptr = *data;
		micBack_ptr++;
		*micBack_ptr = 0;
		micBack_ptr++;
		data++;

		*micFront_ptr = *data;
		micFront_ptr++;
		*micFront_ptr = 0;
		micFront_ptr++;
		data++;
	}
}

void command_motor(void){
	uint16_t peak = 0;
	for(int i = 0; i < FFT_SIZE; i++){
		if(micLeft_output[peak]< micLeft_output[i]) peak = i;
	}
	left_motor_set_speed(0);
	right_motor_set_speed(0);
	if(micLeft_output[peak] < 20000)return;
	float frequency = 8000.0/512.0*(512-abs(peak-512));
	//chprintf((BaseSequentialStream *)&SDU1, "%Frequency =%.2f \r\n", frequency);
	if(frequency >500 && frequency <1500){
		left_motor_set_speed(1000);
		right_motor_set_speed(1000);
	}
	else if(frequency >1500 && frequency <2500){
		left_motor_set_speed(-1000);
		right_motor_set_speed(-1000);
	}
	else if(frequency >2500 && frequency <3500){
		left_motor_set_speed(-1000);
		right_motor_set_speed(1000);
	}
	else if(frequency >3500 && frequency <4500){
		left_motor_set_speed(1000);
		right_motor_set_speed(-1000);
	}
}


void wait_send_to_computer(void){
	chBSemWait(&sendToComputer_sem);
}

float* get_audio_buffer_ptr(BUFFER_NAME_t name){
	if(name == LEFT_CMPLX_INPUT){
		return micLeft_cmplx_input;
	}
	else if (name == RIGHT_CMPLX_INPUT){
		return micRight_cmplx_input;
	}
	else if (name == FRONT_CMPLX_INPUT){
		return micFront_cmplx_input;
	}
	else if (name == BACK_CMPLX_INPUT){
		return micBack_cmplx_input;
	}
	else if (name == LEFT_OUTPUT){
		return micLeft_output;
	}
	else if (name == RIGHT_OUTPUT){
		return micRight_output;
	}
	else if (name == FRONT_OUTPUT){
		return micFront_output;
	}
	else if (name == BACK_OUTPUT){
		return micBack_output;
	}
	else{
		return NULL;
	}
}
