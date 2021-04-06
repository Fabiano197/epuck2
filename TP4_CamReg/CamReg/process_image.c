#include "ch.h"
#include "hal.h"
#include <chprintf.h>
#include <usbcfg.h>
#include <math.h>

#include <main.h>
#include <camera/po8030.h>

#include <process_image.h>


static float distance_cm = 0;
static float angle_deg = 0;

static void running_avg(uint8_t* data, uint16_t length, uint8_t avg){
	int sum = *data;
	uint8_t* p1 = data + 1;
	uint8_t* p2 = data;
	for (int i = 1; i < avg; i++) {
		sum = sum + *p1;
		*p1 = sum / (i + 1);
		++p1;
	}

	for (int i = avg; i < length; i++) {
		sum = sum + *p1 - *p2;
		*p1 = sum / avg;
		++p1;
		++p2;
	}
}

static void detect_slope(uint8_t* data, uint16_t length, uint16_t* width, uint16_t* position) {
	uint8_t* ptr_data = data;
	*width = 0;
	uint16_t avg = 0;

	// image correction
	float correct = 0.0;
	float pixel = 0.0;
	for(uint16_t i=0; i<length; i++){
		correct = 180000/(float)(180000-(i-length/2)*(i-length/2));
		pixel = (float)(*ptr_data);
		pixel *= correct;
		*ptr_data = (uint8_t)pixel;
		avg += *ptr_data;
		ptr_data++;
	}
	avg /= length;
	ptr_data = data;

	for(uint16_t i=0; i<length; i++){
		if(*ptr_data < avg/2){
			(*width)++;
		}
		ptr_data++;
	}

	ptr_data = data;
	for(uint16_t i=0; i<length; i++){
		if(*ptr_data < avg/2){
			*position += i;
		}
		ptr_data++;
	}
	*position /= *width;
	return;
}


//semaphore
static BSEMAPHORE_DECL(image_ready_sem, TRUE);

static THD_WORKING_AREA(waCaptureImage, 256);
static THD_FUNCTION(CaptureImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	//Takes pixels 0 to IMAGE_BUFFER_SIZE of the line 10 + 11 (minimum 2 lines because reasons)
	po8030_advanced_config(FORMAT_RGB565, 0, 10, IMAGE_BUFFER_SIZE, 2, SUBSAMPLING_X1, SUBSAMPLING_X1);
	dcmi_enable_double_buffering();
	dcmi_set_capture_mode(CAPTURE_ONE_SHOT);
	dcmi_prepare();

    while(1){
        //starts a capture
		dcmi_capture_start();
		//systime_t time_start = chVTGetSystemTime();
		//waits for the capture to be done
		wait_image_ready();
		//systime_t time_stop = chVTGetSystemTime();

		//signals an image has been captured
		chBSemSignal(&image_ready_sem);
		//chprintf((BaseSequentialStream *)&SDU1, "Time = %i ms \r\n", time_stop-time_start);

    }
}


static THD_WORKING_AREA(waProcessImage, 1024);
static THD_FUNCTION(ProcessImage, arg) {

    chRegSetThreadName(__FUNCTION__);
    (void)arg;

	uint8_t *img_buff_ptr;
	uint8_t image[IMAGE_BUFFER_SIZE] = {0};
	uint16_t width = 0;
	uint16_t position = 0;

    while(1){
    	//waits until an image has been captured
        chBSemWait(&image_ready_sem);
		//gets the pointer to the array filled with the last image in RGB565
		img_buff_ptr = dcmi_get_last_image_ptr();
		for(int i = 0; i < IMAGE_BUFFER_SIZE; i++){
			image[i] = (0xF8 & *img_buff_ptr) >> 3; //read RED pixels
			img_buff_ptr++;
			img_buff_ptr++;
		}
		//running_avg(image, IMAGE_BUFFER_SIZE, 8);

		detect_slope(&image[0], IMAGE_BUFFER_SIZE, &width, &position);

		distance_cm = (float)1500/width;
		if(width < 40){
			distance_cm = -1.0;
		}
		angle_deg = (float)(position-IMAGE_BUFFER_SIZE/2)*2.0*180/(width*distance_cm*3.1415);
		if((angle_deg > 45.0 || angle_deg < -45.0) || (angle_deg < 3.0 && angle_deg > -3.0)){
			angle_deg = 0.0;
		}
		//chprintf((BaseSequentialStream *)&SDU1, "The robot is %f away from the paper at an angle of %f deg.\n\n\r", distance_cm, angle_deg);


		chThdSleepMilliseconds(100);
		SendUint8ToComputer(image, IMAGE_BUFFER_SIZE);
    }
}

float get_distance_cm(void){
	return distance_cm;
}

float get_angle_deg(void){
	return angle_deg;
}

void process_image_start(void){
	chThdCreateStatic(waProcessImage, sizeof(waProcessImage), NORMALPRIO, ProcessImage, NULL);
	chThdCreateStatic(waCaptureImage, sizeof(waCaptureImage), NORMALPRIO, CaptureImage, NULL);
}
