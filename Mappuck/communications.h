#ifndef COMMUNICATIONS_H
#define COMMUNICATIONS_H

#include <hal.h>
#include "landmarks.h"
#include "measurements.h"
#include "usbcfg.h"
#include "chprintf.h"

typedef struct pos_landmarks_msg{
	int16_t number_lm;				//number of landmarks
	Position_t current_pos;	//current position of the robot
	landmark_t* landmarks;	//pointer to the first landmark
} pos_landmarks_msg_t;

/**
* @brief Starts Bluetooth communication with computer
*/
void communication_init(void);


/**
 * @brief sends the given position via USB cable to the computer
 */
void print_position_USB(Position_t pos);


/**
 * @brief sends the collected data to the computer via bluetooth
 * @detail sends the number of landmarks, the current position of the robot and the positions of the landmarks.
 */
void send_data_Bluetooth(pos_landmarks_msg_t message);

#endif
