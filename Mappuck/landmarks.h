#ifndef LANDMARKS_H
#define LANDMARKS_H

#include <ch.h>
#include <hal.h>
#include <main.h>
#include <math.h>

#define TOF INT16_MIN
#define IR  INT16_MAX
#define PROXIMITYRADIUS 50 //Proxmity Radius in mm
#define NB_LANDMARK_MAX 1500

//Landmarks
typedef struct {
	int16_t x; //mm
	int16_t y; //mm
	int16_t z; //mm, if z = TOF or IR -> landmark is wall

} landmark_t;

/**
* @brief Either returns the index i of the closest landmark (if coordinate lies in PROXMITIYRADIUS of an already existing landmark).
* 		 For landmark localization, only x and y coordinates are used
* 		 Otherwise, a new landmark is created with the coordinates provided
*/
int16_t find_landmark(landmark_t coordinates);

/**
* @brief Returns the ith landmark (if it exists), otherwise the function returns landmark with x, y, z = WALL
*/
landmark_t get_landmark(int16_t i);

/**
* @brief Returns pointer to first landmark
*/
landmark_t* get_landmark_ptr(void);

/**
* @brief Returns number of landmarks currently stored
*/
uint16_t get_nb_landmarks(void);

#endif
