#ifndef LANDMARKS_H
#define LANDMARKS_H

#include <ch.h>
#include <hal.h>
#include <main.h>
#include <math.h>

#define TOF INT16_MIN
#define IR  INT16_MAX
#define PROXIMITY_RADIUS 50 //Proxmity Radius [mm]
#define NB_WALL_LANDMARK_MAX 300
#define NB_SURFACE_LANDMARK_MAX 1000
#define NB_CORNERS_MAX 100
#define MAX_CORRELATION_ERROR 15 //Maximal accaptable correlation error for line fitting before new corner is created [mm]
#define CLOSE_LOOP_RADIUS 75 //Radius for which wall loop will be closed

//Landmarks
typedef struct {
	int16_t x; //mm
	int16_t y; //mm
	int16_t z; //mm

} landmark_t;

typedef struct {
	int16_t x; //mm
	int16_t y; //mm

} wall_t;

typedef struct {
  float alpha;
  float beta;
} line_t;

/**
* @brief Enter new landmark into system, function returns true if loop is closed
*/
bool enter_landmark(landmark_t coordinates);

/**
* @brief Returns the ith landmark (if it exists), otherwise the function returns landmark with x, y, z = WALL
*/
landmark_t get_surface_landmark(int16_t i);

/**
* @brief Returns pointer to first landmark
*/
landmark_t* get_surface_landmark_ptr(void);

/**
* @brief Returns number of landmarks currently stored
*/
uint16_t get_nb_surface_landmarks(void);

uint16_t get_nb_surface_landmarks_to_send(void);

#endif
