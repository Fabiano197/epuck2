#include "landmarks.h"

static uint16_t N = 0;
static landmark_t landmarks[NB_LANDMARK_MAX];



/****************************PUBLIC FUNCTIONS*************************************/

int16_t find_landmark(landmark_t coordinates){
	landmarks[N] = coordinates;
	N++;
	return N;
}


landmark_t* get_landmark_ptr(void){
	return &landmarks[0];
}


uint16_t get_nb_landmarks(void){
	return N;
}
