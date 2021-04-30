#include "landmarks.h"

static uint16_t N = 0;
static uint16_t last_landmark_sent = 0;
static landmark_t landmarks[NB_LANDMARK_MAX];


/****************************PUBLIC FUNCTIONS*************************************/

int16_t find_landmark(landmark_t coordinates){
	landmarks[N] = coordinates;
	N++;
	return -1;
}

landmark_t get_landmark(int16_t i){
	return landmarks[i];
}

landmark_t* get_landmark_ptr(void){
	return &landmarks[last_landmark_sent+1];
}

uint16_t get_nb_landmarks_to_send(void){
	uint16_t diff = N-last_landmark_sent;
	last_landmark_sent = N;
	return diff;
}

uint16_t get_nb_landmarks(void){
	return N;
}

landmark_t* get_next_landmark(uint16_t last_sent){
	if(last_sent == N){
		return 0;
	}
	else{
		return &landmarks[last_sent+1];
	}
}
