#ifndef EKF_H
#define EKF_H

#include "landmarks.h"

/**
* @brief Initializes the Extended Kalman Filter
*/
void ekf_init(void);

/**
* @brief Returns covariance matrix TODO specify datatype for that
*/
void get_covariance_matrix(void);

#endif
