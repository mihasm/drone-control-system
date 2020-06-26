#include "KalmanFilter.h"

KalmanFilter::KalmanFilter() {
}

float KalmanFilter::Output(float input) {
    // do a prediction
    x = A * x;
    P = A * P * A + Q;

    // calculate the Kalman gain
    K = P * H / (H * P * H + R);

    // correct
    x = x + K * (input - H * x);
    P = (1 - K * H) * P;

    return x;
}

void KalmanFilter::change_parameters(float A, float H, float Q, float R, float initial_P, float initial_x) {
    this->A = A;
    this->H = H;
    this->Q = Q;
    this->R = R;
    this->P = initial_P;
    this->x = initial_x;
}
