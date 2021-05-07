#include <KalmanFilter.h>
#include <LPF.h>

class TripleFilter {

    public:
    KalmanFilter KF1, KF2;
    LPF LPF1;
    // Needed for Kalman filters
    //float Q,R,x_init;

    // Needed for this filter
    float kf1_out, lpf_out, kf2_out;

    // Needed for the LPF
    //float fc;

    TripleFilter();
    void change_parameters(float Q, float R, float fc, float x_init);
    float Output(float x, float Ts);

};