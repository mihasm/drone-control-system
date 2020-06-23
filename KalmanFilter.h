class KalmanFilter {

    public:
    float A, H, Q, R, P, x, K;

    KalmanFilter();
    float Output(float input);
    void change_parameters(float A, float H, float Q, float R, float initial_P, float initial_x);

};