class KalmanFilter {

    public:
    float Q, R, P, x, K;
    float A=1;
    float H=1;

    KalmanFilter();
    float Output(float input);
    void change_parameters(float Q, float R, float initial_x);

};