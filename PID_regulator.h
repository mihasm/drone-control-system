class PID_regulator {

	public:
	float error_prior; // PID
	float integral_prior;
	float error;
	float actual_value;
	float integral;
	float derivative;
	float output;
	float bias;
	float Kp;
	float Ki;
	float Kd;

	PID_regulator();
	void set_parameters(float Kp, float Ki, float Kd, float bias=0.0f);
	float Output(float input, float desired_value, float dt, bool stop_integration=false);
	void ResetOutput();
	void ResetID();
};