class PID_regulator {

	public:
	float error_prior1; // PID
	float integral_prior1;
	float error1;
	float actual_value1;
	float integral1;
	float derivative1;
	float output1;
	float bias1=0.0f;
	float Kp=5.0f;
	float Ki=8.0f;
	float Kd=0.0f;

	PID_regulator();
	void set_parameters(float Kp, float Ki, float Kd, float bias1=0.0f);
	float Output(float input, float desired_value1, float dt);

};