//#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <Arduino.h>
#include <PPMReader.h>
#include <KalmanFilter.h>
#include <PID_regulator.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <NeoSWSerial.h>

Adafruit_MPU6050 mpu;

// RC stuff
#define channumber 6 // Number of channels
float channel_norm[channumber];
int PIN_RECEIVER=3; // rc receiver PPM pin
PPMReader ppm(PIN_RECEIVER, channumber);
bool remote_turned_on=false;
float thrust_rc=0;
float pitch_rc=0.5;
float roll_rc=0.5;
float yaw_rc=0.5;

// time variables
long time_start,time_now,time_elapsed;
float dt;

// GPS stuff
TinyGPS gps;
NeoSWSerial ss(4, 7);
bool newGpsData=false;
unsigned long chars;
unsigned short sentences, failed;
float flat, flon;
unsigned long age;
char c;
double gps_elevation, gps_speed, gps_direction;
bool gps_fix=false;

// set up main data arrays
float acceleration[3]; // m/s^2
float angle_acc[2]; //rads
float angle[3]; // rads
float angle_deg[3]; // degs
float omega[3]; // rad/s
float speed[3] = {0.0f,0.0f,0.0f}; // m/s
float total_acceleration=9.81f;

float offset_acceleration[3]={0.0f,0.0f,0.0f};
float offset_omega[3]={0.0f,0.0f,0.0f};

KalmanFilter filter1,filter2,filter3,filter4,filter5,filter6;

float A_rot=1.0f; //factor of real value to previous real value
float B_rot=0.0f; //factor of real value to real control signal
float H_rot=1.0f; // sprememba merjene vrednosti zaradi enote/drugo
float Q_rot=1.0f; // Process noise (wind/driver input)
float R_rot=10.0f; //sensor inaccuracy. more=more innacurate
float P_rot=0.0f; // zacetni vrednosti
float x_rot=0.0f; // zacetni vrednosti

float A_pos=1.0f; //factor of real value to previous real value
float B_pos=0.0f; //factor of real value to real control signal
float H_pos=1.0f; // sprememba merjene vrednosti zaradi enote/drugo
float Q_pos=5.0f; // Process noise (wind/driver input)
float R_pos=10.0f; //sensor inaccuracy. more=more innacurate
float P_pos=0.0f; // zacetni vrednosti
float x_pos=0.0f; // zacetni vrednosti

PID_regulator pid1,pid2,pid3,pid4,pid5,pid6;

float Kp=5.0f; //translational speed control
float Ki=8.0f;
float Kd=0.0f;
float Kp_r=0.1f; // rotational speed control
float Ki_r=0.0f;
float Kd_r=0.03f;
float Kp5=0.5f; // thrust control
float Ki5=0.2f;
float Kd5=0.0f;
float Kp6=0.05f; // yaw control
float Ki6=0.0f;
float Kd6=0.0f;

float desired_value1;
float output1;
float desired_value2;
float output2;
float desired_value3;
float output3;
float desired_value4;
float output4;
float desired_value5;
float output5;
float desired_value6;
float output6;

sensors_event_t a, g, temp;

float F1m,F2m,F3m,F4m;
int pwm_1,pwm_2,pwm_3,pwm_4;

void setup()
{
	Serial.begin(115200);
	Serial.setTimeout(150);

	while(!Serial) {
		delay(10);
	}
	
	ss.begin(9600); // set baudrate in u-center software, use drivers for GT-U7 (Neo 6M)
	
	Serial.println("Drone is starting up...");

	time_start=micros();
	dt=0.0001;
	time_elapsed=0;

	// IMU stuff
	Serial.println("Adafruit MPU6050 test!");
	mpu.begin();
	delay(100);

	Serial.println("MPU6050 Found!");
	mpu.setAccelerometerRange(MPU6050_RANGE_8_G); // 2,4,8,16
	//mpu.getAccelerometerRange()
	mpu.setGyroRange(MPU6050_RANGE_500_DEG); //250,500,1000,2000
	//mpu.getGyroRange()
	mpu.setFilterBandwidth(MPU6050_BAND_21_HZ); // 5,10,21,44,94,184,260 Hz
	//mpu.getFilterBandwidth()

	// set up kalman filters
	filter1.change_parameters(A_pos,H_pos,Q_pos,R_pos,P_pos,x_pos);
	filter2.change_parameters(A_pos,H_pos,Q_pos,R_pos,P_pos,x_pos);
	filter3.change_parameters(A_pos,H_pos,Q_pos,R_pos,P_pos,x_pos);
	filter4.change_parameters(A_rot,H_rot,Q_rot,R_rot,P_rot,x_rot);
	filter5.change_parameters(A_rot,H_rot,Q_rot,R_rot,P_rot,x_rot);
	filter6.change_parameters(A_rot,H_rot,Q_rot,R_rot,P_rot,x_rot);

	// set up PIDs
	pid1.set_parameters(Kp,Ki,Kd);
	pid2.set_parameters(Kp,Ki,Kd);
	pid3.set_parameters(Kp_r,Ki_r,Kd_r);
	pid4.set_parameters(Kp_r,Ki_r,Kd_r);
	pid5.set_parameters(Kp5,Ki5,Kd5,9.81f/4);
	pid6.set_parameters(Kp6,Ki6,Kd6);

	desired_value1=0;
	desired_value2=0;
	desired_value3=0;
	desired_value4=0;
	desired_value5=0;
	desired_value6=0;

	output1=0;
	output2=0;
	output3=0;
	output4=0;
	output5=0;
	output6=0;

	/*
	Serial.println("Calibrationg IMU...");
	for (int i=0; i<100; i++) {
		mpu.getEvent(&a, &g, &temp);
		delay(10);
	}
	Serial.println("Getting offset...");
	get_offset(0.0f,0.0f,-9.81f);
	Serial.println("Offsets:");
	Serial.print(offset_acceleration[0]);
	Serial.print(",");
	Serial.print(offset_acceleration[1]);
	Serial.print(",");
	Serial.print(offset_acceleration[2]);
	Serial.println(",");
	*/

	offset_acceleration[0]=0.15f;
	offset_acceleration[1]=-0.23f;
	offset_acceleration[2]=-0.92f;

	offset_omega[0]=-0.23f;
	offset_omega[1]=-0.02f;
	offset_omega[2]=-0.01f;

	pinMode(PIN_RECEIVER, INPUT);
	
	for (int i=0;i<channumber;i++) {
		channel_norm[i]=-1.0f;
	}
	
	Serial.println("Initialization complete, starting loop...");
}

void loop() {
	newGpsData=false;
	if(ss.available()) {
		parse_gps_string_and_do_critical_work();
	} else {
		do_critical_work();
		do_non_critical_work();
	}
}

void parse_gps_string_and_do_critical_work() {
	while (ss.available()) {
		if (gps.encode(ss.read())) newGpsData = true;
		do_critical_work();
	}
	if (newGpsData) {
		//get basic data
		gps.f_get_position(&flat, &flon, &age);
		gps_elevation = gps.f_altitude();
		gps_direction = gps.f_course();
		gps_speed = gps.f_speed_mps();

		//get fix info
		if (age == TinyGPS::GPS_INVALID_AGE) {
			//Serial.println("No fix detected");
			gps_fix=false;
		} else if (age > 2000) {
			//Serial.println("Warning: possible stale data!");
			gps_fix=false;
		} else {
			gps_fix=true;
			//Serial.println("Data is current.");
		}
	} // Did a new valid sentence come in?
}

void do_critical_work() {
	time_now=micros();
	dt = (time_now - time_elapsed)*1e-6; //s
	time_elapsed=time_now-time_start;

	//Serial.print("dt");
	//Serial.print(dt,6);
	//Serial.println(",");

	get_imu_data();
	apply_kalman_filters();
	get_rc_data();
	calculate_PIDs();
	//print_pwm_data();
	//print_rot_data();
	print_propeller_thrust_data();
	Serial.println("");

	get_serial_commands();
}

void do_non_critical_work() {
	
	//Serial.print("total_a:");
	//Serial.print(total_acceleration);
	//Serial.print(",");
	//print_omega_data();
	//print_raw_acc_data();
	//print_acc_data();
	//print_rot_data();
	//print_gps_data();
	//print_rc_data();

	Serial.println("");
}

void calculate_PIDs() {

		//angle[0]=0.0f;
		//angle[1]=0.0f;
		angle[2]=0.0f;

		desired_value1=0;
		desired_value2=0;
		desired_value5=0; // sprememba višine
		desired_value6=5*yaw_rc; // kot vrtenja (yaw) [MAX x*yaw_rc °/s]

		//get_inputs();

		// PID 1 velocity levo desno
		//output1 = pid1.Output(speed[2],desired_value1,dt);
		output1=0;

		// PID 2 velocity naprej nazaj
		//output2 = pid2.Output(speed[0],desired_value2,dt);
		output2=0;

		desired_value3=3*(roll_rc-0.5);
		desired_value4=3*(pitch_rc-0.5);

		// PID 3 levo desno roll
		output3 = pid3.Output(angle[0],desired_value3,dt);

		// PID 4 naprej nazaj pitch
		output4 = pid4.Output(angle[1],desired_value4,dt);

		// PID 5 thrust (gor dol)
		//output5 = pid5.Output(speed[1],desired_value5,dt);
		output5=thrust_rc;

		// PID 6 yaw
		//output6 = pid6.Output(r2d(omega[2]),desired_value6,dt);
		output6=0;

		F1m=(+output3-output4 +output5 -output6); // Force magnitude
		F2m=(-output3-output4 +output5 +output6);
		F3m=(+output3+output4 +output5 +output6);
		F4m=(-output3+output4 +output5 -output6);

		if (F1m > 1.0f) F1m=1.0f; else if (F1m < 0.0f) F1m=0.0f;
		if (F2m > 1.0f) F2m=1.0f; else if (F2m < 0.0f) F2m=0.0f;
		if (F3m > 1.0f) F3m=1.0f; else if (F3m < 0.0f) F3m=0.0f;
		if (F4m > 1.0f) F4m=1.0f; else if (F4m < 0.0f) F4m=0.0f;

		pwm_1=(int)(F1m*255.0f);
		pwm_2=(int)(F2m*255.0f);
		pwm_3=(int)(F3m*255.0f);
		pwm_4=(int)(F4m*255.0f);

		apply_pwm_to_propellers();
	}

void get_imu_data() {
	mpu.getEvent(&a, &g, &temp);
	
	omega[0]=(g.gyro.x-offset_omega[0]);
	omega[1]=(g.gyro.y-offset_omega[1]);
	omega[2]=(g.gyro.z-offset_omega[2]);

	if (r2d(omega[2]) >= -0.5f && r2d(omega[2]) <= 0.5f) omega[2]=0;

	acceleration[0]=(a.acceleration.x-offset_acceleration[0]);
	acceleration[1]=(a.acceleration.y-offset_acceleration[1]);
	acceleration[2]=(a.acceleration.z-offset_acceleration[2]);

	angle_acc[0] = atan2(acceleration[1], sqrt(acceleration[2] * acceleration[2] + acceleration[0] * acceleration[0]));
	angle_acc[1] = atan2(acceleration[0], sqrt(acceleration[2] * acceleration[2] + acceleration[1] * acceleration[1]));

	angle[0] = (0.98f * (angle[0] + omega[0] * dt)) + (0.02f * angle_acc[0]);
	angle[1] = (0.98f * (angle[1] + omega[1] * dt)) + (0.02f * angle_acc[1]);
	angle[2] = omega[2]*dt+angle[2];

	angle_deg[0]=wrap(r2d(angle[0]));
	angle_deg[1]=wrap(r2d(angle[1]));
	angle_deg[2]=wrap(r2d(angle[2]));

}

void apply_pwm_to_propellers() {
	analogWrite(5, pwm_1);
	analogWrite(6, pwm_2);
	analogWrite(9, pwm_3);
	analogWrite(10, pwm_4);
}

void get_offset(float init_x, float init_y,float init_z) {
	mpu.getEvent(&a, &g, &temp);
	offset_omega[0]=g.gyro.x;
	offset_omega[1]=g.gyro.y;
	offset_omega[2]=g.gyro.z;
	offset_acceleration[0]=a.acceleration.x-init_x;
	offset_acceleration[1]=a.acceleration.y-init_y;
	offset_acceleration[2]=a.acceleration.z-init_z;
}

void apply_kalman_filters() {
	acceleration[0] = filter1.Output(acceleration[0]);
	acceleration[1] = filter2.Output(acceleration[1]);
	acceleration[2] = filter3.Output(acceleration[2]);

	omega[0] = filter4.Output(omega[0]);
	omega[1] = filter5.Output(omega[1]);
	omega[2] = filter6.Output(omega[2]);
}

void print_omega_data() {
	Serial.print("OmgX:");
	Serial.print(omega[0]);
	Serial.print(",");
	Serial.print("OmgY:");
	Serial.print(omega[1]);
	Serial.print(",");
	Serial.print("OmgZ:");
	Serial.print(omega[2]);
	Serial.print(",");
}

void print_acc_data() {
	Serial.print("AccX:");
	Serial.print(acceleration[0]);
	Serial.print(",");
	Serial.print("AccY:");
	Serial.print(acceleration[1]);
	Serial.print(",");
	Serial.print("AccZ:");
	Serial.print(acceleration[2]);
	Serial.print(",");
}

void print_raw_acc_data() {
	Serial.print("AccRawX:");
	Serial.print(a.acceleration.x);
	Serial.print(",");
	Serial.print("AccRawY:");
	Serial.print(a.acceleration.y);
	Serial.print(",");
	Serial.print("AccRawZ:");
	Serial.print(a.acceleration.z);
	Serial.print(",");
}

void print_rc_data() {
	for (int i=0; i<(channumber); i++) {
		Serial.print("RC");
		Serial.print(i+1);
		Serial.print(":");
		Serial.print(channel_norm[i]);
		Serial.print(",");
	}
	Serial.print("REMOTE:");
	Serial.print(remote_turned_on);
	Serial.print(",");
}

void print_gps_data() {
	Serial.print("LAT:");
    Serial.print(flat);
    Serial.print(",");
    Serial.print("LON:");
    Serial.print(flon);
    Serial.print(",");
    Serial.print("DIR:");
    Serial.print(gps_direction);
    Serial.print(",");
    Serial.print("SPD:");
    Serial.print(gps_speed);
    Serial.print(",");
    //Serial.print("ALT:");
    //Serial.print(gps_elevation);
    //Serial.print(",");
    Serial.print("FIX:");
    Serial.print(gps_fix);
    Serial.print(",");

}

void print_rot_data() {
	Serial.print("AngX:");
	Serial.print(angle_deg[0]);
	Serial.print(",");
	Serial.print("AngY:");
	Serial.print(angle_deg[1]);
	Serial.print(",");
	Serial.print("AngZ:");
	Serial.print(angle_deg[2]);
	Serial.print(",");
}

void print_pwm_data() {
	Serial.print("pwm_1:");
	Serial.print(pwm_1);
	Serial.print(",");
	Serial.print("pwm_2:");
	Serial.print(pwm_2);
	Serial.print(",");
	Serial.print("pwm_3:");
	Serial.print(pwm_3);
	Serial.print(",");
	Serial.print("pwm_4:");
	Serial.print(pwm_4);
	Serial.print(",");
}

void print_propeller_thrust_data() {
	Serial.print("F1m:");
	Serial.print(F1m);
	Serial.print(",");
	Serial.print("F2m:");
	Serial.print(F2m);
	Serial.print(",");
	Serial.print("F3m:");
	Serial.print(F3m);
	Serial.print(",");
	Serial.print("F4m:");
	Serial.print(F4m);
	Serial.print(",");
}

void get_rc_data() {
	for (int channel = 1; channel <= channumber; ++channel) {
        unsigned long value = ppm.latestValidChannelValue(channel, 0);
        channel_norm[channel-1] = ((float)value-1000.0f)/10.0f/100.0f;
    }

    if (!remote_turned_on) {
    	if (channel_norm[0] != -1.0f) {
    		remote_turned_on = true;
    	}
    }

    thrust_rc=channel_norm[2];
    pitch_rc=channel_norm[1];
    roll_rc=channel_norm[0];
    yaw_rc=channel_norm[3];
}

float r2d(float degrees) {
	return (degrees * 57.2958f);
}

void get_serial_commands() {
	if (Serial.available() > 0) {
		String incoming = Serial.readString();
		if (incoming == "reset") {
			get_offset(0.0f,0.0f,-9.81f);
		}
	}
}

static float wrap(float angle)
{
	while (angle > +180) angle -= 360;
	while (angle < -180) angle += 360;
	return angle;
}