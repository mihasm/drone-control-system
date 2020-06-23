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

#define channumber 6 // Number of channels

float channel_norm[channumber];

int PIN_RECEIVER=3;

long time_start,time_now,time_elapsed;
float dt;

PPMReader ppm(PIN_RECEIVER, channumber);


// GPS stuff
TinyGPS gps;
NeoSWSerial ss(4, 7);
bool newGpsData=false;
unsigned long chars;
unsigned short sentences, failed;
float flat, flon;
unsigned long age;
char c;

// set up main data arrays
float acceleration[3]; // m/s^2
float angle_acc[2];
float angle[3];
float omega[3]; // deg
float speed[3] = {0.0f,0.0f,0.0f}; // m/s

float offset_acceleration[3]={0.0f,0.0f,0.0f};
float offset_omega[3]={0.0f,0.0f,0.0f};

bool need_to_reset_offset=true;

KalmanFilter filter1,filter2,filter3,filter4,filter5,filter6;

float A_rot=1.0f; //factor of real value to previous real value
float B_rot=0.0f; //factor of real value to real control signal
float H_rot=1.0f; // sprememba merjene vrednosti zaradi enote/drugo
float Q_rot=1.0f; // Process noise (wind/driver input)
float R_rot=50.0f; //sensor inaccuracy. more=more innacurate
float P_rot=0.0f; // zacetni vrednosti
float x_rot=0.0f; // zacetni vrednosti

float A_pos=1.0f; //factor of real value to previous real value
float B_pos=0.0f; //factor of real value to real control signal
float H_pos=1.0f; // sprememba merjene vrednosti zaradi enote/drugo
float Q_pos=1.0f; // Process noise (wind/driver input)
float R_pos=50.0f; //sensor inaccuracy. more=more innacurate
float P_pos=0.0f; // zacetni vrednosti
float x_pos=0.0f; // zacetni vrednosti

PID_regulator pid1,pid2,pid3,pid4,pid5,pid6;

float Kp=5.0f; //translational speed control
float Ki=8.0f;
float Kd=0.0f;
float Kp_r=0.005f; // rotational speed control
float Ki_r=0.0f;
float Kd_r=0.003f;
float Kp5=0.5f; // thrust control
float Ki5=0.2f;
float Kd5=0.0f;
float Kp6=0.05f; // yaw control
float Ki6=0.4f;
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

void setup()
{
	Serial.begin(115200);
	Serial.setTimeout(50);
	
	ss.begin(9600); // set baudrate in u-center software, use drivers for GT-U7 (Neo 6M)
	
	Serial.println("Drone is starting up...");

	time_start=micros();
	dt=0.0001;
	time_elapsed=0;

	pinMode(PIN_RECEIVER, INPUT);

	// IMU stuff
	Serial.println("Adafruit MPU6050 test!");
	// Try to initialize!
	if (!mpu.begin()) {
		Serial.println("Failed to find MPU6050 chip");
		while (1) {
			delay(10);
		}
	}
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


	Serial.println("Initialization complete, starting loop...");
}

void loop() {
	newGpsData = false;

	if (ss.available()) {
		while (ss.available()) {
			c = ss.read();
			//Serial.write(c); // uncomment this line if you want to see the GPS data flowing
			do_work();
		}
		if (gps.encode(c)) newGpsData = true; // Did a new valid sentence come in?
	} else {
		do_work();
	}
}

void do_work() {

	time_now=micros();
	dt = (time_now - time_elapsed)*1e-6; //s
	Serial.print("dt");
	Serial.print(dt,6);
	Serial.print(",");
	time_elapsed=time_now-time_start;

	get_imu_data();
	get_rc_data();
	apply_kalman_filters();

	speed[0]=dt*acceleration[0]+speed[0];
	speed[1]=dt*acceleration[1]+speed[1];
	speed[2]=dt*acceleration[2]+speed[2];

	print_omega_data();
	//print_raw_acc_data();
	print_acc_data();
	print_rot_data();
	//print_gps_data();
	//print_vel_data();
	//print_rc_data();
	Serial.println("");

	get_serial_commands();
}

void calculate_PIDs() {

		desired_value1=0;
		desired_value2=0;
		desired_value5=0; // sprememba višine
		desired_value6=0; // kot vrtenja

		//get_inputs();

		// PID 1 velocity levo desno
		output1 = pid1.Output(speed[2],desired_value1,dt);

		// PID 2 velocity naprej nazaj
		output2 = pid2.Output(speed[0],desired_value2,dt);

		desired_value3=output1;
		desired_value4=output2;

		// PID 3 levo desno roll
		output3 = pid3.Output(omega[0],desired_value3,dt);

		// PID 4 naprej nazaj pitch
		output4 = pid4.Output(-omega[2],desired_value4,dt);

		// PID 5 thrust (gor dol)
		output5 = pid5.Output(speed[1],desired_value5,dt);

		// PID 6 yaw
		output6 = pid6.Output(omega[1],desired_value6,dt);

		//F1m=(-output3-output4+output5-output6); // Force magnitude
		//F2m=(+output3-output4+output5+output6);
		//F3m=(-output3+output4+output5+output6);
		//F4m=(+output3+output4+output5-output6);
	}

void get_imu_data() {
	mpu.getEvent(&a, &g, &temp);
	
	omega[0]=r2d(g.gyro.x-offset_omega[0]);
	omega[1]=r2d(g.gyro.y-offset_omega[1]);
	omega[2]=r2d(g.gyro.z-offset_omega[2]);

	if (omega[2] >= -0.5f && omega[2] <=0.5f) omega[2]=0;

	acceleration[0]=(a.acceleration.x-offset_acceleration[0]);
	acceleration[1]=(a.acceleration.y-offset_acceleration[1]);
	acceleration[2]=(a.acceleration.z-offset_acceleration[2]);

	angle_acc[0] = r2d(atan2(acceleration[1], sqrt(acceleration[2] * acceleration[2] + acceleration[0] * acceleration[0])));
	angle_acc[1] = r2d(atan2(acceleration[0], sqrt(acceleration[2] * acceleration[2] + acceleration[1] * acceleration[1])));

	angle[0] = (0.98f * (angle[0] + omega[0] * dt)) + (0.02f * angle_acc[0]);
	angle[1] = (0.98f * (angle[1] + omega[1] * dt)) + (0.02f * angle_acc[1]);
	angle[2] = omega[2]*dt+angle[2];

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

void print_vel_data() {
	Serial.print("VelX:");
	Serial.print(speed[0]);
	Serial.print(",");
	Serial.print("VelY:");
	Serial.print(speed[1]);
	Serial.print(",");
	Serial.print("VelZ:");
	Serial.print(speed[2]);
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
}

void print_gps_data() {
	Serial.print("LAT=");
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Serial.print(" LON=");
    Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    Serial.print(" SAT=");
    Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
    Serial.print(" PREC=");
    Serial.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
    Serial.println("");
    gps.stats(&chars, &sentences, &failed);
	if (chars == 0) Serial.println("** No characters received from GPS: check wiring **");
}

void print_rot_data() {
	Serial.print("AngX:");
	Serial.print(angle[0]);
	Serial.print(",");
	Serial.print("AngY:");
	Serial.print(angle[1]);
	Serial.print(",");
	Serial.print("AngZ:");
	Serial.print(angle[2]);
	Serial.print(",");
}

void get_rc_data() {
	for (int channel = 1; channel <= channumber; ++channel) {
        unsigned long value = ppm.latestValidChannelValue(channel, 0);
        channel_norm[channel-1] = ((float)value-1000.0f)/10.0f;
    }
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