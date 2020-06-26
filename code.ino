// #include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <Arduino.h>
#include <PPMReader.h>
#include <KalmanFilter.h>
#include <PID_regulator.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <NeoSWSerial.h>
#include <SoftwareWire.h>

// Magnetometer stuff
SoftwareWire myWire(10, 8);

#define address 0x0D

#define Mode_Standby    0b00000000
#define Mode_Continuous 0b00000001

#define ODR_10Hz        0b00000000
#define ODR_50Hz        0b00000100
#define ODR_100Hz       0b00001000
#define ODR_200Hz       0b00001100

#define RNG_2G          0b00000000
#define RNG_8G          0b00010000

#define OSR_512         0b00000000
#define OSR_256         0b01000000
#define OSR_128         0b10000000
#define OSR_64          0b11000000

int mag_x, mag_y, mag_z, mag_x_new, mag_y_new, mag_z_new;
float mag_azimuth;

// MPU stuff
Adafruit_MPU6050 mpu;

// RC stuff
#define kChannelNumber 6  // Number of channels
float channel_norm[kChannelNumber];
int PIN_RECEIVER = 3;  // rc receiver PPM pin
PPMReader ppm(PIN_RECEIVER, kChannelNumber);
bool remote_turned_on = false;
float thrust_rc = 0;
float pitch_rc = 0.5;
float roll_rc = 0.5;
float yaw_rc = 0.5;
unsigned long value;

// time variables
long time_start, time_now, time_prev, time_elapsed;
float dt, timer_magnetometer;

// GPS stuff
TinyGPS gps;
NeoSWSerial ss(4, 7);
bool newGpsData = false;
unsigned long chars;
unsigned short sentences, failed;
float flat, flon;
unsigned long age;
char c;
double gps_elevation, gps_speed, gps_direction;
bool gps_fix = false;

// set up main data arrays
float acceleration[3];  // m/s^2
float angle_acc[2];  // rads
float angle[3];  // rads
float angle_deg[3];  // degs
float omega[3];  // rad/s
float speed[3] = {0.0f, 0.0f, 0.0f};  // m/s
float total_acceleration = 9.81f;

float offset_acceleration[3] = {0.0f, 0.0f, 0.0f};
float offset_omega[3] = {0.0f, 0.0f, 0.0f};

KalmanFilter filter1, filter2, filter3, filter4,
             filter5, filter6, filter7, filter8;

float A_rot = 1.0f;  // factor of real value to previous real value
float B_rot = 0.0f;  // factor of real value to real control signal
float H_rot = 1.0f;  // sprememba merjene vrednosti zaradi enote/drugo
float Q_rot = 1.0f;  // Process noise (wind/driver input)
float R_rot = 10.0f;  // sensor inaccuracy. more = more innacurate
float P_rot = 0.0f;  // zacetni vrednosti
float x_rot = 0.0f;  // zacetni vrednosti

float A_pos = 1.0f;  // factor of real value to previous real value
float B_pos = 0.0f;  // factor of real value to real control signal
float H_pos = 1.0f;  // sprememba merjene vrednosti zaradi enote/drugo
float Q_pos = 5.0f;  // Process noise (wind/driver input)
float R_pos = 10.0f;  // sensor inaccuracy. more = more innacurate
float P_pos = 0.0f;  // zacetni vrednosti
float x_pos = 0.0f;  // zacetni vrednosti

float A_yaw = 1.0f;  // factor of real value to previous real value
float B_yaw = 0.0f;  // factor of real value to real control signal
float H_yaw = 1.0f;  // sprememba merjene vrednosti zaradi enote/drugo
float Q_yaw = 0.05f;  // Process noise (wind/driver input)
float R_yaw = 10.0f;  // sensor inaccuracy. more = more innacurate
float P_yaw = 0.0f;  // zacetni vrednosti
float x_yaw = 0.0f;  // zacetni vrednosti

float A_mag_rot_vel = 1.0f;  // factor of real value to previous real value
float B_mag_rot_vel = 0.0f;  // factor of real value to real control signal
float H_mag_rot_vel = 1.0f;  // sprememba merjene vrednosti zaradi enote/drugo
float Q_mag_rot_vel = 1.0f;  // Process noise (wind/driver input)
float R_mag_rot_vel = 100.0f;  // sensor inaccuracy. more = more innacurate
float P_mag_rot_vel = 0.0f;  // zacetni vrednosti
float x_mag_rot_vel = 0.0f;  // zacetni vrednosti

PID_regulator pid1, pid2, pid3, pid4, pid5, pid6;

float Kp = 5.0f;  // translational speed control
float Ki = 8.0f;
float Kd = 0.0f;
float Kp_r = 0.1f;  // rotational speed control
float Ki_r = 0.0f;
float Kd_r = 0.001f;
float Kp5 = 0.5f;  // thrust control
float Ki5 = 0.2f;
float Kd5 = 0.0f;
float Kp6 = 0.01f;  // yaw control
float Ki6 = 0.0f;
float Kd6 = 0.0f;

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

float F1m, F2m, F3m, F4m;
int pwm_1, pwm_2, pwm_3, pwm_4;

void setup() {
    Serial.begin(74880);
    Serial.setTimeout(150);

    ss.begin(9600);  // set baudrate in u-center software, use drivers for GT-U7 (Neo 6M)
    delay(100);
    Serial.write("Starting drone!\n");

    time_start = micros();
    dt = 0.0001f;
    time_elapsed = 0;

    // IMU stuff
    mpu.begin();

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);  // 2,4,8,16
    // mpu.getAccelerometerRange()
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);  // 250,500,1000,2000
    // mpu.getGyroRange()
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);  // 5,10,21,44,94,184,260 Hz
    // mpu.getFilterBandwidth()

    // set up kalman filters
    filter1.change_parameters(A_pos, H_pos, Q_pos, R_pos, P_pos, x_pos);
    filter2.change_parameters(A_pos, H_pos, Q_pos, R_pos, P_pos, x_pos);
    filter3.change_parameters(A_pos, H_pos, Q_pos, R_pos, P_pos, x_pos);
    filter4.change_parameters(A_rot, H_rot, Q_rot, R_rot, P_rot, x_rot);
    filter5.change_parameters(A_rot, H_rot, Q_rot, R_rot, P_rot, x_rot);
    filter6.change_parameters(A_rot, H_rot, Q_rot, R_rot, P_rot, x_rot);
    // filter7.change_parameters(A_yaw,H_yaw,Q_yaw,R_yaw,P_yaw,x_yaw);
    // filter8.change_parameters(A_mag_rot_vel,H_mag_rot_vel,Q_mag_rot_vel,R_mag_rot_vel,P_mag_rot_vel,x_mag_rot_vel);


    // set up PIDs
    pid1.set_parameters(Kp, Ki, Kd);
    pid2.set_parameters(Kp, Ki, Kd);
    pid3.set_parameters(Kp_r, Ki_r, Kd_r);
    pid4.set_parameters(Kp_r, Ki_r, Kd_r);
    pid5.set_parameters(Kp5, Ki5, Kd5, 9.81f/4);
    pid6.set_parameters(Kp6, Ki6, Kd6);

    desired_value1 = 0;
    desired_value2 = 0;
    desired_value3 = 0;
    desired_value4 = 0;
    desired_value5 = 0;
    desired_value6 = 0;

    output1 = 0;
    output2 = 0;
    output3 = 0;
    output4 = 0;
    output5 = 0;
    output6 = 0;

    offset_acceleration[0] = 0.15f;
    offset_acceleration[1]=-0.23f;
    offset_acceleration[2]=-0.92f;

    offset_omega[0]=-0.23f;
    offset_omega[1]=-0.02f;
    offset_omega[2]=-0.01f;

    pinMode(PIN_RECEIVER, INPUT);

    for (int i = 0; i < kChannelNumber; i++) {
        channel_norm[i]=-1.0f;
    }

    init_magnetometer();

    // Serial.write("Initialization complete, starting loop!\n");
}

void loop() {
    newGpsData = false;
    if (ss.available()) {
        parse_gps_string_and_do_critical_work();
    } else {
        do_critical_work();
        do_non_critical_work();
    }
}

void do_critical_work() {
    get_time();

    timer_magnetometer+=dt;
    if (timer_magnetometer > 0.005) {  // in seconds
        get_magnetometer_data();
        timer_magnetometer = 0;
    }

    // Serial.print(F("dt"));
    // Serial.print(dt,6);
    // Serial.println(F(","));

    get_imu_data();
    apply_kalman_filters();
    get_rc_data();
    calculate_PIDs();

    print_pwm_data();
    // print_rot_data();
    // print_propeller_thrust_data();

    print_magnetometer_data();
    Serial.println(F(""));

    get_serial_commands();
}

void do_non_critical_work() {
    // Serial.print(F("total_a:"));
    // Serial.print(total_acceleration);
    // Serial.print(F(","));
    // print_omega_data();
    // print_raw_acc_data();
    // print_acc_data();
    // print_rot_data();
    // print_gps_data();
    // print_rc_data();

    // Serial.println("");
}

void calibrate_IMU() {
    Serial.println(F("Calibrationg IMU..."));
    for (int i = 0; i < 100; i++) {
        mpu.getEvent(&a, &g, &temp);
        delay(10);
    }
    Serial.println(F("Getting offset..."));
    get_offset(0.0f, 0.0f, -9.81f);
    Serial.println(F("Offsets:"));
    Serial.print(offset_acceleration[0]);
    Serial.print(F(","));
    Serial.print(offset_acceleration[1]);
    Serial.print(F(","));
    Serial.print(offset_acceleration[2]);
    Serial.println(F(","));
}

void parse_gps_string_and_do_critical_work() {
    while (ss.available()) {
        if (gps.encode(ss.read())) newGpsData = true;
        do_critical_work();
    }
    if (newGpsData) {
        // get basic data
        gps.f_get_position(&flat, &flon, &age);
        gps_elevation = gps.f_altitude();
        gps_direction = gps.f_course();
        gps_speed = gps.f_speed_mps();

        // get fix info
        if (age == TinyGPS::GPS_INVALID_AGE) {
            // Serial.println(F("No fix detected"));
            gps_fix = false;
        } else if (age > 2000) {
            // Serial.println(F("Warning: possible stale data!"));
            gps_fix = false;
        } else {
            gps_fix = true;
            // Serial.println(F("Data is current."));
        }
    }  // Did a new valid sentence come in?
}



void get_time() {
    time_now = micros();
    time_elapsed = time_now-time_start;
    dt = (time_now - time_prev)*1e-6;  // s
    time_prev = time_now;
}

void calculate_PIDs() {
    // angle[0] = 0.0f;
    // angle[1] = 0.0f;
    angle[2] = 0.0f;

    desired_value1 = 0;
    desired_value2 = 0;
    desired_value5 = 0;  // sprememba višine
    desired_value6 = 5*yaw_rc;  // kot vrtenja (yaw) [MAX x*yaw_rc °/s]

    // get_inputs();

    // PID 1 velocity levo desno
    // output1 = pid1.Output(speed[2],desired_value1,dt);
    output1 = 0;

    // PID 2 velocity naprej nazaj
    // output2 = pid2.Output(speed[0],desired_value2,dt);
    output2 = 0;

    desired_value3 = 3*(roll_rc-0.5);
    desired_value4 = 3*(pitch_rc-0.5);

    // PID 3 levo desno roll
    output3 = pid3.Output(angle[0], desired_value3, dt);
    output3 = 0;

    // PID 4 naprej nazaj pitch
    output4 = pid4.Output(angle[1], desired_value4, dt);
    output4 = 0;

    // PID 5 thrust (gor dol)
    // output5 = pid5.Output(speed[1],desired_value5,dt);
    output5 = thrust_rc;

    desired_value6 = 10*(yaw_rc-0.5);

    // PID 6 yaw
    output6 = pid6.Output(omega[2], desired_value6, dt);

    F1m = (+output3-output4 +output5 -output6);  // Force magnitude
    F2m = (-output3-output4 +output5 +output6);
    F3m = (+output3+output4 +output5 +output6);
    F4m = (-output3+output4 +output5 -output6);

    if (F1m > 1.0f) {F1m = 1.0f;} else if (F1m < 0.0f) {F1m = 0.0f;}
    if (F2m > 1.0f) {F2m = 1.0f;} else if (F2m < 0.0f) {F2m = 0.0f;}
    if (F3m > 1.0f) {F3m = 1.0f;} else if (F3m < 0.0f) {F3m = 0.0f;}
    if (F4m > 1.0f) {F4m = 1.0f;} else if (F4m < 0.0f) {F4m = 0.0f;}

    if (!remote_turned_on) {
        F1m = 0; F2m = 0; F3m = 0; F4m = 0;
    }

    pwm_1 = (int)(F1m*255.0f);
    pwm_2 = (int)(F2m*255.0f);
    pwm_3 = (int)(F3m*255.0f);
    pwm_4 = (int)(F4m*255.0f);

    apply_pwm_to_propellers();
}

void get_imu_data() {
    mpu.getEvent(&a, &g, &temp);

    omega[0] = (g.gyro.x-offset_omega[0]);
    omega[1] = (g.gyro.y-offset_omega[1]);
    omega[2] = (g.gyro.z-offset_omega[2]);

    if (r2d(omega[2]) >= -0.5f && r2d(omega[2]) <= 0.5f) omega[2] = 0;

    acceleration[0] = (a.acceleration.x-offset_acceleration[0]);
    acceleration[1] = (a.acceleration.y-offset_acceleration[1]);
    acceleration[2] = (a.acceleration.z-offset_acceleration[2]);

    angle_acc[0] = atan2(acceleration[1], sqrt(acceleration[2] * acceleration[2] + acceleration[0] * acceleration[0]));
    angle_acc[1] = atan2(acceleration[0], sqrt(acceleration[2] * acceleration[2] + acceleration[1] * acceleration[1]));

    angle[0] = (0.98f * (angle[0] + omega[0] * dt)) + (0.02f * angle_acc[0]);
    angle[1] = (0.98f * (angle[1] + omega[1] * dt)) + (0.02f * angle_acc[1]);
    angle[2] = omega[2]*dt+angle[2];

    angle_deg[0] = wrap(r2d(angle[0]));
    angle_deg[1] = wrap(r2d(angle[1]));
    angle_deg[2] = wrap(r2d(angle[2]));
}

void apply_pwm_to_propellers() {
    analogWrite(5, pwm_1);
    analogWrite(6, pwm_2);
    analogWrite(9, pwm_3);
    analogWrite(10, pwm_4);
}

void get_offset(float init_x, float init_y, float init_z) {
    mpu.getEvent(&a, &g, &temp);
    offset_omega[0] = g.gyro.x;
    offset_omega[1] = g.gyro.y;
    offset_omega[2] = g.gyro.z;
    offset_acceleration[0] = a.acceleration.x-init_x;
    offset_acceleration[1] = a.acceleration.y-init_y;
    offset_acceleration[2] = a.acceleration.z-init_z;
}

void apply_kalman_filters() {
    acceleration[0] = filter1.Output(acceleration[0]);
    acceleration[1] = filter2.Output(acceleration[1]);
    acceleration[2] = filter3.Output(acceleration[2]);

    omega[0] = filter4.Output(omega[0]);
    omega[1] = filter5.Output(omega[1]);
    omega[2] = filter6.Output(omega[2]);
}


void get_rc_data() {
    for (int channel = 1; channel <= kChannelNumber; ++channel) {
        value = ppm.latestValidChannelValue(channel, 0);
        channel_norm[channel-1] = ((float)value-1000.0f)/10.0f/100.0f;
    }

    if (!remote_turned_on) {
        if (channel_norm[0] != -1.0f) {
            remote_turned_on = true;
        }
    }

    thrust_rc = channel_norm[2];
    pitch_rc = channel_norm[1];
    roll_rc = channel_norm[0];
    yaw_rc = channel_norm[3];
}

float r2d(float degrees) {
    return (degrees * 57.2958f);
}

void get_serial_commands() {
    if (Serial.available() > 0) {
        String incoming = Serial.readString();
        if (incoming == "reset") {
            get_offset(0.0f, 0.0f, -9.81f);
        }
    }
}

static float wrap(float angle) {
    while (angle > +180) angle -= 360;
    while (angle < -180) angle += 360;
    return angle;
}


void init_magnetometer() {
    myWire.begin();

    // initialization
    myWire.beginTransmission(address);
    myWire.write(0x0B);
    myWire.write(0x01);
    myWire.endTransmission();

    // set mode
    myWire.beginTransmission(address);
    myWire.write(0x09);
    myWire.write(Mode_Continuous|ODR_200Hz|RNG_2G|OSR_256);
    myWire.endTransmission();
}

void get_magnetometer_data() {
    // Serial.println(F("getting magnetometer data..."));

    // Tell the HMC5883L where to begin reading data
    myWire.beginTransmission(address);
    myWire.write(0x0);  // dunno why 0x03...
    int err = myWire.endTransmission();
    delay(5);

    if (err) {return;}

    // Read data from each axis, 2 registers per axis
    myWire.requestFrom(0x0D, 6);
    if (6 <= myWire.available()) {
        mag_x_new = myWire.read() | myWire.read() << 8;  // lsb, then msb
        mag_y_new = myWire.read() | myWire.read() << 8;
        mag_z_new = myWire.read() | myWire.read() << 8;
    }
    myWire.endTransmission();

    if (mag_x_new != -1 && mag_y_new != -1 && mag_z_new != -1) {
        mag_x = mag_x_new;
        mag_y = mag_y_new;
        mag_z = mag_z_new;
    }

    mag_azimuth = azimuth(-mag_y, mag_x);
}


float azimuth(int a, int b) {
    float azimuth = atan2((int)a, (int)b) * 180.0/PI;
    return azimuth < 0?360 + azimuth:azimuth;
}



void print_omega_data() {
    Serial.print(F("OmgX:"));
    Serial.print(omega[0]);
    Serial.print(F(",OmgY:"));
    Serial.print(omega[1]);
    Serial.print(F(",OmgZ:"));
    Serial.print(omega[2]);
    Serial.print(F(","));
}

void print_acc_data() {
    Serial.print(F("AccX:"));
    Serial.print(acceleration[0]);
    Serial.print(F(",AccY:"));
    Serial.print(acceleration[1]);
    Serial.print(F(",AccZ:"));
    Serial.print(acceleration[2]);
    Serial.print(F(","));
}

void print_raw_acc_data() {
    Serial.print(F("AccRawX:"));
    Serial.print(a.acceleration.x);
    Serial.print(F(",AccRawY:"));
    Serial.print(a.acceleration.y);
    Serial.print(F(",AccRawZ:"));
    Serial.print(a.acceleration.z);
    Serial.print(F(","));
}

void print_rc_data() {
    for (int i = 0; i < (kChannelNumber); i++) {
        Serial.print(F("RC"));
        Serial.print(i+1);
        Serial.print(F(":"));
        Serial.print(channel_norm[i]);
        Serial.print(F(","));
    }
    Serial.print(F("REMOTE:"));
    Serial.print(remote_turned_on);
    Serial.print(F(","));
}

void print_gps_data() {
    Serial.print(F("LAT:"));
    Serial.print(flat);
    Serial.print(F(",LON:"));
    Serial.print(flon);
    Serial.print(F(",DIR:"));
    Serial.print(gps_direction);
    Serial.print(F(",SPD:"));
    Serial.print(gps_speed);
    // Serial.print(F(",ALT:"));
    // Serial.print(gps_elevation);
    Serial.print(F(",FIX:"));
    Serial.print(gps_fix);
    Serial.print(F(","));
}

void print_rot_data() {
    Serial.print(F("AngX:"));
    Serial.print(angle_deg[0]);
    Serial.print(F(",AngY:"));
    Serial.print(angle_deg[1]);
    Serial.print(F(",AngZ:"));
    Serial.print(angle_deg[2]);
    Serial.print(F(","));
}

void print_pwm_data() {
    Serial.print(F("pwm_1:"));
    Serial.print(pwm_1);
    Serial.print(F(",pwm_2:"));
    Serial.print(pwm_2);
    Serial.print(F(",pwm_3:"));
    Serial.print(pwm_3);
    Serial.print(F(",pwm_4:"));
    Serial.print(pwm_4);
    Serial.print(F(","));
}

void print_propeller_thrust_data() {
    Serial.print(F("F1m:"));
    Serial.print(F1m);
    Serial.print(F(",F2m:"));
    Serial.print(F2m);
    Serial.print(F(",F3m:"));
    Serial.print(F3m);
    Serial.print(F(",F4m:"));
    Serial.print(F4m);
    Serial.print(F(","));
}


void print_magnetometer_data() {
    Serial.print(F("x:"));
    Serial.print(mag_x);
    Serial.print(F(",y:"));
    Serial.print(mag_y);
    Serial.print(F(",z:"));
    Serial.print(mag_z);
    Serial.print(F("a: "));
    Serial.print(mag_azimuth);
    Serial.print(F(","));
}
