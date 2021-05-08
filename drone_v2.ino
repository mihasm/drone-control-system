#include <Arduino.h>
#include <PPMReader.h>
#include <PID_regulator.h>
#include <Servo.h>
#include <SparkFunMPU9250-DMP.h>
#include <TripleFilter.h>

const char comma[] PROGMEM = {","};
#define g 9.81 // 1g ~ 9.81 m/s^2

// MPU9255 I2C init 
MPU9250_DMP imu;
int status_IMU;
float temperature, altitude_calc_prev;
#define acc_scale_value scale_2g
#define gyro_scale_value scale_250dps

// Servo stuff
Servo ESC_Servo_1, ESC_Servo_2, ESC_Servo_3, ESC_Servo_4;
int data1, data2, data3, data4;
float offset_servo1, offset_servo2, offset_servo3, offset_servo4;

// RC stuff
#define kChannelNumber 6  // Number of channels
float channel_norm[kChannelNumber];
#define PIN_RECEIVER 2  // rc receiver PPM pin
PPMReader ppm(PIN_RECEIVER, kChannelNumber);
bool remote_armed = false;
int remote_status = -2;
float thrust_rc = 0;
float pitch_rc = 0.5;
float roll_rc = 0.5;
float yaw_rc = 0.5;
unsigned long value_channel;
float last_flight_mode = -2;

// time variables
unsigned long time_start, time_now, time_prev;
float dt;
float frequency_saved;
int counter;

unsigned long time_now_pres, time_prev_pres;
float dt_pres;

// set up main data arrays
float acceleration[3];  // m/s^2
float omega[3];  // rad/s

// filtered
float acceleration_f[3];  // m/s^2
float omega_f[3];  // rad/s


float angle_acc[2];  // rads
float angle[3];  // rads
float angle_deg[3];  // degs

float speed[3] = {0.0f, 0.0f, 0.0f};  // m/s

float vertical_acceleration_imu;
float vertical_speed;
float vertical_speed_imu;

float offset_acceleration[3] = {-2.13f, -18.57f, 7.48f};
float offset_omega[3] = {0.04f, 0.02f, 0.01f};

// Omega filter
#define  Q_w   0.5   // Process noise (wind/driver input)
#define  R_w   0.5   // sensor inaccuracy. more = more innacurate

// Acceleration filter
#define  Q_a   0.5   // Process noise (wind/driver input)
#define  R_a   0.5   // sensor inaccuracy. more = more innacurate

//LPF
LPF lpfilter_roll_rc,lpfilter_pitch_rc,lpfilter_thrust_rc,lpfilter_yaw_rc;
#define f_c 10 // Hz, cutoff frequency
#define f_c_rc 4 // Hz, cutoff freuqency RC

// TripleFilter
TripleFilter filt1,filt2,filt3,filt4,filt5,filt6;

// PID STUFF

PID_regulator pid1, pid2, pid3, pid4, pid5, pid6;
bool stop_integration_3, stop_integration_4;

// PITCH

#define  Kp_w_pitch   0.0002 // PID 1,2 (stopnja B) (omega)
#define  Ki_w_pitch   0
#define  Kd_w_pitch   8e-6

#define  Kp_theta_pitch   7   // PID 3,4 (stopnja A) (stopinje)
#define  Ki_theta_pitch   1
#define  Kd_theta_pitch   0

// ROLL

#define  Kp_w_roll   0.00005 // PID 1,2 (stopnja B) (omega)
#define  Ki_w_roll   0
#define  Kd_w_roll   8e-7

#define  Kp_theta_roll   6   // PID 3,4 (stopnja A) (stopinje)
#define  Ki_theta_roll   1
#define  Kd_theta_roll   0

// YAW

#define  Kp_yaw   0.00005   // PID 6 - yaw (stopinje)
#define  Ki_yaw   0.0
#define  Kd_yaw   0.0

float desired_value1,desired_value2,desired_value3,desired_value4,desired_value5,desired_value6;
float output1,output2,output3,output4,output5,output6;

// MAIN OUTPUTS

#define MAX_DEGREES 15.0 // Max Degrees (Normal mode)
#define MAX_DPS_YAW 180 // Degrees Per Second
#define MAX_DPS_PITCH_ROLL 150 // Degrees Per Second (Acro mode)
#define MAX_VERT_SPEED 1 // (Only Altitude hold mode)

float F1m, F2m, F3m, F4m;
int pwm_1, pwm_2, pwm_3, pwm_4;


void setup() {
    ESC_Servo_1.attach(3);
    ESC_Servo_2.attach(5);
    ESC_Servo_3.attach(9);
    ESC_Servo_4.attach(6);

    ESC_Servo_1.write(0);
    ESC_Servo_2.write(0);
    ESC_Servo_3.write(0);
    ESC_Servo_4.write(0);
    

    Serial.begin(500000);
    while (!Serial) {}
    Serial.print(F("Init...\n"));

    offset_servo1 = 0;
    offset_servo2 = 0;
    offset_servo3 = 0;
    offset_servo4 = 0;

    int sensorValue = analogRead(A7);
    float voltage = sensorValue * (5.0/1023.0) * 3.518816f;

    Serial.print(F("Voltage: "));
    Serial.print(voltage);
    Serial.println(F(" V"));

    // IMU
    delay(1000);

    if (imu.begin() != INV_SUCCESS)
    {
        while (1)
        {
            // Failed to initialize MPU-9250, loop forever
        }
    }
    Serial.println("success...");

    imu.setSensors(INV_XYZ_GYRO | INV_XYZ_ACCEL); // Enable all sensors

    // Gyro options are +/- 250, 500, 1000, or 2000 dps
    imu.setGyroFSR(500); // Set gyro to 2000 dps
    // Accel options are +/- 2, 4, 8, or 16 g
    imu.setAccelFSR(4); // Set accel to +/-2g

    // The sample rate of the accel/gyro can be set using
    // setSampleRate. Acceptable values range from 4Hz to 1kHz
    imu.setSampleRate(1000);

    // set LPF
    //imu.setLPF(98);

    // set up kalman filters
    filt1.change_parameters(Q_a, R_a, f_c, 0);
    filt2.change_parameters(Q_a, R_a, f_c, 0);
    filt3.change_parameters(Q_a, R_a, f_c, 0);
    
    filt4.change_parameters(Q_w, R_w, f_c, 0);
    filt5.change_parameters(Q_w, R_w, f_c, 0);
    filt6.change_parameters(Q_w, R_w, f_c, 0);

    lpfilter_yaw_rc.change_parameters(f_c_rc,0.5);
    lpfilter_thrust_rc.change_parameters(f_c_rc,0);
    lpfilter_roll_rc.change_parameters(f_c_rc,0.5);
    lpfilter_pitch_rc.change_parameters(f_c_rc,0.5);
    
    // set up PIDs
    pid1.set_parameters(Kp_w_roll, Ki_w_roll, Kd_w_roll);
    pid2.set_parameters(Kp_w_pitch, Ki_w_pitch, Kd_w_pitch);
    pid3.set_parameters(Kp_theta_roll, Ki_theta_roll, Kd_theta_roll);
    pid4.set_parameters(Kp_theta_pitch, Ki_theta_pitch, Kd_theta_pitch);
    //pid5.set_parameters(Kp_5, Ki_5, Kd_5);
    //pid6.set_parameters(Kp_6, Ki_6, Kd_6);

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

    
    Serial.print(F("Waiting for transmitter... "));
    while (get_rc_status() != 1) {
        Serial.print(F(" "));
        Serial.print(get_rc_status());
        get_rc_data();
        delay(100);
    }
    Serial.print(F("Transmitter detected, starting loop!\n"));
    
    

    time_start = micros();
    time_prev = time_start;

    calibrate();
}

void loop() {
    counter += 1;

    get_imu_data();
    get_rc_data();
    calculate_PIDs();
    get_time();

    apply_pid_to_pwm();

    if (remote_armed) {
        apply_pwm_to_propellers();
    } else {
        ESC_Servo_1.writeMicroseconds(1000);
        ESC_Servo_2.writeMicroseconds(1000);
        ESC_Servo_3.writeMicroseconds(1000);
        ESC_Servo_4.writeMicroseconds(1000);
    }
    
    
    //if (counter > 500) {
        //counter = 0;
        //print_dt();
        //print_frequency();
        //print_rc_data();
        //print_processed_rc_data();
        print_angle_deg();
        //print_omega_data(); // raw rot. velocity
        //print_acc_data(); // raw acceleration
        //print_propeller_thrust_data();
        //print_pwm_data();
        //print_pid_data();
        //print_raw_acc_data();
        //print_pressure_data();
        //print_angle_rad();
        //print_vertical_speed();
        //get_serial_commands();
        Serial.println(F(""));
    //}
    
}

void calibrate() {
    Serial.println("Calibrating");
    offset_omega[0] = -imu.calcGyro(imu.gy)*0.0174533;
    offset_omega[1] = imu.calcGyro(imu.gx)*0.0174533;
    offset_omega[2] = imu.calcGyro(imu.gz)*0.0174533;

    offset_acceleration[0] = imu.calcAccel(imu.ay)*9.81;
    offset_acceleration[1] = imu.calcAccel(imu.ax)*9.81;
    offset_acceleration[2] = 9.81 - imu.calcAccel(imu.az)*9.81;

    for (int i=0; i<100; i++) {
        while (imu.dataReady() != 1) {
            delay(1);
        }
        imu.update(UPDATE_ACCEL | UPDATE_GYRO);

        offset_omega[0] = (-imu.calcGyro(imu.gy)*0.0174533+offset_omega[0])/2;
        offset_omega[1] = (imu.calcGyro(imu.gx)*0.0174533+offset_omega[1])/2;
        offset_omega[2] = (imu.calcGyro(imu.gz)*0.0174533+offset_omega[2])/2;

        offset_acceleration[0] = (imu.calcAccel(imu.ay)*9.81+offset_acceleration[0])/2;
        offset_acceleration[1] = (imu.calcAccel(imu.ax)*9.81+offset_acceleration[1])/2;
        offset_acceleration[2] = (9.81 - imu.calcAccel(imu.az)*9.81+offset_acceleration[2])/2;
        Serial.print(".");
    }
    Serial.println("Calibrated!");

    print_offsets();
}


void get_time() {
    time_now = micros();
    dt = (time_now - time_prev)*1e-6;  // s
    time_prev = time_now;
    frequency_saved = (frequency_saved + 1/dt)/2;
}

void get_time_pressure() {
    time_now_pres = micros();
    dt_pres = (micros() - time_prev_pres)*1e-6;  // s
    time_prev_pres = time_now_pres;
}


void calculate_PIDs() {
    if (remote_armed == true) {
        if (abs(output3) > Kp_theta_roll*MAX_DEGREES*2) {
            stop_integration_3 = true;
        } else {
            stop_integration_3 = false;
        }
        if (abs(output4) > Kp_theta_pitch*MAX_DEGREES*2) {
            stop_integration_4 = true;
        } else {
            stop_integration_4 = false;
        }
        
        if (flight_mode_change() == 1) {
            Serial.println(F("Flight mode changed!\n"));
            pid1.ResetID();
            pid2.ResetID();
            pid3.ResetID();
            pid4.ResetID();
            pid5.ResetID();
            pid6.ResetID();
        }
        
        if (get_flight_mode() == 1 || get_flight_mode() == 2) {
            // Normal mode or Altitude hold mode

            desired_value3 = MAX_DEGREES*(roll_rc-0.5)*2;
            desired_value4 = MAX_DEGREES*(pitch_rc-0.5)*2;
            
            // PID 3 roll
            output3 = pid3.Output(-angle_deg[0], -desired_value3, dt, stop_integration_3);
            // PID 4 pitch
            output4 = pid4.Output(angle_deg[1], desired_value4, dt, stop_integration_4);

            // PID 1 omega roll
            output1 = pid1.Output(r2d(-omega[0]), output3, dt);
            // PID 2 omega pitch
            output2 = pid2.Output(r2d(omega[1]), output4, dt);

        } else if (get_flight_mode() == 0) {
            // Acro mode
            output3 = 0;
            output4 = 0;

            desired_value1 = MAX_DPS_PITCH_ROLL*(roll_rc-0.5)*2;
            desired_value2 = MAX_DPS_PITCH_ROLL*(pitch_rc-0.5)*2;
            
            // PID 1 omega roll
            output1 = pid1.Output(r2d(-omega[0]), -desired_value1, dt);
            // PID 2 omega pitch
            output2 = pid2.Output(r2d(omega[1]), desired_value2, dt);

        }

        if (get_flight_mode() == 0 || get_flight_mode() == 1) {
            output5 = thrust_rc;
        } else {
            desired_value5 = MAX_VERT_SPEED*(thrust_rc-0.5)*2;
            output5 = pid5.Output(vertical_speed,desired_value5,dt);
        }

        desired_value6 = MAX_DPS_YAW*(yaw_rc-0.5)*2;
        // PID 6 yaw (All modes)
        output6 = pid6.Output(r2d(omega[2]), desired_value6, dt);
        //output6 =  0.2f*(yaw_rc-0.5)*2;

    } else {
        pid1.ResetOutput();
        pid2.ResetOutput();
        pid3.ResetOutput();
        pid4.ResetOutput();
        pid6.ResetOutput();
        output1 = 0;
        output2 = 0;
        output3 = 0;
        output4 = 0;
        output6 = 0;
    }
}

void apply_pid_to_pwm() {

    F1m = (-output1-output2);  // Force magnitude
    F2m = (+output1-output2);
    F3m = (-output1+output2);
    F4m = (+output1+output2);

    F1m += (+output5 -output6);  // Force magnitude
    F2m += (+output5 +output6);
    F3m += (+output5 +output6);
    F4m += (+output5 -output6);

    F1m += offset_servo1;  // Force magnitude
    F2m += offset_servo2;
    F3m += offset_servo3;
    F4m += offset_servo4;

    if (F1m < 0) {F1m = 0;}
    if (F2m < 0) {F2m = 0;}
    if (F3m < 0) {F3m = 0;}
    if (F4m < 0) {F4m = 0;}

    if (F1m > 1) {F1m = 1;}
    if (F2m > 1) {F2m = 1;}
    if (F3m > 1) {F3m = 1;}
    if (F4m > 1) {F4m = 1;}

    if (!remote_armed) {
        F1m = 0; F2m = 0; F3m = 0; F4m = 0;
    }

    pwm_1 = static_cast<int>(F1m*255 + 0.5);
    pwm_2 = static_cast<int>(F2m*255 + 0.5);
    pwm_3 = static_cast<int>(F3m*255 + 0.5);
    pwm_4 = static_cast<int>(F4m*255 + 0.5);
}

void get_imu_data() {
    imu.update(UPDATE_ACCEL | UPDATE_GYRO);

    omega[0] = -imu.calcGyro(imu.gy)*0.0174533;
    omega[1] = imu.calcGyro(imu.gx)*0.0174533;
    omega[2] = imu.calcGyro(imu.gz)*0.0174533;

    acceleration[0] = imu.calcAccel(imu.ay)*9.81;
    acceleration[1] = imu.calcAccel(imu.ax)*9.81;
    acceleration[2] = -imu.calcAccel(imu.az)*9.81;

    //subtract offset
    omega[0] = omega[0]- offset_omega[0];
    omega[1] = omega[1]- offset_omega[1];
    omega[2] = omega[2]- offset_omega[2];

    //subtract acc
    acceleration[0] = acceleration[0]- offset_acceleration[0];
    acceleration[1] = acceleration[1]- offset_acceleration[1];
    acceleration[2] = acceleration[2]- offset_acceleration[2];

    //print_acc_data();
    //print_omega_data();

    // APPLY FILT
    acceleration[0] = filt1.Output(acceleration[0],dt);
    acceleration[1] = filt2.Output(acceleration[1],dt);
    acceleration[2] = filt3.Output(acceleration[2],dt);

    omega[0] = filt4.Output(omega[0],dt);
    omega[1] = filt5.Output(omega[1],dt);
    omega[2] = filt6.Output(omega[2],dt);

    //print_acc_data_filter();
    //print_omega_data_filter();

    angle_acc[0] = atan2(acceleration[1], sqrt(acceleration[2] * acceleration[2] + acceleration[0] * acceleration[0]));
    angle_acc[1] = atan2(acceleration[0], sqrt(acceleration[2] * acceleration[2] + acceleration[1] * acceleration[1]));

    angle[0] = (0.99f * (angle[0] + omega[0] * dt)) + (0.01f * angle_acc[0]);
    angle[1] = (0.99f * (angle[1] + omega[1] * dt)) + (0.01f * angle_acc[1]);
    angle[2] = 0.0f;

    angle_deg[0] = wrap(r2d(angle[0]));
    angle_deg[1] = wrap(r2d(angle[1]));
    angle_deg[2] = wrap(r2d(angle[2]));

    //vertical_acceleration_imu =  -(acceleration[2]*cos(angle[0])*cos(angle[1]) - acceleration[0]*sin(angle[1]) - acceleration[1]*cos(angle[1])*sin(angle[0])+9.82f);
    //vertical_speed_imu = (0.4f*vertical_speed+0.6f*vertical_speed_imu) + vertical_acceleration_imu*dt;
    //vertical_speed = 0.05f*vertical_speed_BMP+0.95f*vertical_speed_imu;
    
}

void apply_pwm_to_propellers() {
    data1 = map(pwm_1, 0, 256, 1000, 2000);
    data2 = map(pwm_2, 0, 256, 1000, 2000);
    data3 = map(pwm_3, 0, 256, 1000, 2000);
    data4 = map(pwm_4, 0, 256, 1000, 2000);

    ESC_Servo_1.writeMicroseconds(data1);
    ESC_Servo_2.writeMicroseconds(data2);
    ESC_Servo_3.writeMicroseconds(data3);
    ESC_Servo_4.writeMicroseconds(data4);
}


void get_rc_data() {
    for (int channel = 0; channel < kChannelNumber; channel++) {
        value_channel = ppm.latestValidChannelValue(channel+1,0);
        channel_norm[channel] = ((float)value_channel - 1000)/1000; // TODO: this line eats up 400 ms of time....
    }

    roll_rc = lpfilter_roll_rc.Output(channel_norm[0],dt);
    pitch_rc = lpfilter_pitch_rc.Output(channel_norm[1],dt);
    thrust_rc = lpfilter_thrust_rc.Output(channel_norm[2],dt);
    yaw_rc = lpfilter_yaw_rc.Output(channel_norm[3],dt);

    remote_status = get_rc_status();

    if (remote_status == 2) {
        if (remote_armed == false) {
            if (thrust_rc < 0.05) {
                remote_armed = true;
            }
        }
    } else {
        remote_armed = false;
    }
}

float r2d(float degrees) {
    return (degrees * 57.2958f);
}

void get_serial_commands() {
    if (Serial.available() > 0) {
        String incoming = Serial.readString();
        if (incoming == "reset") {
        }
    }
}

static float wrap(float angle) {
    while (angle > +180) angle -= 360;
    while (angle < -180) angle += 360;
    return angle;
}


int get_rc_status() {
    if (channel_norm[4] < -0.5) {
        // receiver connected, transmitter not connected
        return -1;
    } else if (channel_norm[4] > -0.1 && channel_norm[4] < 0.1) {
        // fall back mode
        return 0;
    } else if (channel_norm[4] > 0.4 && channel_norm[4] < 0.6) {
        // receiver connected, transmitter connected (NOT ARMED)
        return 1;
    } else if (channel_norm[4] > 0.9) {
        // receiver connected, transmitter connected (ARMED)
        return 2;
    }
}

int get_flight_mode() {
    if (channel_norm[5] < -0.5) {
        // receiver connected, transmitter not connected
        return -1;
    } else if (channel_norm[5] > -0.1 && channel_norm[5] < 0.1) {
        // switch all the way forward
        return 0;
    } else if (channel_norm[5] > 0.4 && channel_norm[5] < 0.6) {
        // switch in the middle
        return 1;
    } else if (channel_norm[5] > 0.9) {
        // switch all the way back
        return 1;
        // currently set to 1 because there is no pressure sensor
    }
}

bool flight_mode_change() {
    if (get_flight_mode() != last_flight_mode) {
        last_flight_mode = get_flight_mode();
        return 1;
    }
    last_flight_mode = get_flight_mode();
    return 0;
}

void print_dt() {
    Serial.print(F("dt:"));
    Serial.print(dt*1e3,10);
    delimiter();
}

void print_frequency() {
    Serial.print(F("f:"));
    Serial.print(1/dt,0);
    delimiter();
}

void delimiter() {
    Serial.print(comma[0]);
}


void print_omega_data() {
    Serial.print(F("OmgX:"));
    Serial.print(omega[0]*100,10);
    Serial.print(F(",OmgY:"));
    Serial.print(omega[1]*100,10);
    Serial.print(F(",OmgZ:"));
    Serial.print(omega[2]*100,10);
    delimiter();
}

void print_omega_data_filter() {
    Serial.print(F("OmgX_f:"));
    Serial.print(omega[0]*100,10);
    Serial.print(F(",OmgY_f:"));
    Serial.print(omega[1]*100,10);
    Serial.print(F(",OmgZ_f:"));
    Serial.print(omega[2]*100,10);
    delimiter();
}

void print_acc_data() {
    Serial.print(F("aX:"));
    Serial.print(acceleration[0]);
    Serial.print(F(",aY:"));
    Serial.print(acceleration[1]);
    Serial.print(F(",aZ:"));
    Serial.print(acceleration[2]);
    delimiter();
}



void print_acc_data_filter() {
    Serial.print(F("aX_f:"));
    Serial.print(acceleration[0]);
    Serial.print(F(",aY_f:"));
    Serial.print(acceleration[1]);
    Serial.print(F(",aZ_f:"));
    Serial.print(acceleration[2]);
    delimiter();
}

void print_rc_data() {
    for (int i = 0; i < (kChannelNumber); i++) {
        Serial.print(F("RC"));
        Serial.print(i+1);
        Serial.print(F(":"));
        Serial.print(channel_norm[i],4);
        delimiter();
    }
    Serial.print(F("RC_STATUS:"));
    Serial.print(get_rc_status());
    delimiter();
}

void print_angle_deg() {
    Serial.print(F("AngX:"));
    Serial.print(angle_deg[0]);
    Serial.print(F(",AngY:"));
    Serial.print(angle_deg[1]);
    Serial.print(F(",AngZ:"));
    Serial.print(angle_deg[2]);
    delimiter();
}

void print_angle_rad() {
    Serial.print(F("AngX:"));
    Serial.print(angle[0]);
    Serial.print(F(",AngY:"));
    Serial.print(angle[1]);
    Serial.print(F(",AngZ:"));
    Serial.print(angle[2]);
    delimiter();
}

void print_pwm_data() {
    Serial.print(F("pwm1:"));
    Serial.print(pwm_1);
    Serial.print(F(",pwm2:"));
    Serial.print(pwm_2);
    Serial.print(F(",pwm3:"));
    Serial.print(pwm_3);
    Serial.print(F(",pwm4:"));
    Serial.print(pwm_4);
    delimiter();
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
    delimiter();
}

void print_offsets() {
    Serial.println(F("Offsets:"));
    Serial.print(offset_acceleration[0]);
    delimiter();
    Serial.print(offset_acceleration[1]);
    delimiter();
    Serial.print(offset_acceleration[2]);
    delimiter();

    Serial.println(F("Offsets omega:"));
    Serial.print(offset_omega[0]);
    delimiter();
    Serial.print(offset_omega[1]);
    delimiter();
    Serial.print(offset_omega[2]);
    Serial.println(",\n");
}

void print_desired_values() {
    Serial.print(F("desired_roll:"));
    Serial.print(desired_value3);
    Serial.print(F(",desired_pitch:"));
    Serial.print(desired_value4);
    delimiter();
}

void print_pid_data() {
    Serial.print(F("pid1:"));
    Serial.print(output1);
    Serial.print(F(",pid2:"));
    Serial.print(output2);
    Serial.print(F(",pid3:"));
    Serial.print(output3);
    Serial.print(F(",pid4:"));
    Serial.print(output4);
    Serial.print(F(",pid5:"));
    Serial.print(output5);
    Serial.print(F(",desired3:"));
    Serial.print(desired_value3);
    Serial.print(F(",desired4:"));
    Serial.print(desired_value4);
    delimiter();
}

void print_processed_rc_data() {
    Serial.print(F("roll:"));
    Serial.print(roll_rc,4);                    // Display the results    
    Serial.print(F(",pitch:"));
    Serial.print(pitch_rc,4);    
    Serial.print(F(",thrust:"));
    Serial.print(thrust_rc,4);
    Serial.print(F(",yaw:"));
    Serial.print(yaw_rc,4);
    
    delimiter();
}