#include <Arduino.h>
#include <PPMReader.h>
#include <KalmanFilter.h>
#include <PID_regulator.h>
#include <Servo.h>
#include <BMP280_DEV.h>
#include <MPU9255.h>// include MPU9255 library

const char comma[] PROGMEM = {","};
const char ADDRESSES[] PROGMEM = {0x76,0x68}; // [0]->BMP, [1]->IMU

// MPU9255 I2C init 

#define g 9.81 // 1g ~ 9.81 m/s^2
#define magnetometer_cal 0.06 //magnetometer calibration

MPU9255 mpu;
int status_IMU;
float temperature, altitude_calc_prev;
#define acc_scale_value scale_4g
#define gyro_scale_value scale_500dps

// BMP280 I2C init
float temperature_BMP;
float pressure = 1e3;            // Create the temperature, pressure and altitude variables
BMP280_DEV bmp280;                                // Instantiate (create) a BMP280_DEV object and set-up for I2C operation
float altitude_BMP, altitude_BMP_prev, vertical_speed_BMP;
bool first_data = 1;

// Servo stuff
Servo ESC_Servo_1, ESC_Servo_2, ESC_Servo_3, ESC_Servo_4;
int data1, data2, data3, data4;
int offset_servo1, offset_servo2, offset_servo3, offset_servo4;

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
unsigned long time_now_pres, time_prev_pres;
float dt_pres;

// set up main data arrays
float acceleration[3];  // m/s^2
float angle_acc[2];  // rads
float angle[3];  // rads
float angle_deg[3];  // degs
float omega[3];  // rad/s
float speed[3] = {0.0f, 0.0f, 0.0f};  // m/s
float vertical_acceleration_imu;
float vertical_speed;
float vertical_speed_imu;

float offset_acceleration[3] = {0.0f, 0.0f, 0.0f};
float offset_omega[3] = {0.0f, 0.0f, 0.0f};

KalmanFilter filter1, filter2, filter3, filter4,
             filter5, filter6, filter7, filter8, filter9, filter10;

#define  A_rot   1.0   // factor of real value to previous real value
#define  B_rot   0.0   // factor of real value to real control signal
#define  H_rot   1.0   // sprememba merjene vrednosti zaradi enote/drugo
#define  Q_rot   1.0   // Process noise (wind/driver input)
#define  R_rot   10.0   // sensor inaccuracy. more = more innacurate
#define  P_rot   0.0   // zacetni vrednosti
#define  x_rot   0.0   // zacetni vrednosti

#define  A_degrees   1.0   // factor of real value to previous real value
#define  B_degrees   0.0   // factor of real value to real control signal
#define  H_degrees   1.0   // sprememba merjene vrednosti zaradi enote/drugo
#define  Q_degrees   5.0   // Process noise (wind/driver input)
#define  R_degrees   10.0   // sensor inaccuracy. more = more innacurate
#define  P_degrees   0.0   // zacetni vrednosti
#define  x_degrees   0.0   // zacetni vrednosti

#define  A_height   1.0   // factor of real value to previous real value
#define  B_height   0.0   // factor of real value to real control signal
#define  H_height   1.0   // sprememba merjene vrednosti zaradi enote/drugo
#define  Q_height   0.01   // Process noise (wind/driver input)
#define  R_height   0.5   // sensor inaccuracy. more = more innacurate
#define  P_height   0.0   // zacetni vrednosti
#define  x_height   115.0   // zacetni vrednosti

#define  A_temp   1.0   // factor of real value to previous real value
#define  B_temp   0.0   // factor of real value to real control signal
#define  H_temp   1.0   // sprememba merjene vrednosti zaradi enote/drugo
#define  Q_temp   0.05   // Process noise (wind/driver input)
#define  R_temp   0.1   // sensor inaccuracy. more = more innacurate
#define  P_temp   0.0   // zacetni vrednosti
#define  x_temp   20.0   // zacetni vrednosti

#define  A_press   1.0   // factor of real value to previous real value
#define  B_press   0.0   // factor of real value to real control signal
#define  H_press   1.0   // sprememba merjene vrednosti zaradi enote/drugo
#define  Q_press   0.02   // Process noise (wind/driver input)
#define  R_press   5   // sensor inaccuracy. more = more innacurate
#define  P_press   0.0   // zacetni vrednosti
#define  x_press   1000.0   // zacetni vrednosti

#define  A_vert   1.0   // factor of real value to previous real value
#define  B_vert   0.0   // factor of real value to real control signal
#define  H_vert   1.0   // sprememba merjene vrednosti zaradi enote/drugo
#define  Q_vert   0.05   // Process noise (wind/driver input)
#define  R_vert   15.0   // sensor inaccuracy. more = more innacurate
#define  P_vert   0.0   // zacetni vrednosti
#define  x_vert   0.0   // zacetni vrednosti

PID_regulator pid1, pid2, pid3, pid4, pid5, pid6;
bool stop_integration_3, stop_integration_4;

#define  Kp   0.0010 // PID 1,2 (stopnja B) (omega)
#define  Ki   0.0000
#define  Kd   0.000005

#define  Kp_r   10.0   // PID 3,4 (stopnja A) (stopinje)
#define  Ki_r   0.0  // npr 2
#define  Kd_r   0.0

#define  Kp_5   1.0   // PID 5 - thrust (Altitude hold)
#define  Ki_5   0.0
#define  Kd_5   0.0

#define  Kp_6   0.0   // PID 6 - yaw (stopinje)
#define  Ki_6   0.0
#define  Kd_6   0.0

#define CALIBRATION_ITERATIONS 100

#define MAX_DEGREES 15.0 // Max Degrees (Normal mode)
#define MAX_DPS_YAW 5 // Degrees Per Second
#define MAX_DPS_PITCH_ROLL 180 // Degrees Per Second (Acro mode)
#define MAX_VERT_SPEED 1 // (Only Altitude hold mode)

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

    Serial.begin(230400);
    Serial.setTimeout(150);
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

    Serial.print(F("Initializing IMU..."));
    if(mpu.init()){
        Serial.println("initialization failed");
        while (1) {};
    } else {
        Serial.println("initialization successful!");
    }

    mpu.set_acc_bandwidth(acc_1113Hz);//set accelerometer bandwidth
    mpu.set_gyro_bandwidth(gyro_250Hz);//set gyroscope bandwidth
    mpu.set_acc_scale(acc_scale_value);
    mpu.set_gyro_scale(gyro_scale_value);

    //set new offset

    mpu.set_gyro_offset(X_axis,-43);
    mpu.set_gyro_offset(Y_axis,88);
    mpu.set_gyro_offset(Z_axis,-14);
    mpu.set_acc_offset(X_axis,907);
    mpu.set_acc_offset(Y_axis,241);
    mpu.set_acc_offset(Z_axis,194);

    if (!bmp280.begin(ADDRESSES[0])) {
        Serial.print(F("BMP initialization failed!"));
        while (1) {};
    }              // Default initialisation with alternative I2C address (0x76), place the BMP280 into SLEEP_MODE 
    bmp280.setPresOversampling(OVERSAMPLING_X1);    // Set the pressure oversampling to X4
    bmp280.setTempOversampling(OVERSAMPLING_SKIP);    // Set the temperature oversampling to X1
    bmp280.setIIRFilter(IIR_FILTER_OFF);              // Set the IIR filter to setting 4
    bmp280.setTimeStandby(TIME_STANDBY_05MS);     // Set the standby time to 2 seconds
    bmp280.startNormalConversion();                 // Start BMP280 continuous conversion in NORMAL_MODE

    while (!bmp280.getTempPres(temperature_BMP, pressure)) {
        delay(10);
    }

    altitude_BMP = (pow(1013.25/pressure,1/5.257)-1)*(temperature+273.15)/0.0065;


    // set up kalman filters
    filter1.change_parameters(A_degrees, H_degrees, Q_degrees, R_degrees, P_degrees, x_degrees);
    filter2.change_parameters(A_degrees, H_degrees, Q_degrees, R_degrees, P_degrees, x_degrees);
    filter3.change_parameters(A_degrees, H_degrees, Q_degrees, R_degrees, P_degrees, x_degrees);
    filter4.change_parameters(A_rot, H_rot, Q_rot, R_rot, P_rot, x_rot);
    filter5.change_parameters(A_rot, H_rot, Q_rot, R_rot, P_rot, x_rot);
    filter6.change_parameters(A_rot, H_rot, Q_rot, R_rot, P_rot, x_rot);
    filter7.change_parameters(A_height, H_height, Q_height, R_height, P_height, altitude_BMP);
    filter8.change_parameters(A_temp, H_temp, Q_temp, R_temp, P_temp, temperature);
    filter9.change_parameters(A_press, H_press, Q_press, R_press, P_press, pressure);
    filter10.change_parameters(A_vert, H_vert, Q_vert, R_vert, P_vert, x_vert);

    // set up PIDs
    pid1.set_parameters(Kp, Ki, Kd);
    pid2.set_parameters(Kp, Ki, Kd);
    pid3.set_parameters(Kp_r, Ki_r, Kd_r);
    pid4.set_parameters(Kp_r, Ki_r, Kd_r);
    pid5.set_parameters(Kp_5, Ki_5, Kd_5);
    pid6.set_parameters(Kp_6, Ki_6, Kd_6);

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
}

void loop() {
    get_time();
    get_rc_data();
    get_imu_data();
    //get_pressure_data();
    calculate_PIDs();
    apply_pid_to_pwm();

    
    if (remote_armed) {
        apply_pwm_to_propellers();
    } else {
        ESC_Servo_1.writeMicroseconds(1000);
        ESC_Servo_2.writeMicroseconds(1000);
        ESC_Servo_3.writeMicroseconds(1000);
        ESC_Servo_4.writeMicroseconds(1000);
    }
    

    //print_dt();
    //print_frequency();
    //print_rc_data();
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
}

double process_acceleration(int input, scales sensor_scale ) {
  /*
  To get acceleration in 'g', each reading has to be divided by :
   -> 16384 for +- 2g scale (default scale)
   -> 8192  for +- 4g scale
   -> 4096  for +- 8g scale
   -> 2048  for +- 16g scale
  */
  double output = 1;

  //for +- 2g

  if(sensor_scale == scale_2g)
  {
    output = input;
    output = output/16384;
    output = output*g;
  }

  //for +- 4g
  if(sensor_scale == scale_4g)
  {
    output = input;
    output = output/8192;
    output = output*g;
  }

  //for +- 8g
  if(sensor_scale == scale_8g)
  {
    output = input;
    output = output/4096;
    output = output*g;
  }

  //for +-16g
  if(sensor_scale == scale_16g)
  {
    output = input;
    output = output/2048;
    output = output*g;
  }

  return output;
}

//process raw gyroscope data
//input = raw reading from the sensor, sensor_scale = selected sensor scale
//returns : angular velocity in degrees per second
double process_angular_velocity(int16_t input, scales sensor_scale ) {
  /*
  To get rotation velocity in dps (degrees per second), each reading has to be divided by :
   -> 131   for +- 250  dps scale (default value)
   -> 65.5  for +- 500  dps scale
   -> 32.8  for +- 1000 dps scale
   -> 16.4  for +- 2000 dps scale
  */

  //for +- 250 dps
  if(sensor_scale == scale_250dps)
  {
    return input/131;
  }

  //for +- 500 dps
  if(sensor_scale == scale_500dps)
  {
    return input/65.5;
  }

  //for +- 1000 dps
  if(sensor_scale == scale_1000dps)
  {
    return input/32.8;
  }

  //for +- 2000 dps
  if(sensor_scale == scale_2000dps)
  {
    return input/16.4;
  }

  return 0;
}

//process raw magnetometer data
//input = raw reading from the sensor, sensitivity =
//returns : magnetic flux density in μT (in micro Teslas)
double process_magnetic_flux(int16_t input, double sensitivity) {
  /*
  To get magnetic flux density in μT, each reading has to be multiplied by sensitivity
  (Constant value different for each axis, stored in ROM), then multiplied by some number (calibration)
  and then divided by 0.6 .
  (Faced North each axis should output around 31 µT without any metal / walls around
  Note : This manetometer has really low initial calibration tolerance : +- 500 LSB !!!
  Scale of the magnetometer is fixed -> +- 4800 μT.
  */
  return (input*magnetometer_cal*sensitivity)/0.6;
}


void get_time() {
    time_now = micros();
    dt = (time_now - time_prev)*1e-6;  // s
    time_prev = time_now;
}

void get_time_pressure() {
    time_now_pres = micros();
    dt_pres = (micros() - time_prev_pres)*1e-6;  // s
    time_prev_pres = time_now_pres;
}


void calculate_PIDs() {
    if (remote_armed == true) {
        if (abs(output3) > Kp_r*MAX_DEGREES) {
            stop_integration_3 = true;
        } else {
            stop_integration_3 = false;
        }
        if (abs(output4) > Kp_r*MAX_DEGREES) {
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

        //desired_value6 = MAX_DPS_YAW*(yaw_rc-0.5)*2;
        // PID 6 yaw (All modes)
        //output6 = pid6.Output(r2d(omega[2]), desired_value6, dt);
        output6 =  0.2f*(yaw_rc-0.5)*2;

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
    // read the sensor
    mpu.read_acc();
    mpu.read_gyro();
    //mpu.read_mag();

    // temperature = IMU.getTemperature_C();
    // temperature = filter8.Output(temperature);

    omega[0] = -process_angular_velocity(mpu.gy,gyro_scale_value)*0.0174533;
    omega[1] = process_angular_velocity(mpu.gx,gyro_scale_value)*0.0174533;
    omega[2] = process_angular_velocity(mpu.gz,gyro_scale_value)*0.0174533;

    acceleration[0] = process_acceleration(mpu.ay,acc_scale_value);
    acceleration[1] = process_acceleration(mpu.ax,acc_scale_value);
    acceleration[2] = process_acceleration(mpu.az,acc_scale_value);

    acceleration[0] = filter1.Output(acceleration[0]);
    acceleration[1] = filter2.Output(acceleration[1]);
    acceleration[2] = filter3.Output(acceleration[2]);

    omega[0] = filter4.Output(omega[0]);
    omega[1] = filter5.Output(omega[1]);
    omega[2] = filter6.Output(omega[2]);

    angle_acc[0] = atan2(acceleration[1], sqrt(acceleration[2] * acceleration[2] + acceleration[0] * acceleration[0]));
    angle_acc[1] = atan2(acceleration[0], sqrt(acceleration[2] * acceleration[2] + acceleration[1] * acceleration[1]));

    angle[0] = (0.98f * (angle[0] + omega[0] * dt)) + (0.02f * angle_acc[0]);
    angle[1] = (0.98f * (angle[1] + omega[1] * dt)) + (0.02f * angle_acc[1]);
    angle[2] = 0.0f;

    angle_deg[0] = wrap(r2d(angle[0]));
    angle_deg[1] = wrap(r2d(angle[1]));
    angle_deg[2] = wrap(r2d(angle[2]));

    
    vertical_acceleration_imu =  -(acceleration[2]*cos(angle[0])*cos(angle[1]) - acceleration[0]*sin(angle[1]) - acceleration[1]*cos(angle[1])*sin(angle[0])+9.82f);
    vertical_speed_imu = (0.4f*vertical_speed+0.6f*vertical_speed_imu) + vertical_acceleration_imu*dt;
    vertical_speed = 0.05f*vertical_speed_BMP+0.95f*vertical_speed_imu;
}

void get_pressure_data() {
    if (bmp280.getTempPres(temperature_BMP, pressure)) {
    }
    pressure = filter9.Output(pressure);
    altitude_BMP = (pow(1013.25/pressure,1/5.257)-1)*(temperature+273.15)/0.0065;
    altitude_BMP = filter7.Output(altitude_BMP);
    if (first_data) {
        vertical_speed_BMP = 0;
        first_data = 0;
    } else {
        vertical_speed_BMP = (altitude_BMP - altitude_BMP_prev)/dt; // m/s
    }
    vertical_speed_BMP = filter10.Output(vertical_speed_BMP);
    altitude_BMP_prev = altitude_BMP;
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

    roll_rc = channel_norm[0];
    pitch_rc = channel_norm[1];
    thrust_rc = channel_norm[2];
    yaw_rc = channel_norm[3];
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

void print_omega_data() {
    Serial.print(F("OmgX:"));
    Serial.print(omega[0]);
    Serial.print(F(",OmgY:"));
    Serial.print(omega[1]);
    Serial.print(F(",OmgZ:"));
    Serial.print(omega[2]);
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

void print_raw_acc_data() {
    Serial.print(F("aXr:"));
    Serial.print(mpu.ax);
    Serial.print(F(",aYr:"));
    Serial.print(mpu.ay);
    Serial.print(F(",aZr:"));
    Serial.print(mpu.az);
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
    Serial.println(F(","));
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

void print_pressure_data() {
    Serial.print(F("temp:"));
    Serial.print(temperature,4);                    // Display the results    
    Serial.print(F(",press:"));
    Serial.print(pressure);    
    Serial.print(F(",alt:"));
    Serial.print(altitude_BMP);
    delimiter();
}

void print_vertical_speed() {
    
    Serial.print(F("vertical_speed_BMP:"));
    Serial.print(vertical_speed_BMP);
    delimiter();

    Serial.print(F("altitude_BMP:"));
    Serial.print(altitude_BMP);
    delimiter();

    Serial.print(F("vertical_acceleration_imu:"));
    Serial.print(vertical_acceleration_imu);
    delimiter();

    Serial.print(F("vertical_speed_imu:"));
    Serial.print(vertical_speed_imu);
    delimiter();

    Serial.print(F("vertical_speed:"));
    Serial.print(vertical_speed);
    delimiter();
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
        return 2;
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
    Serial.print(dt*1e3);
    delimiter();
}

void print_frequency() {
    Serial.print(F("f:"));
    Serial.print(1/dt);
    delimiter();
}

void delimiter() {
    Serial.print(comma[0]);
}
