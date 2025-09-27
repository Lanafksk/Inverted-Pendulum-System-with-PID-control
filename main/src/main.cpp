/*
 * This ESP32 code is created by esp32io.com
 *
 * This ESP32 code is released in the public domain
 *
 * For more detail (instruction and wiring diagram), visit https://esp32io.com/tutorials/esp32-servo-motor-controlled-by-potentiometer
 */


#include <Arduino.h>
#include <Wire.h>
#include <HX711_ADC.h>
#include <ESP32Servo.h>
#include <PID_v1.h>


// IMU setup
int16_t Acc_raw_X, Acc_raw_Y, Acc_raw_Z;
int16_t Gyro_raw_X, Gyro_raw_Y, Gyro_raw_Z;
float AccX, AccY, AccZ, GyroX, GyroY, GyroZ;
float GyroX_calib, GyroY_calib, GyroZ_calib;
float AccX_calib, AccY_calib, AccZ_calib;

// long unsigned elapsedTime, time, timePrev;
long unsigned prev_time_mpu, dt_mpu;
float rad_to_deg = 180 / 3.141592654;
float Acc_angle[3];
float Gyro_angle[3] = {0, 0, 0};
float Total_angle[3] = {0, 0, 0};
int i;

// Serial configuration parameters
unsigned long curr_time = 0, prev_time_com = 0, dt_com = 50000; // time interval in us

// Loadcell setup
#define LOADCELL_DT_PIN 16
#define LOADCELL_SCK_PIN 4

// motor setup
#define PIN_ESC1         25 // ESP32 pin GPIO26 onnected to servo motor
#define PIN_ESC2         26 // ESP32 pin GPIO26 onnected to servo motor

// ESC
Servo esc1;  // create servo object to control a servo
Servo esc2;

int throttleValue1 = 0; // variable to store the servo position
int angle1 = 0; // variable to store the angle of the potentiometer

int throttleValue2 = 0; // variable to store the servo position
int angle2 = 0; // variable to store the angle of the potentiometer

#define PIN_POTENTIOMETER1 36 // ESP32 pin GPIO36 (ADC0) onnected to potentiometer
#define PIN_POTENTIOMETER2 39 // ESP32 pin GPIO36 (ADC0) onnected to potentiometer

#define calibration_factor 460 // Depends on the load cell

// PID configuration parameters
double kp = 2.50, ki = 1.25, kd = 0, input = 0, output = 0, setpoint = 0;
PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);

HX711_ADC scale(LOADCELL_DT_PIN, LOADCELL_SCK_PIN);

// #define calibration_factor 460 // Depends on the load cell

volatile boolean newDataReady;

float loadcell_data; // measurement from the loadcell in gr

// serial configuration parameters
// unsigned long curr_time = 0, prev_time = 0, dt = 50000; // time interval in us

void dataReadyISR();
void SerialDataWrite();
void gyro_signals();
void acc_signals();
float floatMap(float, float, float, float, float);

void setup()
{
  Serial.begin(9600);

  // ESC & motor setup
  esc1.attach(PIN_ESC1); // (pin, min pulse width, max pulse width in microseconds)
  esc1.writeMicroseconds(1000); // Initialize the ESC at minimum throttle (adjust if needed)

  // ESC & motor setup
  esc2.attach(PIN_ESC2); // (pin, min pulse width, max pulse width in microseconds)
  esc2.writeMicroseconds(1000); // Initialize the ESC at minimum throttle (adjust if needed)

  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(50); // Set the sample time for the PID control
  myPID.SetOutputLimits(-200, 200);

  while (!Serial)
    ;

  // Loadcell Setup
  scale.begin();
  scale.start(2000, true);
  scale.setCalFactor(calibration_factor);
  pinMode(LOADCELL_DT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(LOADCELL_DT_PIN), dataReadyISR, FALLING);

  // MPU Setup and Calibration
  Wire.setClock(400000); // set wire clock to the max (400kHz)
  Wire.begin();          // Start the wire comunication
  // delay(100);
  Wire.beginTransmission(0x68); // Start the communication with the MPU6050
  Wire.write(0x6B);             // Start the gyro in power mode
  Wire.write(0x00);             // Set the requested starting register
  Wire.endTransmission();       // End the transmission

  // MPU6050 Gyro Calibration
  // Serial.println("Begin MPU Gyro Calibration");
  for (i = 0; i < 2000; i++)
  {
    gyro_signals();
    GyroX_calib += GyroX;
    GyroY_calib += GyroY;
    GyroZ_calib += GyroZ;
  }
  GyroX_calib /= 2000;
  GyroY_calib /= 2000;
  GyroZ_calib /= 2000;
  // MPU6050 Acc Calibration
  // Serial.println("Begin MPU Acc  Calibration");
  for (i = 0; i < 2000; i++)
  {
    acc_signals();
    AccX_calib += AccX;
    AccY_calib += AccY;
    AccZ_calib += AccZ;
  }
  // For the calibration of the accelerometer, check what is the direction of the MPU. For me g is in the negative X-axis
  // Because the gravitational acceleration should not be part of the calibration
  AccX_calib = AccX_calib / 2000 - 1;
  AccY_calib = AccY_calib / 2000;
  AccZ_calib = AccZ_calib / 2000;

  // Serial.println("MPU calibrated. Press any key to start the main program.");
  // while (!Serial.available());
  Serial.read();
}

void loop()
{
  // Loadcell
  if (newDataReady)
  {
    newDataReady = 0;
    loadcell_data = scale.getData();
  }

  // MPU
  curr_time = micros();
  gyro_signals(); // Get gyro data GyroX, GyroY, GyroZ
  acc_signals();  // Get acc data AccX, AccY, AccZ

  GyroX -= GyroX_calib;
  GyroY -= GyroY_calib;
  GyroZ -= GyroZ_calib;
  AccX -= AccX_calib;
  AccY -= AccY_calib;
  AccZ -= AccZ_calib;

  dt_mpu = (curr_time - prev_time_mpu);
  prev_time_mpu = curr_time;

  Acc_angle[1] = atan2(AccZ, sqrt(pow(AccY, 2) + pow(AccX, 2))) * rad_to_deg;
  Acc_angle[2] = atan2(-AccY, sqrt(pow(AccX, 2) + pow(AccZ, 2))) * rad_to_deg;

  Gyro_angle[0] += GyroX * dt_mpu / 1000000;
  Gyro_angle[1] += GyroY * dt_mpu / 1000000;
  Gyro_angle[2] += GyroZ * dt_mpu / 1000000;

  Total_angle[0] = Gyro_angle[0];
  Total_angle[1] = .98 * (Total_angle[1] + GyroY * dt_mpu / 1000000) + .02 * Acc_angle[1];
  Total_angle[2] = .98 * (Total_angle[2] + GyroX * dt_mpu / 1000000) + .02 * Acc_angle[2];

  // Serial write
  curr_time = micros();
  if (curr_time - prev_time_com >= dt_com)
  {
    prev_time_com += dt_com;
    SerialDataWrite();
  }

  // input = Total_angle[2]; // Update the PID input
  // myPID.Compute();       // Calculate new output

  // // potentiometer control
  // int analogValue1 = analogRead(PIN_POTENTIOMETER1);
  // throttleValue1 = map(analogValue1, 0, 4095, 1000, 2000); // exact writing value to esc
  // angle1 = map(analogValue1, 0, 4095, 0, 270); // just for display the angle of potentionmeter
  // esc1.writeMicroseconds(throttleValue1);

  // // -----------------------------

  // int analogValue2 = analogRead(PIN_POTENTIOMETER2);
  // throttleValue2 = map(analogValue2, 0, 4095, 1000, 2000); // exact writing value to esc
  // angle2 = map(analogValue2, 0, 4095, 0, 270); // just for display the angle of potentionmeter
  // esc2.writeMicroseconds(throttleValue2);


  // --------------------success
  input = Total_angle[2]; // Yaw angle as input for PID
  myPID.Compute();       // Calculate new output

  // Calculate throttle values for each motor
  throttleValue1 = 1200 - output; // Adjust this as needed based on your system //left motor
  throttleValue2 = 1330 + output; // Adjust this as needed based on your system //right motor

  // Make sure throttle values are within acceptable bounds (e.g., 1000 to 2000)
  throttleValue1 = constrain(throttleValue1, 1100, 1800); // deadzone of the motor 1 is below 1100
  throttleValue2 = constrain(throttleValue2, 1220, 1800); // 

  // Set throttle values for each motor
  esc1.writeMicroseconds(throttleValue1); // Motor 1
  esc2.writeMicroseconds(throttleValue2); // Motor 2



}

void SerialDataWrite()
{
  // Serial.print("Current time: ");
  Serial.print(curr_time / 1000);
  Serial.print(" , ");
  // Serial.print("R: ");
  Serial.print(Total_angle[2]);
  Serial.print(" , ");
  // Serial.print("P: ");
  Serial.print(Total_angle[1]);
  Serial.print(" , ");
  // Serial.print("Y: ");
  Serial.print(Total_angle[0]);
  Serial.print(" , "); 
  // Serial.print("Loadcell: ");
  Serial.print(loadcell_data);
  Serial.print(" , ");
  // Serial.print("Throttle 1: ");
  Serial.print(throttleValue1);
  Serial.print(" , ");
  // Serial.print("Throttle 2: ");
  Serial.print(throttleValue2);
  Serial.print(" , ");
  // Serial.print("Ouput: ");
  Serial.println(output);
}

void gyro_signals(void)
{
  Wire.beginTransmission(0x68); // Start the communication with the MPU6050
  Wire.write(0x1A);             // Enable Low-pass filter
  Wire.write(0x05);
  Wire.endTransmission(); // End Transmission
  Wire.beginTransmission(0x68);
  Wire.write(0x1B); // Change Gyro sensitivity
  Wire.write(0x8);
  Wire.endTransmission();
  Wire.beginTransmission(0x68); // Access register storing Gyro data
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6); // Read the Gyro data
  Gyro_raw_X = Wire.read() << 8 | Wire.read();
  Gyro_raw_Y = Wire.read() << 8 | Wire.read();
  Gyro_raw_Z = Wire.read() << 8 | Wire.read();
  GyroX = (float)Gyro_raw_X / 131.0;
  GyroY = (float)Gyro_raw_Y / 131.0;
  GyroZ = (float)Gyro_raw_Z / 131.0;
}

void acc_signals(void)
{
  Wire.beginTransmission(0x68); // Start the communication with the MPU6050
  Wire.write(0x1A);             // Enable Low-pass filter
  Wire.write(0x05);             // Enable Low-pass filter - 10Hz-ish
  Wire.endTransmission();       // End Transmission
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); // Access register storing Acc data
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6); // Read the Acc data
  Acc_raw_X = Wire.read() << 8 | Wire.read();
  Acc_raw_Y = Wire.read() << 8 | Wire.read();
  Acc_raw_Z = Wire.read() << 8 | Wire.read();
  AccX = (float)Acc_raw_X / 16384.0;
  AccY = (float)Acc_raw_Y / 16384.0;
  AccZ = (float)Acc_raw_Z / 16384.0;
}

float floatMap(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void dataReadyISR()
{
  if (scale.update())
    newDataReady = 1;
}
