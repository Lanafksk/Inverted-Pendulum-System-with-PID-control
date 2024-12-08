/*
 * This ESP32 code is created by esp32io.com
 *
 * This ESP32 code is released in the public domain
 *
 * For more detail (instruction and wiring diagram), visit https://esp32io.com/tutorials/esp32-servo-motor-controlled-by-potentiometer
 */

#include <ESP32Servo.h>
#include <HX711_ADC.h>
#include <Arduino.h>
#include <PID_v1.h>
#include "ACS712.h"


#define PIN_ESC         26 // ESP32 pin GPIO26 connected to ESC servo motor
#define LOADCELL_DT_PIN 16
#define LOADCELL_SCK_PIN 4
#define V_SENSOR_PIN  34 // voltage sensor pin
// #define C_SENSOR_PIN  32 // current sensor pin 
#define C_SENSOR_PIN  13 // current sensor pin 

#define calibration_factor 430 // Depends on the load cell

Servo esc;  // create servo object to control a servo

// Loadcell setup
HX711_ADC scale(LOADCELL_DT_PIN, LOADCELL_SCK_PIN);

int throttleValue = 0; // variable to store the servo position
int throttleFix = 0; // variable to store the servo position


// check if data is ready to be read from the loadcell
volatile boolean newDataReady;

// ACS712 sensor(ACS712_05B, C_SENSOR_PIN); // set the 5A current sensor
ACS712 sensor(ACS712_30A, C_SENSOR_PIN);

// Floats for voltage and current sensors
float adc_voltage = 0.0;
float voltage = 0.0;
float current = 0.0;
int raw_current;
 
// Floats for resistor values in divider (in ohms) in Voltage sensor // do not touch (it is fixed value)
float R1 = 30000.0;
float R2 = 7500.0; 
 
// Float for Reference Voltage
// float ref_voltage = 25.2; //voltage from the battery
// default reference value of the sensor is 5V but I can modify it to 3.3 when I connnect with it
 float ref_voltage = 5;  // do not touch (it is fixed value)

// variable for analog voltage sensor reading
int adc_value = 0;  

// variable to store the loadcell data
float loadcell_data; // measurement from the loadcell in gr

// variables to store the time
unsigned long curr_time = 0, prev_time = 0, dt = 50000; // time interval in us

// PID configuration parameters
double kp = 0.062, ki = 0.058, kd = 0, input = 0, output = 0, setpoint = 1;
PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);

// functions declarations
void dataReadyISR();
void SerialDataWrite();
void calibrateESC();

void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);

  // initialize the ESc to control the servo
  esc.attach(PIN_ESC, 1000, 2000);
  esc.writeMicroseconds(1000); // send minimum throttle signal to the ESC
  delay(2000); // wait for the ESC to recognize the minimum throttle signal

  int zero = sensor.calibrate();

  while (!Serial);
  scale.begin();
  scale.start(2000, true);
  scale.setCalFactor(calibration_factor);
  pinMode(LOADCELL_DT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(LOADCELL_DT_PIN), dataReadyISR, FALLING);

  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(50); // Set the sample time for the PID control
  myPID.SetOutputLimits(0, 180);

  // calibrateESC(); // calibrate the ESC (only the first time)
}

void loop() {

    adc_value = analogRead(V_SENSOR_PIN);   // read the state of the the input pin:
  adc_voltage  = (adc_value * ref_voltage) / 4096.0;  // Determine voltage at ADC input // analog read resolution 10 bi(1025)? 12bit(4096)? 
  voltage = adc_voltage / (R2/(R1+R2)) ;    // Calculate voltage at divider input

  current = sensor.getCurrentDC();  // this current will set with 2 decima
  // raw_current = analogRead(C_SENSOR_PIN); 
  // current = (3.3/2 - raw_current * 3.3 / 4096.0) / 0.066; // 3.3V is the reference voltage of the ESP32


  if (newDataReady)
  {
    newDataReady = 0;
    loadcell_data = scale.getData();
  }

  curr_time = micros();
  if (curr_time - prev_time >= dt)
  {
    prev_time += dt;
    SerialDataWrite();
  }
  

  input = loadcell_data; // Update the PID input
  myPID.Compute();       // Calculate new output
  //output = constrain(output, 0, 80); // Constrain the output to be between 0 and 180

  // throttleValue = map(output, 0, 180, 1191, 2000); // exact writing value to esc // the motor starts from 1192
  throttleValue = map(output, 0, 180, 1000, 2000); // exact writing value to esc // the motor starts from 1192
  // throttleFix = constrain(throttleValue, 1150, 1300);

  esc.writeMicroseconds(throttleValue); // Send the output to the ESC
}

void dataReadyISR()
{
  if (scale.update())
    newDataReady = 1;
}

void SerialDataWrite()
{
  // Serial.print("1.Loadcell: ");
  // Serial.print(loadcell_data);
  // Serial.print("2.Setpoint: ");
  // Serial.print(setpoint);
  Serial.print("3.Input: ");
  Serial.print(input);
  Serial.print("4.Output: ");
  Serial.print(output);
  Serial.print("5.Throttle: ");
  Serial.print(throttleValue);
  // Serial.print("6.Voltage: ");
  // Serial.print(voltage, 2);
  Serial.print("Raw Current: ");
  Serial.print(raw_current);
  Serial.print("Current: ");
  Serial.print(current);
  Serial.println();
}