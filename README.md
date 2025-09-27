# Inverted Pendulum System with PID Control

An ESP32-based inverted pendulum balancing system using PID control algorithm with IMU sensor feedback and dual motor propulsion.

## Overview

This project implements an autonomous inverted pendulum system that maintains balance using a PID (Proportional-Integral-Derivative) control algorithm. The system uses an ESP32 microcontroller to process sensor data from an MPU6050 IMU sensor and controls two ESC-driven motors to maintain equilibrium.

## Features

- **Real-time PID Control**: Implements a tunable PID controller for maintaining balance
- **IMU Sensor Integration**: Uses MPU6050 for accurate angle measurement (accelerometer + gyroscope)
- **Dual Motor Control**: Controls two ESC motors for precise movement and stabilization
- **Load Cell Integration**: Includes HX711 load cell for weight measurement (optional functionality)
- **Serial Data Monitoring**: Real-time data output for monitoring and debugging
- **Sensor Calibration**: Automatic gyroscope and accelerometer calibration on startup

## Hardware Requirements

### Main Components
- **ESP32 Development Board** - Main microcontroller
- **MPU6050** - 6-axis IMU sensor (accelerometer + gyroscope)
- **2x ESC (Electronic Speed Controllers)** - Motor control
- **2x Brushless Motors** - Propulsion system
- **HX711 Load Cell Amplifier** - Weight sensing
- **Load Cell** - Force measurement
- **2x Potentiometers** (optional) - Manual control input

### Pin Configuration

| Component | ESP32 Pin | Function |
|-----------|-----------|----------|
| ESC Motor 1 | GPIO 25 | PWM Control |
| ESC Motor 2 | GPIO 26 | PWM Control |
| HX711 DT | GPIO 16 | Load Cell Data |
| HX711 SCK | GPIO 4 | Load Cell Clock |
| Potentiometer 1 | GPIO 36 (ADC0) | Manual Input |
| Potentiometer 2 | GPIO 39 (ADC0) | Manual Input |
| MPU6050 SDA | GPIO 21 | I2C Data |
| MPU6050 SCL | GPIO 22 | I2C Clock |

## Software Dependencies

### Libraries Used
- **ESP32Servo** (v0.13.0) - Motor control via ESC
- **HX711_ADC** (v1.2.12) - Load cell data acquisition
- **PID_v1** (v1.2.1) - PID control algorithm
- **Wire** - I2C communication for MPU6050
- **Arduino** - Core framework

## PID Control System

### Control Parameters
- **Kp (Proportional)**: 2.50
- **Ki (Integral)**: 1.25
- **Kd (Derivative)**: 0.00
- **Sample Time**: 50ms
- **Output Limits**: -200 to +200

### Control Logic
The system uses the yaw angle (Total_angle[2]) from the IMU as the primary input for the PID controller. The PID output is used to adjust the throttle values of both motors differentially:

```cpp
throttleValue1 = 1200 - output; // Left motor
throttleValue2 = 1330 + output; // Right motor
```

## Sensor Fusion

The system implements a complementary filter to combine accelerometer and gyroscope data:

```cpp
Total_angle[1] = 0.98 * (Total_angle[1] + GyroY * dt/1000000) + 0.02 * Acc_angle[1];
Total_angle[2] = 0.98 * (Total_angle[2] + GyroX * dt/1000000) + 0.02 * Acc_angle[2];
```

This provides stable angle measurements by leveraging the strengths of both sensors while minimizing their individual weaknesses.

## Installation and Setup

### Prerequisites
- PlatformIO IDE or Arduino IDE with ESP32 support
- USB cable for ESP32 programming
- Proper hardware assembly as per pin configuration

### Building and Uploading

1. **Clone the repository**:
   ```bash
   git clone <repository-url>
   cd Inverted-Pendulum-System-with-PID-control
   ```

2. **Open the project**:
   - Open `ED3-WP2-main` folder in PlatformIO
   - The project configuration is defined in `platformio.ini`

3. **Build and upload**:
   ```bash
   pio run -t upload
   ```

4. **Monitor serial output**:
   ```bash
   pio device monitor -b 9600
   ```

### Calibration Process

The system automatically calibrates sensors on startup:

1. **Gyroscope Calibration**: Takes 2000 samples to determine bias
2. **Accelerometer Calibration**: Accounts for gravitational acceleration
3. **Ready State**: System is ready for operation after calibration

**Important**: Keep the system stationary during the calibration phase for optimal performance.

## Usage

### Normal Operation
1. Power on the system
2. Wait for automatic sensor calibration (keep stationary)
3. The system will begin autonomous balancing
4. Monitor performance via serial output at 9600 baud

### Serial Data Output
The system outputs the following data every 50ms:
- Current timestamp (ms)
- Roll angle (degrees)
- Pitch angle (degrees)
- Yaw angle (degrees)
- Load cell reading (grams)
- Motor 1 throttle value
- Motor 2 throttle value
- PID output value

### Manual Control (Optional)
Uncomment potentiometer control sections in the code for manual motor control during testing.

## Tuning and Optimization

### PID Tuning
Adjust the PID parameters in the code:
```cpp
double kp = 2.50, ki = 1.25, kd = 0;
```

### Motor Deadzone Settings
Modify motor constraints based on your specific ESCs:
```cpp
throttleValue1 = constrain(throttleValue1, 1100, 1800); // Motor 1 deadzone
throttleValue2 = constrain(throttleValue2, 1220, 1800); // Motor 2 deadzone
```

### Complementary Filter
Adjust the filter ratio for different response characteristics:
```cpp
// Current: 98% gyro, 2% accelerometer
Total_angle[1] = .98 * (gyro_component) + .02 * (acc_component);
```

## Troubleshooting

### Common Issues
1. **Motors not responding**: Check ESC calibration and deadzone settings
2. **Unstable balancing**: Verify PID parameters and sensor mounting
3. **Sensor drift**: Ensure proper calibration and stable power supply
4. **Communication errors**: Check I2C connections and pull-up resistors

### Debug Tips
- Monitor serial output for sensor readings
- Verify IMU orientation and mounting
- Check motor direction and throttle response
- Ensure adequate power supply for motors

## Project Structure

```
ED3-WP2-main/
├── src/
│   └── main.cpp          # Main application code
├── lib/                  # Custom libraries (if any)
├── include/              # Header files
├── test/                 # Test files
└── platformio.ini        # Project configuration
```

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test thoroughly
5. Submit a pull request

## License

This project is released under the public domain. Feel free to use, modify, and distribute as needed.

## Acknowledgments

- Based on ESP32 servo motor control examples from esp32io.com
- Utilizes standard PID control algorithms
- MPU6050 sensor fusion techniques
- Community contributions to the embedded systems field

---

**Note**: This is an educational project demonstrating control systems principles. Always ensure proper safety measures when working with motors and mechanical systems.
