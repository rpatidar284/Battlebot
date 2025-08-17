#include <Servo.h>
#include <Wire.h>
#include <MPU9250.h>

// Pin Configurations
const int DRIVE_CHANNEL = 4;  // Forward/Reverse input pin
const int TURN_CHANNEL = 6;   // Turning input pin
const int LEFT_ESC = 8;        // Left drive PWM
const int RIGHT_ESC = 10;      // Right drive PWM

// Control Parameters
const int MAX_MOTOR = 180;
const int MIN_MOTOR = 0;
const int NEUTRAL = 90;

// Kalman Filter Variables
float kalmanDriveEstimate = 0.0;
float kalmanDriveErrorCovariance = 1.0;
float kalmanTurnEstimate = 0.0;
float kalmanTurnErrorCovariance = 1.0;

// Global Objects
Servo leftESC, rightESC;
MPU9250 mpu;

void setup() {
    Serial.begin(9600);
    Wire.begin();

    // Initialize MPU-9250
    if (mpu.setup(0x68)) { // Default I2C address
        Serial.println("MPU-9250 connection successful");
    } else {
        Serial.println("Failed to connect to MPU-9250");
    }

    // Attach ESCs to respective pins
    leftESC.attach(LEFT_ESC);
    rightESC.attach(RIGHT_ESC);

    // Set initial motor states
    leftESC.write(NEUTRAL);
    rightESC.write(NEUTRAL);
}

void loop() {
    // Read drive and turn inputs from receiver
    int rawDrive = map(pulseIn(DRIVE_CHANNEL, HIGH), 1000, 2000, -100, 100);
    int rawTurn = map(pulseIn(TURN_CHANNEL, HIGH), 1000, 2000, -100, 100);

    // Apply Kalman filter to smooth inputs
    float filteredDrive = kalmanFilter(rawDrive, &kalmanDriveEstimate, &kalmanDriveErrorCovariance, 1e-3, 0.1);
    float filteredTurn = kalmanFilter(rawTurn, &kalmanTurnEstimate, &kalmanTurnErrorCovariance, 1e-3, 0.1);

    // Apply deadband to filtered inputs
    int drive = applyDeadband((int)filteredDrive);
    int turn = applyDeadband((int)filteredTurn);

    // Calculate motor speeds based on drive and turn inputs
    int leftSpeed = drive + turn;
    int rightSpeed = drive - turn;

    leftSpeed = map(constrain(leftSpeed, -100, 100), -100, 100, MIN_MOTOR, MAX_MOTOR);
    rightSpeed = map(constrain(rightSpeed, -100, 100), -100, 100, MIN_MOTOR, MAX_MOTOR);

    leftESC.write(leftSpeed);
    rightESC.write(rightSpeed);
}

int applyDeadband(int value) {
    if (abs(value) < 10) return 0;
    return value;
}

float kalmanFilter(float measurement, float* estimate, float* errorCovariance, float processVariance, float measurementVariance) {
    float prediction = *estimate;
    float predictionCovariance = *errorCovariance + processVariance;

    float kalmanGain = predictionCovariance / (predictionCovariance + measurementVariance);
    *estimate = prediction + kalmanGain * (measurement - prediction);
    *errorCovariance = (1 - kalmanGain) * predictionCovariance;

    return *estimate;
}
