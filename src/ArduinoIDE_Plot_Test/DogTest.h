#ifndef DogTest_h
#define DogTest_h

#include <Arduino.h>
#include <Stepper.h>
#include <ESP32Servo.h>
#include <PID_v1.h>

const int stepsPerRevolution = 800;
const bool clockwise = true;
const bool counterClockwise = false;

class LegStepper
{
private:
    Stepper *stepper;
    int stepPin;
    int dirPin; // The stepper motor itself
public:
    void attach(int stepPin, int dirPin)
    {
        this->stepPin = stepPin;
        this->dirPin = dirPin;
        stepper = new Stepper(stepsPerRevolution, stepPin, dirPin);
    }

    void rotateMotor(bool direction)
    {
        if (direction == true)
        {
            digitalWrite(this->dirPin, HIGH);
        }
        else
        {
            digitalWrite(this->dirPin, LOW);
        }
        for (int i = 0; i <= stepsPerRevolution; i++)
        {
            digitalWrite(this->stepPin, HIGH);
            delayMicroseconds(400);
            digitalWrite(this->stepPin, LOW);
            delayMicroseconds(400);
        }
    }
    void stopRotating()
    {
        digitalWrite(this->stepPin, LOW);
    }
};

class LegServo
{
private:
    Servo servo;
    int servoPin;

public:
    void attach(int servoPin)
    {
        this->servoPin = servoPin;
        this->servo.attach(servoPin);
    }
    void rotateServo(int angle)
    {
        angle = constrain(angle, 0, 180);
        int pulseWidth = map(angle, 0, 180, 500, 2400);
        servo.writeMicroseconds(pulseWidth);
    }
};

class LegPot
{
private:
    int analogPin;

public:
    void attach(int analogPin)
    {
        this->analogPin = analogPin;
    }

    int readValue()
    {
        int value = analogRead(this->analogPin);
        value = map(value, 0, 4055, 0, 200);
        return value;
    }
};

class RobotDog
{
private:
    const int initAngle = 110;
    double Setpoint = 65; // Example setpoint
    double Input;
    double Output;
    double Kp = 3;
    double Ki = 0.3;
    double Kd = 0.1;

    PID pid;

    void initPID()
    {
        pid.SetMode(AUTOMATIC);         // Set PID to automatic mode
        pid.SetOutputLimits(-255, 255); // Set output limits according to your needs
        pid.SetSampleTime(10);          // Set PID sample time in milliseconds (adjust as needed)
        pid.SetTunings(Kp, Ki, Kd);     // Set PID tunings
    }

    void moveToPosition(LegPot &pot, LegStepper &stepper, LegServo &servo, int position, int angle)
    {
        int currentPos = pot.readValue();
        if (currentPos != position)
        {
            if (currentPos < position)
            {
                stepper.rotateMotor(clockwise);
            }
            else if (currentPos > position)
            {
                stepper.rotateMotor(counterClockwise);
            }
        }
        servo.rotateServo(initAngle);
    }

public:
    const int initValue = 75;
    const int minValue = 65;
    const int maxValue = 75;
    bool towardsMax = false;
    bool towardsMin = true;

    LegStepper LeftHandStepper;
    LegStepper RightHandStepper;
    LegStepper LeftLegStepper;
    LegStepper RightLegStepper;

    LegServo LeftHandServo;
    LegServo RightHandServo;
    LegServo LeftLegServo;
    LegServo RightLegServo;

    LegPot LeftHandPot;
    LegPot RightHandPot;
    LegPot LeftLegPot;
    LegPot RightLegPot;

    RobotDog() : pid(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT) {}
    void init()
    {
        moveToPosition(LeftHandPot, LeftHandStepper, LeftHandServo, initValue, initAngle);
        moveToPosition(RightHandPot, RightHandStepper, RightHandServo, initValue, initAngle);
        moveToPosition(LeftLegPot, LeftLegStepper, LeftLegServo, initValue, initAngle);
        moveToPosition(RightLegPot, RightLegStepper, RightLegServo, initValue, initAngle);
    }
    // TODO: Walk Forward Sequence
    void moveLegForward(LegStepper &stepper, LegServo &servo, LegPot &pot, int position)
    {
        initPID();
        int currentPos = pot.readValue();
        Input = currentPos;
        pid.Compute();
        // if (currentPos >= position)
        // {
            if (Output > 0)
            {
                stepper.rotateMotor(counterClockwise);
            }
        // }
        if (currentPos == position)
        {
            stepper.stopRotating();
        }
    }
    void walkForward()
    {
        if (LeftLegPot.readValue() == maxValue && RightHandPot.readValue() == maxValue)
        {
        }
    }
};

#endif