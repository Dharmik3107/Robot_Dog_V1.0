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

    void rotateMotor(bool direction, int steps)
    {
        if (direction == true)
        {
            digitalWrite(this->dirPin, HIGH);
        }
        else
        {
            digitalWrite(this->dirPin, LOW);
        }
        for (int i = 0; i <= steps; i++)
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
        value = map(value, 0, 4055, 0, 100);
        return value;
    }
};

class RobotDog
{
private:
    const int initAngle = 110;
    // double Setpoint = 85; // Example setpoint
    // double Input;
    // double Output;
    // double Kp = 50;
    // double Ki = 0.1;
    // double Kd = 0.1;

    // PID pid;

    // void initPID()
    // {
    //     pid.SetMode(AUTOMATIC);         // Set PID to automatic mode
    //     pid.SetOutputLimits(-255, 255); // Set output limits according to your needs
    //     pid.SetSampleTime(10);          // Set PID sample time in milliseconds (adjust as needed)
    //     pid.SetTunings(Kp, Ki, Kd);     // Set PID tunings
    // }

    // void moveToPosition(LegPot &pot, LegStepper &stepper, LegServo &servo, int position, int angle)
    // {
    //     int currentPos = pot.readValue();
    //     if (currentPos != position)
    //     {
    //         if (currentPos < position)
    //         {
    //             stepper.rotateMotor(clockwise);
    //         }
    //         else if (currentPos > position)
    //         {
    //             stepper.rotateMotor(counterClockwise);
    //         }
    //     }
    //     else if (currentPos == position)
    //     {
    //         stepper.stopRotating();
    //     }
    //     servo.rotateServo(initAngle);
    // }

public:
    const int initValue = 35;
    const int seatingValue = 75;

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

    int calculateSteps(int currentPos, int targetPos)
    {
        int potDiff;
        if (currentPos > targetPos)
        {
            potDiff = currentPos - targetPos;
        }
        else
        {
            potDiff = targetPos - currentPos;
        }
        return int(potDiff * 37.5 * 2);
    }

    void moveLegMotors(LegStepper &stepper, LegServo &servo, LegPot &pot, int targetPos, int angle)
    {
        int curPos = pot.readValue();
        if (curPos < targetPos)
        {
            stepper.rotateMotor(0, calculateSteps(curPos, targetPos));
        }
        else
        {
            stepper.rotateMotor(1, calculateSteps(curPos, targetPos));
        }
        servo.rotateServo(angle);
    }

    void seat()
    {
        moveLegMotors(LeftLegStepper, LeftLegServo, LeftLegPot, 60, 65);
        // moveLegMotors(LeftHandStepper, LeftHandServo, LeftHandPot, 60, 65);
        // moveLegMotors(RightLegStepper, RightLegServo, RightLegPot, 60, 65);
        // moveLegMotors(RightHandStepper, RightHandServo, RightHandPot, 60, 65);
    }

    void stand()
    {
        moveLegMotors(LeftLegStepper, LeftLegServo, LeftLegPot, 40, 110);
        moveLegMotors(LeftHandStepper, LeftHandServo, LeftHandPot, 40, 110);
    }

    void walkForward()
    {
        moveLegMotors(LeftLegStepper, LeftLegServo, LeftLegPot, 35, 90);
        delay(100);
        moveLegMotors(LeftLegStepper, LeftLegServo, LeftLegPot, 35, 110);
        delay(500);
        moveLegMotors(LeftHandStepper, LeftHandServo, LeftHandPot, 35, 90);
        delay(100);
        moveLegMotors(LeftHandStepper, LeftHandServo, LeftHandPot, 35, 110);
        delay(100);
        moveLegMotors(LeftLegStepper, LeftLegServo, LeftLegPot, 40, 900);
        delay(100);
        moveLegMotors(LeftLegStepper, LeftLegServo, LeftLegPot, 40, 110);
        delay(500);
    }
};

#endif