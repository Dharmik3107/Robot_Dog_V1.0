#include <Arduino.h>
#include <Stepper.h>
#include <ESP32Servo.h>
#include <DogTest.h>
// #include <WiFi.h>

// const int stepsPerRevolution = 200;

// Leg Back Left
const int potPinBLL = 33;
const int stepPinBLL = 25;
const int dirPinBLL = 26;
const int servoPinBLL = 27;

// Leg Front Left
const int potPinFLL = 32;
const int stepPinFLL = 14;
const int dirPinFLL = 12;
const int servoPinFLL = 13;

// Leg Back Right
const int potPinBRL = 35;
const int stepPinBRL = 5;
const int dirPinBRL = 18;
const int servoPinBRL = 04;

// Leg Front Right
const int potPinFRL = 34;
const int stepPinFRL = 15;
const int dirPinFRL = 2;
const int servoPinFRL = 19;

// Wifi Cradentials
const char *ssid = "robotic_dog";
const char *password = "rdog@0212";

// Constants
bool isRotated = false;
int standValue = 40;
bool isSeated = false;
bool isStanding = false;

// Stepper myStepperBLL(stepsPerRevolution, stepPinBLL, dirPinBLL);
// Stepper myStepperFLL(stepsPerRevolution, stepPinFLL, dirPinFLL);

// Servo myServoBLL;
// Servo myServoFLL;

// bool isHomingDone = false;
bool towardsMin = true;
bool towardsMax = false;

RobotDog myDog;

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

void setup()
{
  Serial.begin(9600);

  // // Wifi Setup
  // WiFi.mode(WIFI_STA);
  // WiFi.begin(ssid, password);
  // Serial.print("Connecting to Wifi...");

  // while (WiFi.status() != WL_CONNECTED)
  // {
  //   Serial.print(".");
  // }

  // if (WiFi.status() == WL_CONNECTED)
  // {
  // Serial.println("\nConnected to the WiFi network");
  // Serial.print("Local ESP32 IP: ");
  // Serial.println(WiFi.localIP());

  // Stepper pin definitions
  myDog.LeftLegStepper.attach(stepPinBLL, dirPinBLL);
  myDog.LeftHandStepper.attach(stepPinFLL, dirPinFLL);
  myDog.RightLegStepper.attach(stepPinBRL, dirPinBRL);
  myDog.RightHandStepper.attach(stepPinFRL, dirPinFRL);

  // Servo pin definitions
  myDog.LeftLegServo.attach(servoPinBLL);
  myDog.LeftHandServo.attach(servoPinFLL);
  myDog.RightLegServo.attach(servoPinBRL);
  myDog.RightHandServo.attach(servoPinFRL);

  // Potentiometer pin definitions
  myDog.LeftLegPot.attach(potPinBLL);
  myDog.LeftHandPot.attach(potPinFLL);
  myDog.RightLegPot.attach(potPinBRL);
  myDog.RightHandPot.attach(potPinFRL);
  // }
}

void stopAtPosition(int position)
{
}
void loop()
{
  int potvalLH = myDog.LeftHandPot.readValue();
  int potvalLL = myDog.LeftLegPot.readValue();
  int potvalRH = myDog.RightHandPot.readValue();
  int potvalRL = myDog.RightLegPot.readValue();
  // Serial.print("Value 1: ");
  // Serial.println(potvalRL);
  // // Serial.print("Value 1: ");
  // // Serial.println(potvalLG);
  if (isSeated == false)
  {
    myDog.seat();
    // if (potvalLH == 60 && potvalLL == 60 && potvalRH == 60 && potvalRL == 60)
    if (potvalLL == 60)
    {
      isSeated = true;
    }
  }
  // delay(1000);
  // if (isStanding == false && isSeated == true)
  // {
  //   myDog.stand();
  //   if (potvalLG == 40 && potvalLH == 40)
  //   {
  //     isStanding = true;
  //   }
  // }
  // delay(1000);
  // if (isStanding == true)
  // {
  //   myDog.walkForward();
  // }

  // // TODO: Stepper Rotation Check Code
  // myDog.LeftHandStepper.rotateMotor(1, 100);
  // myDog.RightHandStepper.rotateMotor(1, 100);

  // // TODO: Potentiometer Setpoint Code
  // // ! Right Hand
  // int potval = myDog.LeftHandPot.readValue();
  // Serial.print("Value Left Hand: ");
  // Serial.println(potval);
  // //! Right Leg
  // int potval2 = myDog.LeftLegPot.readValue();
  // Serial.print("Value Right Hand: ");
  // Serial.println(potval2);

  // TODO: Wifi Connection Code
}
