/*
 * Pawprint Prototyping:
 * 
 * Arduino Due
 * Libraries Used:
 * HiTorque Motor:
 * https://github.com/pololu/high-power-stepper-driver-arduino
 * DueTimer:
 * https://github.com/ivanseidel/DueTimer
 */

#include <SPI.h>
#include "SerialController.h"
#include <string.h>
//#include "StepperMotorLib.h"
//#include "StepperControllerLib.h"
#include "StepperMotorLib_TMC2208.h"
#include "StepperControllerLib_TMC2208.h"
#include "Button.h"

// State Stuff ///////////////////
//Enum for easy state name access in switch case later
typedef enum
{
  INIT,
  STBY,
  GOHOME,
  HOME,
  GOTODEGREE,
  TurnClockwise,
  TurnAntiClockwise,
  Wave1,
  Wave2
  //wave, talk, wave2, etc
}TopState; 

//volatile uint32_t intFlag = 0; //interrupt flags
//Create the state variable for switch case later
TopState StateVar;

//Debug pin
uint8_t debugPin = 21;


//Motors ///////////////////////

//Declare Arm Motor
//Declare Arm Motor Pins
StepperMotorPinNames_TMC2208 ArmMotorPins = { .directionPin = 2, .stepPin = 3, };
StepperMotorPinNames_TMC2208 * ArmPinsPtr = &ArmMotorPins;
//Arm Microstepping Mode
MicroStepModeEnum_TMC2208 StepModeEnum = MicroStep32_TMC2208;
//Arm speed in tenths of a degree per second
uint16_t armSpeed = 300;
//Arm Per phase current
//uint16_t armCurrent = 500;
//Arm limit in deci degrees. 300 == 30 degrees
uint16_t armLimitDeciDeg = 200;

//Declare Jaw Motor
//Declare Jaw motor pins
StepperMotorPinNames_TMC2208 JawMotorPins = { .directionPin = 5, .stepPin = 6, };
StepperMotorPinNames_TMC2208 * JawPinsPtr = &JawMotorPins;
//Jaw Speed
uint16_t jawSpeed = 250;
//uint16_t jawCurrent = 500;
//Jaw home angle is how far from home the position target is set to. 
//The goal is to get close to home without setting off the limit switch
uint16_t jawHomeAngle = 25;
uint16_t jawLimitDeciDeg = 230;
MicroStepModeEnum_TMC2208 JawStepEnum = MicroStep32_TMC2208;

// Serial Parser
SerialParser serialParser;
String outputBuffer;

//Declare Motor Controller
//Arm
StepperController_TMC2208 ArmController;
StepperController_TMC2208 * ArmControlPtr = &ArmController;

//Jaw
StepperController_TMC2208 JawController;
StepperController_TMC2208 * JawControlPtr = &JawController;

//Buttons //////////////////////





//Button mode; //debug button for changing mode, remove after integrating rPi talk

//Input Pin numbers
uint8_t  EmergencyStopBttnPin = 8;
//uint8_t  ArmHmLimSwPin = 20;
uint8_t  ArmOpenLimSwPin = 19;
uint8_t  JawHmLimSwPin = 18;


//uint8_t ledPin = LED_BUILTIN;
//volatile bool ledState = true;



void setup()
{
  // put your setup code here, to run once:
  //StateMachine starts at init, which finishes initializing
  //anything that didn't finish here
  StateVar = INIT;
  
  //Set Debug Pin
  pinMode(debugPin, INPUT);

  //Enable Serial and wait for it to connect.
  Serial.begin(115200);
  while (!Serial) {}

  
  //What is this pin for?
  pinMode(53, OUTPUT);

  
  

  //Init Motor Controllers:
  //Arm
  stepperControllerInit_TMC2208(
    ArmControlPtr,
    ArmPinsPtr,
    StepModeEnum,
    Clockwise_TMC2208,
    ArmOpenLimSwPin,
    //armCurrent,
    armSpeed,
    armLimitDeciDeg);
  //Jaw
  stepperControllerInit_TMC2208(
    JawControlPtr,
    JawPinsPtr,
    StepModeEnum,
    AntiClockwise_TMC2208,
    JawHmLimSwPin,
    //jawCurrent,
    jawSpeed,
    jawLimitDeciDeg);

  //set isJaw on the jaw motor to true
  setIsJaw_TMC2208(JawControlPtr, true);
  //set target angle on Jaw to be near 0
  controllerSetTargetAngle_TMC2208( JawControlPtr, jawHomeAngle);

  //attach interrupts
  //attachInterrupt(digitalPinToInterrupt(EmergencyStopBttn.pinNumber), emergencyStopBttnISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ArmController.HomeLimitSwitch.pinNumber), armOpenLimSwISR, RISING);
  //attachInterrupt(digitalPinToInterrupt(ArmHmLimitSwitch.pinNumber), armOpenLimSwISR, RISING);
  attachInterrupt(digitalPinToInterrupt(JawController.HomeLimitSwitch.pinNumber), jawHomeLimSwISR, RISING);
}


// put your main code here, to run repeatedly:
void loop()
{
  updateMotor_TMC2208(ArmControlPtr);
  updateMotor_TMC2208(JawControlPtr);
  
  updateSerial();
}

void updateSerial() {
  if (Serial.available() > 0) {
    int charToRead = Serial.read();
    if (charToRead < 0) return;

    String command = serialParser.parse(charToRead);

    if (command != SerialParser::COMMAND_NONE) {
      String response = "";
      response += command;
      response += " ";

      // --- COMMAND: IS READY? ---
      // ?R
      if (command == SerialParser::QUERY_IS_READY) {
         // Return 1 only if both motors have successfully found their home position
         if(ArmControlPtr->homeFound && JawControlPtr->limitFound ) {
             response += "1";
         } else {
             response += "0";
         }
      }
      
      // --- COMMAND: HAND HOME (STOP WAVING) ---
      // !HH
      else if (command == SerialParser::COMMAND_HAND_HOME) {
        controllerDisable_TMC2208(ArmControlPtr);
        controllerSetState_TMC2208(ArmControlPtr, M_HOMING_TMC2208);
      } 
      
      // --- COMMAND: HAND WAVE (START WAVING) ---
      // !HW
      else if (command == SerialParser::COMMAND_HAND_WAVE) {
        controllerEnable_TMC2208(ArmControlPtr);
      } 
      
      // --- COMMAND: MOUTH HOME (STOP TALKING) ---
      // !MH
      else if (command == SerialParser::COMMAND_MOUTH_HOME) {
        controllerDisable_TMC2208(JawControlPtr);
        controllerSetState_TMC2208(JawControlPtr, M_HOMING_TMC2208);
      } 
      
      // --- COMMAND: MOUTH TALK (START TALKING) ---
      // !MW
      else if (command == SerialParser::COMMAND_MOUTH_TALK) {
        controllerEnable_TMC2208(JawControlPtr);
      } 
      
      else {
        // Unknown command
        response += "?";
      }
      
      response += '\n';
      Serial.write(response.c_str(), response.length());
    }
  }
}

//here be ISR's
void armOpenLimSwISR()
{
  ArmController.HomeLimitSwitch.active = true;
}

void jawHomeLimSwISR()
{
  JawController.HomeLimitSwitch.active = true;
}