//
// Shows how to run AccelStepper in the simplest,
// fixed speed mode with no accelerations
/// \author  Mike McCauley (mikem@airspayce.com)
// Copyright (C) 2009 Mike McCauley
// $Id: ConstantSpeed.pde,v 1.1 2011/01/05 01:51:01 mikem Exp mikem $

#define DEBUG 1

#include <Debug.h>
#include <StepperMotor_4.h>

StepperMotor_4 stepper(StepperMotor_4::HALF4WIRE, 8, 9, 10, 11); // Defaults to StepperMotor_4::FULL4WIRE (4 pins) on 2, 3, 4, 5


long stepsPerMotorRev = 8 * stepper.interface();
float gearRatio = 64; // 63.65;
float stepsPerShaftRev = stepsPerMotorRev * gearRatio;


long stepsToTurn(float revs) { return  revs * stepsPerShaftRev; }


void setup()
{  
  Serial.begin(115200);

  DEBUG_PRINTLN("");
  DEBUG_PRINTLN("");
  DEBUG_PRINTLN2("gearRatio=",gearRatio);
  DEBUG_PRINTLN2("stepsPerMotorRev=",stepsPerMotorRev);
  DEBUG_PRINTLN2("stepsPerShaftRev=",stepsPerShaftRev);

  float shaftRevs = 2;
  long position = stepsToTurn(shaftRevs);

  DEBUG_PRINTLN2("revolutions to turn=",shaftRevs);
  DEBUG_PRINTLN2("target position=",position);
  DEBUG_PRINTLN("");
  
  stepper.setAcceleration(120);
  stepper.setTargetSpeed(500);
  stepper.setTargetPosition(position);
}

void loop()
{
  stepper.runToPosition();  
//  stepper.enableOutputs();
//  stepper.setCurrentPosition(0);
//  stepper.setTargetPosition(stepsToTurn);
//  stepper.runToPosition();
//  stepper.disableOutputs();
//  stepsToTurn = -stepsToTurn;
}
