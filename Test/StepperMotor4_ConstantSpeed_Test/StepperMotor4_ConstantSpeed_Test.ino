// ConstantSpeed.pde
// -*- mode: C++ -*-
//
// Shows how to run AccelStepper in the simplest,
// fixed speed mode with no accelerations
/// \author  Mike McCauley (mikem@airspayce.com)
// Copyright (C) 2009 Mike McCauley
// $Id: ConstantSpeed.pde,v 1.1 2011/01/05 01:51:01 mikem Exp mikem $

#define DEBUG 1

#include <Debug.h>
#include <StepperMotor_4.h>

StepperMotor_4 stepper(StepperMotor_4::HALF4WIRE, 8, 9, 10, 11); // Defaults to AccelStepper::FULL4WIRE (4 pins) on 2, 3, 4, 5


void setup()
{
  Serial.begin(115200);

  DEBUG_PRINTLN("");
  DEBUG_PRINTLN("StepperMotor4_ConstantSpeed_Test");
  DEBUG_PRINTLN("");
   
  stepper.setAcceleration(120);
  stepper.setCurrentSpeed(-1000);  
  stepper.setTargetSpeed(1000);
}

void loop()
{  
   stepper.run();
}
