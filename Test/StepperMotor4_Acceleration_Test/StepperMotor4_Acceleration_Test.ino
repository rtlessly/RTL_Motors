// ConstantSpeed.pde
// -*- mode: C++ -*-
//
// Shows how to run AccelStepper in the simplest,
// fixed speed mode with no accelerations
/// \author  Mike McCauley (mikem@airspayce.com)
// Copyright (C) 2009 Mike McCauley
// $Id: ConstantSpeed.pde,v 1.1 2011/01/05 01:51:01 mikem Exp mikem $

//#define DEBUG 1
#include <Debug.h>
#include <StepperMotor_4.h>

StepperMotor_4 stepper(StepperMotor_4::HALF4WIRE, 8, 9, 10, 11); // Defaults to StepperMotor_4::FULL4WIRE (4 pins) on 2, 3, 4, 5


long stepsPerMotorRev = 64;
float gearRatio = 64; // 63.65;
long stepsPerShaftRev = stepsPerMotorRev * gearRatio;
long stepsToTurn = 5 * stepsPerShaftRev;

void setup()
{  
  Serial.begin(115200);
  DEBUG_PRINTLN2("stepsPerMotorRev=",stepsPerMotorRev);
  DEBUG_PRINTLN2("gearRatio=",gearRatio);
  DEBUG_PRINTLN2("stepsPerShaftRev=",stepsPerShaftRev);
  DEBUG_PRINTLN("");
  
  stepper.setAcceleration(250);
  stepper.setTargetSpeed(1000);

  DEBUG_PRINTLN3("acceleration=", stepper.acceleration(), " steps/sec/sec");
  DEBUG_PRINTLN3("targetSpeed=",  stepper.targetSpeed(),  " steps/sec");
  DEBUG_PRINTLN("");
}


int stepNumber = 1;

void loop()
{  
  stepper.run();

  if (stepper.currentSpeed() == stepper.targetSpeed() && stepNumber < 8)
  {
    DEBUG_PRINTLN5(millis(), ": speed=", stepper.currentSpeed(), ", motor position=", stepper.currentPosition());
    DEBUG_PRINTLN("");

    switch (stepNumber)
    {
      case 1:
        DEBUG_PRINTLN2(millis(), ": running for 2000 steps");
        stepper.setCurrentPosition(0);
        stepper.runToNewPosition(2000);
        break;  

      case 2:
        DEBUG_PRINTLN("");
        DEBUG_PRINTLN2(millis(), ": Accelerating to 1500 steps/sec");
        stepper.setCurrentPosition(0);
        stepper.setTargetSpeed(1500);    
        break;

      case 3:
        DEBUG_PRINTLN2(millis(), ": running for 2000 steps");
        stepper.setCurrentPosition(0);
        stepper.runToNewPosition(2000);
        break;  

      case 4:
        DEBUG_PRINTLN("");
        DEBUG_PRINTLN2(millis(), ": Deccelerating to 500 steps/sec");
        stepper.setCurrentPosition(0);
        stepper.setTargetSpeed(500);    
        break;

//      case 5:
//        DEBUG_PRINTLN2(millis(), ": running for 2000 steps");
//        stepper.setCurrentPosition(0);
//        stepper.runToNewPosition(2000);
//        break;  
//
//      case 6:
//        DEBUG_PRINTLN("");
//        DEBUG_PRINTLN2(millis(), ": Deccelerating to 0 steps/sec");
//        stepper.setCurrentPosition(0);
//        stepper.setTargetSpeed(0);    
//        break;
    }
    
    stepNumber++;
  }
}

