#define DEBUG 1

#include <Arduino.h>
#include <Common.h>
#include <Debug.h>
#include <AF_MotorShield.h>
#include <StepperMotorController.h>


// Create the motor shield object with the default I2C address
AF_MotorShield AFMS; 

// Get a stepper motor
AF_StepperMotor* motor1 = AFMS.GetStepperMotor(0, 200);

// Create a StepperMotorController
StepperMotorController controller(motor1);


void setup()
{  
  Serial.begin(115200);
  
  AFMS.Begin();
  motor1->Mode(IStepperMotor::INTERLEAVE);
  controller.Acceleration(100);
}


int stepNumber = 0;
int direction = 1;
unsigned long endTime;

void loop()
{ 
    controller.Run();

    switch (stepNumber)
    {
      case 0:
        stepNumber = 1;
        DebugLog("");
        DebugLog("Step 1: Run for 1500 steps");
        controller.Position(0);
        controller.TargetSpeed(200*direction);
        controller.RunToPosition(1500*direction);
        break;  

      case 1:
        //if (!controller.IsRunning()) 
        if (controller.IsAtTargetPosition())
        {
          stepNumber = 2;
          DebugLog("");
          DebugLog("Step 2: Accelerating to 400 steps/sec");
          controller.TargetSpeed(400*direction);
        }        
        break;  

      case 2:
        if (controller.IsAtTargetSpeed())
        {
          stepNumber = 3;
          DebugLog("");
          DebugLog("Step 3: Running at constant speed for 3 seconds");
          endTime = millis() + 3000;
        }
        break;

      case 3:
        if (millis() > endTime)
        {
          stepNumber = 4;
          DebugLog("");
          DebugLog("Step 4: Decellerating to zero with Stop()");
          controller.Stop();
        }
        break;  

      case 4:
        // When the motor stops then transistion to the next step
        if (!controller.IsRunning()) 
        {
          stepNumber = 0;
          DebugLog("");
          DebugLog("Step 0: Starting over after 2 seconds");
          delay(2000);
          direction = -direction;
        }
        break;  
    }
}

