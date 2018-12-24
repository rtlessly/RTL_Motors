// StepperMotorController.cpp
// Based on AccelStepper library for Arduino, Copyright (C) 2009-2013 Mike McCauley
// See http://www.gnu.org/copyleft/gpl.html
//
// This strips down AccelStepper to support only 4-wire stepper motors, and optimizes some of the code

#define DEBUG 0

#include <Arduino.h>
#include <Debug.h>
#include <RTL_Math.h>
#include "StepperMotorController.h"


static DebugHelper Debug("StepperMotorController");


StepperMotorController::StepperMotorController(IStepperMotor* pMotor)
{
    _motor = pMotor;

    // Keeps track of motor speed
    _currentSpeed = 0.0;
    _targetSpeed = 0;

    // Keeps track of the step time intervals
    _currentStepSize = 0;
    _targetStepSize = 0.0;
    _lastStepTime = 0;

    // Keeps track of motor step position
    _currentPos = 0;
    _targetPos = 0;
    _isRunningToPosition = false;

    // Acceleration variables
    _acceleration = 0.0;
    _accelStep = 0;
    _accelStepSize = 0.0;
    _accelStepSize0 = 0.0;
}


//******************************************************************************
//******************************************************************************
// Logic to run the motor
//******************************************************************************
//******************************************************************************

void StepperMotorController::Poll()
{
    Debug.Log(this) << __func__ << endl;
    Run();
}


void StepperMotorController::Stop()
{
    TargetSpeed(0);
}


void StepperMotorController::StopFast()
{
    Debug.Log(this) << __func__ << '[' << _motor->ID() << ']' << endl;
    Speed(0);
    TargetSpeed(0);
    _isRunningToPosition = false;
}


void StepperMotorController::RunToPosition(long targetPosition)
{
    Debug.Log(this) << __func__ << '[' << _motor->ID() << F("]: targetPosition=") << targetPosition << endl;
    TargetPosition(targetPosition);
    _isRunningToPosition = true;
}


/******************************************************************************
 Runs the motor, at most one step, using acceleration.
 You must call this method continually to keep the motor moving, preferably
 in your main loop.
 If the motor is at the desired speed, the cost is very small.
 Returns true if the motor is still running.
******************************************************************************/
boolean StepperMotorController::Run()
{
    if (_isRunningToPosition)
    {
        RunToTargetPosition();
    }
    else
    {
        if (OneStep()) ComputeNewSpeed();
    }

    return _currentSpeed != 0.0;
}


/******************************************************************************
 Runs the motor, at most one step, using acceleration until the target position is reached.
 You must call this method continually to keep the motor moving, preferably
 in your main loop.
 If the motor is at the desired position this will not step the motor and the cost
 is very small.
 Returns true if the motor is still running.
******************************************************************************/
void StepperMotorController::RunToTargetPosition()
{
    Debug.Log(this) << __func__ << '[' << _motor->ID() << F("]: _currentPos=") << _currentPos
                                                       << F(", _targetPos=") << _targetPos
                                                       << F(", _currentSpeed=") << _currentSpeed
                                                       << endl;

    if (OneStep())
    {
        if (_targetPos == _currentPos)
        {
            StopFast();
        }
        else if (_acceleration != 0.0)
        {
            if (_targetSpeed != 0.0)
            {
                long stepsToGo   = abs(_targetPos - _currentPos);
                long stepsToStop = ((_currentSpeed * _currentSpeed) / (2.0 * _acceleration)); // Equation 16

                if (stepsToStop > stepsToGo) // Using > instead of >= to avoid off-by-one error
                {
                    TargetSpeed(0.0);
                }
            }

            ComputeNewSpeed();
        }

        _isRunningToPosition = (_currentSpeed != 0.0);
    }
}


/******************************************************************************
 Steps motor one step according to the current step interval.
 May not step motor if current step interval has not yet expired.
 You must call this method continually to keep the motor moving, preferably in
 your main loop.
 Returns true if a step occurred
******************************************************************************/
bool StepperMotorController::OneStep()
{
    // Early exit if don't have a step interval
    if (_currentStepSize == 0)  return false;

    unsigned long time = micros();

    // Early exit if step interval has not yet expired
    if ((time - _lastStepTime) < _currentStepSize) return false;

    // Otherwise, we are going to step the motor
    Debug.Log(this) << __func__ << '[' << _motor->ID() << F("]: Stepping Motor") << endl;

    int direction = Direction();

    // Update step position, +1 for FORWARD, -1 for BACKWARD; And last step time
    _currentPos += direction;
    _lastStepTime = time;

    // Actually step motor
    _motor->OneStep(direction);

    return true;
}


//******************************************************************************
//******************************************************************************
// Acceleration logic
//******************************************************************************
//******************************************************************************

/******************************************************************************
 Computes the interval for the next motor step taking acceleration into account.

 !!This is the key method for acceleration!!

 If there is no acceleration then the motor speed is not changed. In other words,
 setting acceleration to 0 effectively disables acceleration.

 If acceleration is not in effect, then simply set the motor speed using Speed()
 and call Run() in your main loop. This will run the motor at a constant speed
 until you set the current speed to 0 or stop calling Run().
******************************************************************************/
void StepperMotorController::ComputeNewSpeed()
{
    Debug.Log(this) << __func__ << '[' << _motor->ID() << F("]: _currentSpeed=") << _currentSpeed
                                                       << F(", _targetSpeed=") << _targetSpeed
                                                       << endl;

    // Early exit if there is no need to accelerate
    if (_acceleration == 0.0 || _currentSpeed == _targetSpeed) return;

    // If the target speed is 0 and we are at the 0-point on acceleration curve (step 0)
    // then we should stop
    if (_targetSpeed == 0 && _accelStep == 0)
    {
        Debug.Log(this) << __func__ << '[' << _motor->ID() << F("]: stopping at speed 0") << endl;
        _currentStepSize = fabs(_targetStepSize);
        _currentSpeed = _targetSpeed;
        _motor->Release();
    }
    else
    {
        // Compute acceleration direction (+1=speeding up, -1=slowing down)
        long direction = (_targetSpeed >= _currentSpeed) ? 1 : -1;

        // calculate the next step size (Cn)
        // Special case if starting at step 0 on the acceleration curve: use precomputed C0
        // Otherwise, use equation 13 for all subsequent steps. Works for speeding up and
        // slowing down, including changes in direction (CW to CCW or vis-versa).
        _accelStepSize = (_accelStep == 0) ? (_accelStepSize0 * direction)
                                           : (_accelStepSize - ((2.0 * _accelStepSize) / ((4.0 * _accelStep * direction) + 1)));

        // Compute new speed based on new step size
        float newSpeed = 1000000.0 / _accelStepSize;

        if ((direction >= 0 && newSpeed > _targetSpeed) || (direction < 0 && newSpeed < _targetSpeed))
        {
            // If we overshot the target speed then adjust to target speed
            Debug.Log(this) << __func__ << '[' << _motor->ID() << F("]: _targetSpeed=") << _targetSpeed << endl;
            _currentStepSize = fabs(_targetStepSize);
            _currentSpeed = _targetSpeed;
        }
        else
        {
            // Otherwise, just update the current speed, step size, and acceleration step number
            _currentStepSize = fabs(_accelStepSize);
            _currentSpeed = newSpeed;
            _accelStep += direction; // increment/decrement acceleration step number (n)
        }
    }

    Debug.Log(this) << __func__ << '[' << _motor->ID() << F("] exit: _currentStepSize=") << _currentStepSize
                                                       << F(", _currentSpeed=") << _currentSpeed
                                                       << F(", Direction=") << Direction()
                                                       << endl;
}


void StepperMotorController::UpdateAccelerationParameters()
{
    // Compute the starting step number (n) on the acceleration curve and
    // the starting step size, based on the current speed and acceleration
    if (_acceleration != 0)
    {
        long direction = Direction();

        _accelStep = ((_currentSpeed * _currentSpeed) / (2.0 * _acceleration)) * direction; // Equation 16
        _accelStepSize = ((_currentStepSize == 0) ? _accelStepSize0 : _currentStepSize) * direction;
    }
}


//******************************************************************************
//******************************************************************************
// Properties
//******************************************************************************
//******************************************************************************

int8_t StepperMotorController::Direction()
{
    return (_currentSpeed > 0 ? FORWARD : (_currentSpeed < 0 ? BACKWARD : STOPPED));
}


float StepperMotorController::Speed()
{
    return _currentSpeed;
}


void StepperMotorController::Speed(float value)
{
  _currentSpeed = value;
  _currentStepSize = (value == 0.0) ? 0 : fabs(1000000.0 / value);

  if (_currentSpeed == 0.0) _motor->Release();

  UpdateAccelerationParameters();
}


void StepperMotorController::TargetSpeed(float value)
{
    _targetSpeed = value;
    _targetStepSize = value != 0.0 ? (1000000.0 / value) : 0;
    UpdateAccelerationParameters();

    // Compute the next step size
    ComputeNewSpeed();
    _lastStepTime = 0; // Forces a step on next call to Run().
}


float StepperMotorController::TargetSpeed()
{
    return _targetSpeed;
}


long StepperMotorController::Position()
{
    return _currentPos;
}


// Assigns the specified value to the current motor position
void StepperMotorController::Position(long value)
{
    _targetPos = _currentPos = value;
}


long StepperMotorController::TargetPosition()
{
    return _targetPos;
}


void StepperMotorController::TargetPosition(long value)
{
    _targetPos = value;

    if (_acceleration != 0.0)
    {
        long deltaPos = _targetPos - _currentPos;
        //long deltaSpeed = _targetSpeed - _currentSpeed;

        if (SIGN(deltaPos) != SIGN(_targetSpeed)) TargetSpeed(-_targetSpeed);
        //if (SIGN(deltaPos) != SIGN(deltaSpeed)) TargetSpeed(-_targetSpeed);
        //if (_targetPos < _currentPos) TargetSpeed(-_targetSpeed);

        _lastStepTime = 0; // Forces a step on next call to Run(). micros();
    }
}


float StepperMotorController::Acceleration()
{
    return _acceleration;
}


void StepperMotorController::Acceleration(float value)
{
    if (value != 0)
    {
        value = fabs(value);

        // Compute the equivalent acceleration step number on the new acceleration curve
        // _accelStep = _accelStep * (_acceleration / value);

        // Compute the step zero size (C0) per Equation 7, with correction per Equation 15
        _accelStepSize0 = 0.676 * sqrt(2.0 / value) * 1000000.0;
    }

    // Set the new acceleration
    _acceleration = value;
}


bool StepperMotorController::IsRunning()
{
    return (_currentSpeed != 0.0 || _isRunningToPosition);
}


bool StepperMotorController::IsRunningToPosition()
{
    return (_isRunningToPosition);
}


bool StepperMotorController::IsAtTargetSpeed()
{
    return (_currentSpeed == _targetSpeed);
}


bool StepperMotorController::IsAtTargetPosition()
{
    return (_currentPos == _targetPos);
}
