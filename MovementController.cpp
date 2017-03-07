#define DEBUG 1

#include <Arduino.h>
#include <Debug.h>
#include <Common.h>
#include "MovementController.h"



static DebugHelper Debug("MovementController");

EVENT_ID MovementController::StartEvent     = EventSource::GenerateEventID();
EVENT_ID MovementController::StopEvent      = EventSource::GenerateEventID();
EVENT_ID MovementController::TurnBeginEvent = EventSource::GenerateEventID();
EVENT_ID MovementController::TurnEndEvent   = EventSource::GenerateEventID();


MovementController::MovementController(IMotorController& leftController, IMotorController& rightController)
: _leftMotor(leftController), _rightMotor(rightController)
{
    _heading = 0;
    _stepsToTurn = 0;
    _stepStart = 0;
    _xWaypoint = 0;
    _yWaypoint = 0;
    _dx = 0;
    _dy = 0;
    SetWaypoint();    // Set initial starting waypoint
}


void MovementController::Poll()
{
    static bool isMoving = false;
    
    if (IsTurning())
    {
        IMotorController& motor = (_stepsToTurn > 0) ? _rightMotor : _leftMotor;
        long position = abs(motor.Position());
        long stepsToGo = (long)abs(_stepsToTurn) - position;

        Debug.Log("%s, stepsToGo=%l", __func__, stepsToGo);
        
        if (stepsToGo <= 0)
        {
            // Done turning
            SetWaypoint();    // Set a waypoint whenever we complete a turn
            TurnComplete();
        }
    }

    bool isMovingNow = IsMoving();

    if (isMovingNow != isMoving)
    {
        isMoving = isMovingNow;

        if (isMovingNow)      // Just started moving, so post START event
        {
            DispatchEvent(StartEvent);
        }
        else                  // Just stopped moving, so post the STOP event
        {
            TurnComplete();   // Can't be turning anymore if we are stopped
            SetWaypoint();    // Set a waypoint whenever we stop
            DispatchEvent(StopEvent);
        }
    }
    
    // If moving and not turning then update position
    if (IsMoving() && !IsTurning())
    {
        UpdatePosition();
    }
}


void MovementController::Move(int newSpeed)
{
    Debug.Log("%s(%i)", __func__, newSpeed);

    _driveSpeed = constrain(newSpeed, -MAX_SPEED, MAX_SPEED);
    _leftMotor.TargetSpeed(_driveSpeed);
    _rightMotor.TargetSpeed(_driveSpeed);
}


void MovementController::Go()
{
    Move(_driveSpeed);
}


void MovementController::GoForward()
{
    Move(abs(_driveSpeed));
}


void MovementController::GoBackward()
{
    Move(-abs(_driveSpeed));
}


void MovementController::Reverse()
{
    Move(-_driveSpeed);
}


void MovementController::Stop()
{
    Debug.Log(__func__);
    _leftMotor.Stop();
    _rightMotor.Stop();
}


void MovementController::StopFast()
{
    Debug.Log(__func__);
    _leftMotor.StopFast();
    _rightMotor.StopFast();
}


void MovementController::Acceleration(float acceleration)
{
    Debug.Log("%s(%f)", __func__, acceleration);

    _leftMotor.Acceleration(acceleration);
    _rightMotor.Acceleration(acceleration);
}


void MovementController::Accelerate(int deltaSpeed)
{
    Debug.Log("%s(%i)", __func__, deltaSpeed);

    int newSpeed = _driveSpeed + deltaSpeed;

    // Don't allow Accelerate to change direction. If the new speed
    // passes through zero the force to zero.
    if (SIGN(newSpeed) != SIGN(_driveSpeed)) newSpeed = 0;

    Move(newSpeed);
}


/******************************************************************************
 Spin by a certain angle, in degrees. Spin angles can be any amount, positive 
 values spin left, negative values spin right.
 
 With spinning, the wheels turn in opposite directions so that the vehicle pivots
 about its center between the wheels. To spin by a certain angle we employ 
 geometry to calculate the distance the wheels need to rotate (in steps) to cover 
 the angle. This means we need to know 3 things: 
 
    1) The diameter of the wheels 
    2) The distance between the wheels (aka axle width)
    3) The steps per revolution of the motor

 NOTE: If the spin angle is less than what can be measured (as determined by the
       motor step resolution), or the calculated rotation steps for either motor 
       is zero, then no spin is performed.

 NOTE: Spinning stops both motors before beginning the spin, and leaves the
       motors stopped when the spin completes.
******************************************************************************/
void MovementController::Spin(float angle)
{
    Debug.Log("%s(%f)", __func__, angle);

//    // if the angle is 0 or close to 0 drive forward
//    if (-2 < angle && angle < 2) return;

    // Calculate the minimum angle that can be measure for a spin
    float minAngle = 360.0 / min(_leftMotor.StepsPerRev(), _rightMotor.StepsPerRev());
    
    // If the spin angle is less than what can be measured then bail out
    if (abs(angle) < minAngle) return;
    
    StopFast(); // Always start a spin from a stop

    int  turnSpeed  = 60;                               // Spin speed. Could be RPM or raw throttle setting, but 60 is a good comprimise for both
    float r    = _axleWidth / 2;                        // Spin radius. When spinning, the radius is 1/2 the axle width
    float d    = r * (abs(angle) * PI / 180);           // Actual, physical distance a wheel must travel to cover the angle
    float revs = d / (PI * _wheelDiameter);             // Number of revolutions to cover that distance
    long leftSteps  = revs * _leftMotor.StepsPerRev();  // Number of steps required for left wheel
    long rightSteps = revs * _rightMotor.StepsPerRev(); // Number of steps required for right wheel

    Debug.Log("%s: angle=%f, r=%f, d=%f, revs=%f, leftSteps=%l, rightSteps=%l, turnSpeed=%i", __func__, angle, r, d, revs, leftSteps, rightSteps, turnSpeed);

    // Bail out if the calculation for either motor returned 0 steps
    if (leftSteps == 0 || rightSteps == 0) return;

    _leftMotor.Position(0);
    _rightMotor.Position(0);
    _leftMotor.RunToPosition(leftSteps);
    _rightMotor.RunToPosition(rightSteps);

    if (angle < 0) // Spinning right
    {
        _leftMotor.TargetSpeed(turnSpeed);
        _rightMotor.TargetSpeed(-turnSpeed);
    }
    else           // Spinning left
    {
        _leftMotor.TargetSpeed(-turnSpeed);
        _rightMotor.TargetSpeed(turnSpeed);
    }
    
    // Go ahead and update the heading even before we start the spin since this
    // is the only time we have the information to do that.
    // NOTE: we don't have to set the waypoint here since a spin always ends with 
    //       a stop, and the Poll() methods sets the waypoint when a stop occurs.
    _heading = fmod(_heading + angle, 360.0);
}


/******************************************************************************
 Turn by a certain angle, in degrees. Turn angles can be any amount, positive 
 values turn left, negative values turn right.
 
 In a turn, the inside wheel (which is the wheel corresponding to the turn direction) 
 is slowed down to 1/2 of the current speed while the other (outer) wheel continues 
 turning at speed. To turn by a certain angle, we employ geometry to calculate how 
 much the outer wheel needs to rotate (in terms of motor steps) to cover the angle. 
 This means we need to know 3 things: 
 
    1) The diameter of the wheels
    2) The distance between the wheels (aka axle width)
    3) The steps per revolution of the motor
 
 The method CalculateTurnSteps() takes care of this.

 NOTE: If abs(angle) is less than 2 degrees then no turn is performed.
******************************************************************************/
void MovementController::Turn(float angle)
{
    Debug.Log("%s(%f)", __func__, abs(angle));

    // if the angle is 0 or close to 0 then ignore
    if (-2 < angle && angle < 2) return;

    const float turnSpeedFactor = 2; 

    // Determine which motor is inner and outer for turn
    IMotorController& innerMotor = (angle < 0) ? _rightMotor : _leftMotor;
    IMotorController& outerMotor = (angle < 0) ? _leftMotor : _rightMotor;

    // Calculate steps to turn for the outer motor
    _stepsToTurn = SIGN(angle)*CalculateTurnSteps(angle, outerMotor, turnSpeedFactor);

    // The outer motor continues running at the current speed, but the inner motor 
    // is slowed to half-speed for the turn.
    outerMotor.TargetSpeed(_driveSpeed);
    innerMotor.TargetSpeed(_driveSpeed/turnSpeedFactor);

    // Reset outer motor step counter to 0 so we can count the turn steps.
    outerMotor.Position(0);

    // Go ahead and update the final position even before the turn starts since 
    // this is the only time we have all the necessary information to do that.
    UpdatePositionAfterTurn(angle, abs(_stepsToTurn), outerMotor);
  
//  if (angle < 0)
//  {
//    // Turning right so slow the right motor
//    _stepsToTurn = -CalculateTurnSteps(angle, _leftMotor, turnSpeedFactor);
//    _rightMotor.TargetSpeed(_driveSpeed/turnSpeedFactor);
//    _leftMotor.TargetSpeed(_driveSpeed);
//    _leftMotor.Position(0);
//  }
//  else
//  {
//    // Turning left so slow the left motor
//    _stepsToTurn = CalculateTurnSteps(angle, _rightMotor, turnSpeedFactor);
//    _leftMotor.TargetSpeed(_driveSpeed/turnSpeedFactor);
//    _rightMotor.TargetSpeed(_driveSpeed);
//    _rightMotor.Position(0);
//  }
  
    Debug.Log("%s(_stepsToTurn=%i)", __func__, _stepsToTurn);
    DispatchEvent(TurnBeginEvent);
}


void MovementController::TurnComplete()
{
    _stepsToTurn = 0;
    DispatchEvent(TurnEndEvent);
}


//float MovementController::CalculateTurnSpeed(IMotorController& motor)
//{
//  float WD  = _wheelDiameter;
//  float TW  = _axleWidth;
//  float tr  = turnRate;
//  float s2 = abs(Speed());
//  float ratio = 1 - (tr * TW / s2);
//  float s1  = s2 * ratio;

//  s1 *= SIGN(s1);

//  return (long)s1;
//}


// Calculates the number of steps for a wheel to turn through an angle
long MovementController::CalculateTurnSteps(float angle, IMotorController& motor, float speedRatio)
{
    if (abs(speedRatio - 1) < 0.001) return 0;
    
    float WD  = _wheelDiameter;         // Wheel diameter
    float TW  = _axleWidth;             // Track width
    float SPR = motor.StepsPerRev();    // Steps per revolution of the motor
    float factor = (speedRatio) / (speedRatio - 1);
    float w   = factor * abs(angle) / 360;
    float steps = 0.5 + (w * SPR * TW) / WD;

    return (long)steps;
}


void MovementController::UpdatePosition()
{
    long  steps = _leftMotor.Position() - _stepStart;
    float s = steps * _wheelDiameter / _leftMotor.StepsPerRev();
    float theta = radians(_heading);

    _dx = s * sin(theta);
    _dy = s * cos(theta);
}


void MovementController::UpdatePositionAfterTurn(float turnAngle, long stepsToTurn, IMotorController& motor)
{
    float theta = radians(turnAngle);
    float r = (stepsToTurn * _wheelDiameter) / (theta * motor.StepsPerRev()); 
    
    _dx += r * sin(theta);
    _dy += r * (1 - cos(theta));
    _heading = fmod(_heading + turnAngle, 360.0);
}


void MovementController::SetWaypoint()
{
    _xWaypoint = Xpos();
    _yWaypoint = Ypos();
    _leftMotor.Position(0);
    _rightMotor.Position(0);
    _stepStart = 0;
    
    PrintLine("%s: xpos=%f, ypos=%f, heading=%f", __func__, Xpos(), Ypos(), _heading);
}
