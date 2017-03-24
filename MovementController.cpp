#define DEBUG 0

#include <Arduino.h>
#include <Debug.h>
#include <RTL_Stdlib.h>
#include "MovementController.h"


static DebugHelper Debug("MovementController");


MovementController::MovementController(IMotorController& leftController, IMotorController& rightController)
: _leftMotor(leftController), _rightMotor(rightController)
{
    _stepsToTurn = 0;
    _wheelCircumference = 0;
    _leftStepResolution = 0;
    _rightStepResolution = 0;
}


void  MovementController::WheelDiameter(float value) 
{ 
    _wheelDiameter = value; 
    _wheelCircumference = PI * _wheelDiameter;
    _leftStepResolution = _wheelCircumference / _leftMotor.StepsPerRev();
    _rightStepResolution = _wheelCircumference / _rightMotor.StepsPerRev();
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
        
        if (stepsToGo <= 0) // Done turning
        {
            NotifyTurnComplete();
        }
    }

    bool isMovingNow = IsMoving();

    if (isMovingNow != isMoving)
    {
        isMoving = isMovingNow;

        if (isMovingNow)      // Just started moving, so post START event
        {
            DispatchEvent(START_EVENT);
        }
        else                  // Just stopped moving, so post the STOP event
        {
            NotifyTurnComplete();   // Can't be turning anymore if we are stopped
            DispatchEvent(STOP_EVENT);
        }
    }
    
    // If not turning then send a notification that we have moved in a straight line
    if (!IsTurning())
    {
        NotifyLinearMove();
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

 NOTE: If abs(angle) is less than 2 degrees, or if the spin steps are zero for
       either motor, then no spin is performed.

 NOTE: Both motors are stopped before beginning the spin and when the spin completes.
******************************************************************************/
void MovementController::Spin(float angle)
{
    Debug.Log("%s(%f)", __func__, angle);

    // if the angle is 0 or close to 0 then ignore
    if (-2 < angle && angle < 2) return;
    
    StopFast(); // Always start a spin from a stop

    int  turnSpeed  = 60;                               // Spin speed. Could be RPM or raw throttle setting, but 60 is a good comprimise for both
    float r    = _axleWidth / 2;                        // Spin radius. When spinning, the radius is 1/2 the axle width
    float d    = r * radians(abs(angle));               // Actual, physical distance a wheel must travel to cover the angle
    float revs = d / _wheelCircumference;               // Number of revolutions to cover that distance
    long leftSteps  = revs * _leftMotor.StepsPerRev();  // Number of steps required for left wheel
    long rightSteps = revs * _rightMotor.StepsPerRev(); // Number of steps required for right wheel

    Debug.Log("%s: angle=%f, r=%f, d=%f, revs=%f, leftSteps=%l, rightSteps=%l, turnSpeed=%i", __func__, angle, r, d, revs, leftSteps, rightSteps, turnSpeed);

    // Bail out if the calculation for either motor returned 0 steps
    if (leftSteps == 0 || rightSteps == 0) return;

    // Send a linear move notification before starting the spin
    NotifyLinearMove();

    _stepsToTurn = rightSteps;
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
    
    // Go ahead and post a turn event even before we start the spin since this
    // is the only time we have the information to do that.
    NotifyTurnMove(0, angle);
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
 
 NOTE: If abs(angle) is less than 2 degrees then no turn is performed.
******************************************************************************/
void MovementController::Turn(float angle)
{
    Debug.Log("%s(%f)", __func__, abs(angle));

    // if the angle is 0 or close to 0 then ignore
    if (-2 < angle && angle < 2) return;

    const int   turnSpeedRatio = 2; 
    const float turnSpeedFactor = turnSpeedRatio / (turnSpeedRatio - 1); 

    // Send a linear move notification before starting the turn
    NotifyLinearMove();

    // Determine which motor is inner and outer for turn
    IMotorController& innerMotor = (angle < 0) ? _rightMotor : _leftMotor;
    IMotorController& outerMotor = (angle < 0) ? _leftMotor : _rightMotor;

    // Calculate the steps to turn for the outer motor
    float turnRadius = turnSpeedFactor * _axleWidth; //(outerStepResolution * _stepsToTurn) / radians(angle); 
    float arcLength  = turnRadius * radians(angle);
    float rotations  = arcLength / _wheelCircumference;
    
    _stepsToTurn = rotations * outerMotor.StepsPerRev();
//PrintLine("ANGLE=%f TW=%f WC=%f R=%f ARC=%f ROT=%f RES=%i STEPS=%i", angle, _axleWidth, _wheelCircumference, turnRadius, arcLength, rotations, outerMotor.StepsPerRev(), _stepsToTurn);
    
    // The outer motor continues running at the current speed, but the inner motor 
    // is slowed to half-speed for the turn.
    outerMotor.TargetSpeed(_driveSpeed);
    innerMotor.TargetSpeed(_driveSpeed/turnSpeedRatio);

    // Reset outer motor step counter to 0 so we can count the turn steps.
    outerMotor.Position(0);

    // Go ahead and update the final position even before the turn starts since 
    // this is the only time we have all the necessary information to do that.    
    NotifyTurnMove(turnRadius, angle); 
    
    Debug.Log("%s(_stepsToTurn=%i)", __func__, _stepsToTurn);
    DispatchEvent(TURN_BEGIN_EVENT);
}


//// Calculates the number of steps for a wheel to turn through an angle
//long MovementController::CalculateTurnSteps(float angle, int stepsPerRev, float turnSpeedFactor)
//{
//    float w = turnSpeedFactor * abs(angle) / 360.0; // Angular speed
//    float steps = 0.5 + (stepsPerRev * w * _axleWidth) / _wheelDiameter;
//
//    return (long)steps;
//}


void MovementController::NotifyTurnComplete()
{
    _stepsToTurn = 0;
    DispatchEvent(TURN_END_EVENT);

    // Reset motor position counters
    _leftMotor.Position(0);
    _rightMotor.Position(0);
}


void MovementController::NotifyLinearMove()
{
    // This method should only be called when moving in a straight line. With
    // that assumption, then the distance traversed by either motor whould be
    // about the same. So it doesn't really matter which motor we choose to
    // calculate distance moved.
    long steps = abs(_leftMotor.Position()) * SIGN(_driveSpeed);
    
    if (steps == 0) return;
    
    float distance = steps * _leftStepResolution;

    Debug.Log("%s - distance=%f", __func__, distance);
    DispatchEvent(MOVED_EVENT, distance);

    // Reset motor position counters
    _leftMotor.Position(0);
    _rightMotor.Position(0);
}


void MovementController::NotifyTurnMove(float turnRadius, float turnAngle)
{
    PolarVector2D turn(turnRadius, turnAngle);

    Debug.Log("%s - radius=%f,angle=%f", __func__, turn.Radius, turn.Angle);
    DispatchEvent(TURNED_EVENT, &turn);
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


//void MovementController::UpdatePositionAfterTurn(float turnAngle, long stepsToTurn, int stepsPerRev)
//{
//    float theta = radians(turnAngle);
//    float turnRadius = (stepsToTurn * _wheelDiameter) / (theta * stepsPerRev()); 
//    
//    NotifyTurnMove(turnRadius, turnAngle);
//}

//void MovementController::SetWaypoint()
//{
//    _xWaypoint = Xpos();
//    _yWaypoint = Ypos();
//    _leftMotor.Position(0);
//    _rightMotor.Position(0);
//    _stepStart = 0;
//    
//    PrintLine("%s: xpos=%f, ypos=%f, heading=%f", __func__, Xpos(), Ypos(), _heading);
//}
