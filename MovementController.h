#ifndef _MovementController_H_
#define _MovementController_H_

#include <inttypes.h>
#include <EventSource.h>
#include "MotorController.h"


class MovementController : public EventSource
{
    //*************************************************************
    // Constants
    //*************************************************************
    public: const int MAX_SPEED = 1000;
    public: const int FAST_SPEED = 600;
    public: const int CRUISE_SPEED = 200;
    public: const int SLOW_SPEED = 50;
    public: const int STOPPED = 0;

    public: static EVENT_ID StartEvent;
    public: static EVENT_ID StopEvent;
    public: static EVENT_ID TurnBeginEvent;
    public: static EVENT_ID TurnEndEvent;
    
    //*************************************************************
    // Constructors
    //*************************************************************
    public: MovementController(IMotorController& leftController, IMotorController& rightController);

    //*********************************************************************
    // IPollable interface
    //*********************************************************************
    public: void Poll();

    //*************************************************************
    // Public methods
    //*************************************************************
    public: void Move(int newSpeed);

    public: void Accelerate(int deltaSpeed);

    public: void Go();

    public: void GoForward();

    public: void GoBackward();

    public: void Stop();

    public: void StopFast();

    public: void Reverse();

    public: void Turn(float angle);
    
    public: void Spin(float angle);
    
    //*************************************************************
    // Properties
    //*************************************************************
    public: int  Speed() { return _driveSpeed; }
    public: void Speed(int value) { _driveSpeed  = constrain(value, -MAX_SPEED, MAX_SPEED); }
    private: int _driveSpeed = 0;

    public: float Acceleration() { return _leftMotor.Acceleration(); } // Both motors should always have the same acceleration
    public: void  Acceleration(float acceleration);
    
    public: float WheelDiameter() { return _wheelDiameter; }
    public: void  WheelDiameter(float value) { _wheelDiameter = value; }
    private: float _wheelDiameter = 0;
    
    public: float AxleWidth() { return _axleWidth; }
    public: void  AxleWidth(float value) { _axleWidth  = value; }
    private: float _axleWidth = 0;

    public: bool IsMoving() { return (_leftMotor.IsRunning() || _rightMotor.IsRunning()); };

    public: bool IsStopped() { return !IsMoving(); };
    
    public: bool IsForward() { return _driveSpeed > 0; };
    
    public: bool IsBackward() { return _driveSpeed < 0; };
    
    public: bool IsTurning() { return _stepsToTurn != 0; };

    public: float Xpos() { return _xWaypoint + _dx; };

    public: float Ypos() { return _yWaypoint + _dy; };
    

    //*************************************************************
    // Internal methods
    //*************************************************************
    private: void TurnComplete();
    
    private: void UpdatePosition();
    
    private: void UpdatePositionAfterTurn(float turnAngle, long stepsToTurn, IMotorController& motor);

    private: void SetWaypoint();
    
    private: long CalculateTurnSteps(float angle, IMotorController& motor, float speedRatio=1);
    
    //*************************************************************
    // Private variables
    //*************************************************************
    private: IMotorController& _leftMotor;
    private: IMotorController& _rightMotor;
    
    private: float _heading;   // 0-360 degrees
    private: int   _stepsToTurn;
    private: long  _stepStart; // Starting step count for last waypoint
    private: float _xWaypoint; // Absolute X position in centimeters of last waypoint (relative to initial starting position)
    private: float _yWaypoint; // Absolute Y position in centimeters of last waypoint (relative to initial starting position)
    private: float _dx;        // X position in centimeters from last waypoint
    private: float _dy;        // Y position in centimeters from last waypoint
 };

#endif
