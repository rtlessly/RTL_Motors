#ifndef _MovementController_H_
#define _MovementController_H_

#include <inttypes.h>
#include <EventSource.h>
#include "IMotorController.h"


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

    public: static const EVENT_ID START_EVENT      = (EventSourceID::MovementController | EventCode::StartMotion);
    public: static const EVENT_ID STOP_EVENT       = (EventSourceID::MovementController | EventCode::StopMotion);
    public: static const EVENT_ID TURN_BEGIN_EVENT = (EventSourceID::MovementController | EventCode::TurnBegin);
    public: static const EVENT_ID TURN_END_EVENT   = (EventSourceID::MovementController | EventCode::TurnEnd);
    public: static const EVENT_ID MOVED_EVENT      = (EventSourceID::MovementController | EventCode::Moved);
    public: static const EVENT_ID TURNED_EVENT     = (EventSourceID::MovementController | EventCode::Turned);
    
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
    
    public: float WheelDiameter() { return _wheelDiameter; };
    public: void  WheelDiameter(float value); // { _wheelDiameter = value; };
    private: float _wheelDiameter = 0;
    
    public: float AxleWidth() { return _axleWidth; }
    public: void  AxleWidth(float value) { _axleWidth  = value; }
    private: float _axleWidth = 0;

    public: bool IsMoving() { return (_leftMotor.IsRunning() || _rightMotor.IsRunning()); };

    public: bool IsStopped() { return !IsMoving(); };
    
    public: bool IsForward() { return _driveSpeed > 0; };
    
    public: bool IsBackward() { return _driveSpeed < 0; };
    
    public: bool IsTurning() { return (_stepsToTurn != 0); };    

    //*************************************************************
    // Internal methods
    //*************************************************************
    private: void NotifyTurnComplete();
    
    private: void NotifyLinearMove();

    private: void NotifyTurnMove(float turnRadius, float turnAngle);

    //private: long CalculateTurnSteps(float angle, int stepsPerRev, float turnSpeedFactor=1);
    
    //*************************************************************
    // Private variables
    //*************************************************************
    private: IMotorController& _leftMotor;
    private: IMotorController& _rightMotor;

    private: float _wheelCircumference;
    private: float _leftStepResolution;
    private: float _rightStepResolution;    
    private: int   _stepsToTurn;
 };

#endif
