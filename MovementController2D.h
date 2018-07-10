/*******************************************************************************
 MovementController2D.h

 A 2-dimensional movement controller for wheeled vehicles. This movement controller
 differs from the previous movement controller in that it does not attempt to
 do measured turns or spins. Instead, it defers that function to an IMU or similar
 device to measure turns and spins. This controller merely allows you to control 
 the turn tightness or spin speed.
 
 R. Terry Lessly, May 2017
*******************************************************************************/
#ifndef _MovementController2D_H_
#define _MovementController2D_H_

#include <inttypes.h>
#include <RTL_Stdlib.h>
#include <RTL_Math.h>
#include <EventSource.h>

#include "IMotorController2.h"


enum Turn_Enum
{
    Turn_None   =   0,
    Turn_Slight =  16,
    Turn_Slow   =  32,
    Turn_Normal =  64,
    Turn_Sharp  =  96,
    Turn_Max    = 127,
    Turn_Left   =  64,
    Turn_Right  = -64,
};


enum Spin_Enum
{
    Spin_None   =   0,
    Spin_Slow   =  48,
    Spin_Normal =  64,
    Spin_Fast   =  96,
    Spin_Max    = 127,
    Spin_Left   =  64,
    Spin_Right  = -64,
};


class MovementController2D : public EventSource
{
	DECLARE_CLASSNAME;

    //*************************************************************************
    // Constants
    //*************************************************************************
    public: static const int16_t STOPPED      = 0;
    public: static const int16_t SPEED_SLOW   = 50;
    public: static const int16_t SPEED_CRUISE = 200;
    public: static const int16_t SPEED_FAST   = 600;
    public: static const int16_t SPEED_MAX    = 1000;

    public: static const EVENT_ID START_EVENT      = (EventSourceID::MovementController | EventCode::StartMotion);
    public: static const EVENT_ID STOP_EVENT       = (EventSourceID::MovementController | EventCode::StopMotion);
    public: static const EVENT_ID TURN_BEGIN_EVENT = (EventSourceID::MovementController | EventCode::TurnBegin);
    public: static const EVENT_ID TURN_END_EVENT   = (EventSourceID::MovementController | EventCode::TurnEnd);
    public: static const EVENT_ID SPIN_BEGIN_EVENT = (EventSourceID::MovementController | EventCode::SpinBegin);
    public: static const EVENT_ID SPIN_END_EVENT   = (EventSourceID::MovementController | EventCode::SpinEnd);
    public: static const EVENT_ID MOVED_EVENT      = (EventSourceID::MovementController | EventCode::Moved);
    public: static const EVENT_ID TURNED_EVENT     = (EventSourceID::MovementController | EventCode::Turned);
    
    
    //*************************************************************************
    // Constructors
    //*************************************************************************
    public: MovementController2D(IMotorController2& leftController, IMotorController2& rightController);

    //*************************************************************************
    // IPollable interface
    //*************************************************************************
    public: void Poll();

    //*************************************************************************
    // Public methods
    //*************************************************************************
    public: void Begin();

    public: void Move(int16_t speed);

    public: void Go();

    public: void GoForward();

    public: void GoBackward();

    public: void Stop();

    public: void StopNow();

    public: void Reverse();

    public: void Turn(int8_t tightness);
    
    public: void Spin(int8_t speed);
    
    public: void SpinThruAngle(int16_t angle);

    public: void Accelerate(int16_t deltaSpeed);
    
    //*************************************************************************
    // Properties
    //*************************************************************************
    public: int16_t  Speed() { return _driveSpeed; }
    public: void Speed(int16_t value) { _driveSpeed  = constrain(value, -SPEED_MAX, SPEED_MAX); }

    public: float Acceleration() { return _leftMotor.Acceleration(); } // Both motors should always have the same acceleration
    public: void  Acceleration(float acceleration);
    
    public: float WheelDiameter() { return _wheelDiameter; };
    public: void  WheelDiameter(float value); 
    
    public: float TrackWidth() { return _trackWidth; }
    public: void  TrackWidth(float value) { _trackWidth  = value; }
    
    public: bool Direction() { return SIGN(_driveSpeed); };

    public: bool IsMoving();

    public: bool IsStopped() { return !IsMoving(); };
    
    public: bool IsTurning() { return (_leftMotor.GetSpeed() != _rightMotor.GetSpeed()); };    
    
    public: bool IsForward() { return _driveSpeed > 0; };
    
    public: bool IsBackward() { return _driveSpeed < 0; };
	
	public: uint32_t LeftRevs() { };
	
	public: uint32_t RightRevs() { };

    //*************************************************************************
    // Internal methods
    //*************************************************************************

    //*************************************************************************
    // Private variables
    //*************************************************************************
    private: IMotorController2& _leftMotor;
    private: IMotorController2& _rightMotor;
    
    private: bool _wasMoving;
    
    private: bool _wasTurning;
    
    private: bool _isSpinning;

    private: uint32_t _leftSpinTarget;

    private: uint32_t _rightSpinTarget;
    
    private: int16_t _driveSpeed = 0;
    
    private: float _trackWidth = 0;
    
    private: float _wheelDiameter = 0;
    
    private: float _wheelCircumference;
 };

#endif
