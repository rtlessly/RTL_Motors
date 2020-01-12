/*******************************************************************************
 MovementController2D.h

 A 2-dimensional movement controller for wheeled vehicles. This movement controller
 differs from the previous movement controller in that it does not attempt to
 do measured turns or spins. Instead, it defers that function to an IMU or similar
 device to measure turns and spins. This controller merely allows you to control 
 the turn tightness or spin speed.
 
 R. Terry Lessly, May 2017
*******************************************************************************/
#define DEBUG 0

#include <Arduino.h>
#include <RTL_Debug.h>

#include "MovementController2D.h"


DEFINE_CLASSNAME(MovementController2D);


MovementController2D::MovementController2D(IMotorController2& leftController, IMotorController2& rightController)
: _leftMotor(leftController), _rightMotor(rightController)
{
    _trackWidth = 0; 
    _wheelDiameter = 0; 
    _wheelCircumference = 0;
    _wasMoving = false;
    _wasTurning = false;
}


void MovementController2D::Begin() 
{
    _leftMotor.Initialize();
    _rightMotor.Initialize();
}


void MovementController2D::WheelDiameter(float value) 
{ 
    _wheelDiameter = value; 
    _wheelCircumference = PI * _wheelDiameter;
}


void MovementController2D::Poll()
{
    //TRACE(Logger(_classname_, this) << F("Poll") << endl); 

	if (_isSpinning)
	{
		auto leftCount = _leftMotor.GetCount();
		auto rightCount = _rightMotor.GetCount();
		
        TRACE(Logger(_classname_, this) << F("Poll: isSpinning, leftCount=") << leftCount << F(", rightCount=") << rightCount << endl); 

		if (leftCount >= _leftSpinTarget && rightCount >= _rightSpinTarget)
		{
			_isSpinning = false;
			StopNow();
			DispatchEvent(SPIN_END_EVENT);
		}
	}
	else
	{
		auto isMoving = IsMoving();
		auto isTurning = IsTurning();

		if (isMoving ^ _wasMoving)   QueueEvent(isMoving ? START_EVENT : STOP_EVENT);
		
		if (isTurning ^ _wasTurning) QueueEvent(isTurning ? TURN_BEGIN_EVENT : TURN_END_EVENT);

		_wasMoving = isMoving;
		_wasTurning = isTurning;
	}
}


void MovementController2D::Move(int16_t speed)
{
    TRACE(Logger(_classname_, this) << F("Move: speed=") << speed << endl); 
    Speed(speed);
    _leftMotor.SetSpeed(_driveSpeed);
    _rightMotor.SetSpeed(_driveSpeed);
}


void MovementController2D::Go()
{
    TRACE(Logger(_classname_, this) << F("Go") << endl); 
    Move(_driveSpeed);
}


void MovementController2D::GoForward()
{
    TRACE(Logger(_classname_, this) << F("GoForward") << endl); 
    Move(abs(_driveSpeed));
}


void MovementController2D::GoBackward()
{
    TRACE(Logger(_classname_, this) << F("GoBackward") << endl); 
    Move(-abs(_driveSpeed));
}


void MovementController2D::Reverse()
{
    TRACE(Logger(_classname_, this) << F("Reverse") << endl); 
    Move(-_driveSpeed);
}


void MovementController2D::Stop()
{
    TRACE(Logger(_classname_, this) << F("Stop") << endl); 
    _leftMotor.Stop();
    _rightMotor.Stop();
}


void MovementController2D::StopNow()
{
    TRACE(Logger(_classname_, this) << F("StopNow") << endl); 
    _leftMotor.StopNow();
    _rightMotor.StopNow();
}


void MovementController2D::Acceleration(float acceleration)
{
    TRACE(Logger(_classname_, this) << F("Acceleration=") << acceleration << endl); 
    _leftMotor.Acceleration(acceleration);
    _rightMotor.Acceleration(acceleration);
}


void MovementController2D::Accelerate(int16_t deltaSpeed)
{
    TRACE(Logger(_classname_, this) << F("Accelerate: deltaSpeed=") << deltaSpeed << endl); 
    Move(_driveSpeed + deltaSpeed);
}


/******************************************************************************
 Perform a spin at a certain speed (in motor RPM). Positive spin amounts spin 
 left (CCW), negative values spin right (CW). 
 
 With spinning, the wheels turn in opposite directions at the same speed so that 
 the vehicle pivots about its center between the wheels. The spin speed can vary 
 from +127 (max spin speed left (CCW)) to -127 (max spin speed right (CW)). Note 
 that a spin speed of 0 indicates no spin at all. 
 
 The Spin_Enum can be used to specify standard spin speed values. In particular, 
 the Spin_Left and Spin_Right enum values can be used to specify a 'normal' left
 or right spin.
 
 NOTE: Spin speeds are constrained to relatively low values to help maintain 
       control during the spin so that accurate spin angles can be achieved.

 NOTE: Both motors are stopped before beginning the spin.
******************************************************************************/
void MovementController2D::Spin(int8_t speed)
{
    TRACE(Logger(_classname_, this) << F("Spin: speed=") << speed << endl); 

    // if the speed is 0 or close to 0 then ignore
    if (speed == 0) return;
    
    StopNow(); // Always start a spin from a stop
    //delay(500);  possibly delay a little to allow the vehicle to stop (or at least slow down a lot)

	DispatchEvent(SPIN_BEGIN_EVENT);
    _leftMotor.SetSpeed(-speed);
    _rightMotor.SetSpeed(speed);
}


/******************************************************************************
 Perform a spin through the specified angle. Positive angles spin left (CCW), 
 negative angle spin right (CW). 
 
 With spinning, the wheels turn in opposite directions at the same speed so that 
 the vehicle pivots about its center between the wheels. 

 NOTE: Both motors are stopped before beginning the spin.
******************************************************************************/
void MovementController2D::SpinThruAngle(int16_t angle)
{
    TRACE(Logger(_classname_, this) << F("SpinThruAngle: angle=") << angle << endl); 

    // if the angle is 0 or close to 0 then ignore
    if (angle == 0) return;
    
    StopNow(); // Always start a spin from a stop

    auto spinCircumfrence = _trackWidth * PI;
    auto fullSpinRevs = spinCircumfrence / _wheelCircumference; 
    auto spinRevs = fullSpinRevs * abs(angle) / 360;
	auto leftCount = _leftMotor.GetCount();
	auto rightCount = _rightMotor.GetCount();
	
    _leftSpinTarget = leftCount + (uint32_t)(spinRevs * _leftMotor.StepsPerRev());
	_rightSpinTarget = rightCount + (uint32_t)(spinRevs * _rightMotor.StepsPerRev());

    TRACE(Logger(_classname_, this) << F("SpinThruAngle: spinRevs=") << spinRevs 
	                                << F(", leftCount=") << leftCount 
	                                << F(", rightCount=") << rightCount 
	                                << F(", _leftSpinTarget=") << _leftSpinTarget 
	                                << F(", _rightSpinTarget=") << _rightSpinTarget 
	                                << endl); 
	
	_leftMotor.SetSpeed(-Spin_Normal*SIGN(angle));
    _rightMotor.SetSpeed(Spin_Normal*SIGN(angle));
	_isSpinning = true;
	DispatchEvent(SPIN_BEGIN_EVENT);
}


/******************************************************************************
 Perform a turn, specifying the 'tightness' of the turn. 
 
 A turn is preformed by slowing the 'inside' motor of the turn while the 'outside'
 motor continues running at speed. The 'inside' motor corresponds to the motor
 on the same side as the direction of the turn, and is also known as the 'turning 
 motor'. 

 The tightness argument indicates both the tightness of the turn (how sharp the 
 rate of turn is) and the direction of the turn. Positive amounts turn left (CCW)
 and negative amounts turn right (CW). The absolute value of the tightness parameter 
 indicates how much to slow the inside motor. It is interpreted as a fraction of 
 the current speed of the motor, using the following formula:
 
     turning motor speed = speed * (127 - abs(tightness)) / 127;
 
 The Turn_Enum can be used to specify standard turn rate values. In particular, 
 the Turn_Left and Turn_Right enum values can be used to specify a 'normal' left
 or right turn.
 
 Specifying a turn tightness=0 will not slow the motor at all, so no turn will occur.
 Conversely, specifying a turn tightness=+/-127 will slow the turning motor down to
 zero resulting in the tightest possible turn (without inducing a spin).
******************************************************************************/
void MovementController2D::Turn(int8_t tightness)
{
    TRACE(Logger(_classname_, this) << F("Turn: tightness=") << tightness << endl); 

    // if the tightness is 0 or close to 0 then ignore
    if (tightness == 0) return;

    // Determine which motor is inner and outer for turn
    IMotorController2& innerMotor = (tightness < 0) ? _rightMotor : _leftMotor;
    IMotorController2& outerMotor = (tightness < 0) ? _leftMotor : _rightMotor;

    // Calculate motor speed for inner motor
    int turnSpeed = (_driveSpeed * (127 - abs(tightness))) / 127; 

    innerMotor.SetSpeed(turnSpeed);
    outerMotor.SetSpeed(_driveSpeed);
}


bool MovementController2D::IsMoving() 
{
	auto leftRunning = _leftMotor.IsRunning();
	auto rightRunning =_rightMotor.IsRunning();
	
    //Logger(_classname_, __func__, this) << F("L=") << leftRunning << F(", R=") << rightRunning << endl;
		
	return (leftRunning || rightRunning); 
};


