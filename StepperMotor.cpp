/******************************************************************
 An Arduino driver for a 4-Wire stepper motor. This driver requires 
 4 digital pins on the Arduino. This driver can control both 
 uni-polar and bi-polar motors as the same control protocol works 
 for both.
 
 The driver supports various modes of operation for stepper motors.
 
 It is assumed that the motor is connected to the Arduino via a 
 simple pass-thru power controller such as an ULN2003 or ULN2004 
 Darlington array.
 
 Written by R. Terry Lessly 2018-12-03.
 ******************************************************************/
#define DEBUG 0

#include <Arduino.h>
#include <RTL_Debug.h>
#include <RTL_Stdlib.h>
#include "StepperMotor.h"


DEFINE_CLASSNAME(StepperMotor);


/*******************************************************************************
    STEPPER MOTORS
*******************************************************************************/

StepperMotor::StepperMotor(uint16_t stepsPerRev, uint8_t pinA1, uint8_t pinA2, uint8_t pinB1, uint8_t pinB2) 
{
    _stepsPerRev = stepsPerRev;
    _motorState.pinA1 = pinA1; 
    _motorState.pinA2 = pinA2;
    _motorState.pinB1 = pinB1; 
    _motorState.pinB2 = pinB2;

    _currentStep = 0;
	_lastStepTime = 0;
	_stepCount = 0;
	_isRunning = false;
	
	pinMode(pinA1, OUTPUT);
	pinMode(pinA2, OUTPUT);
	pinMode(pinB1, OUTPUT);
	pinMode(pinB2, OUTPUT);

    Release();
}


void StepperMotor::Poll(void) 
{
    TRACE(Logger(_classname_) << F("Poll") << endl);
	
    // Only need to step if we are running and speed is not 0
    if (_isRunning && _usPerStep > 0)
	{
		unsigned long time = micros();

		// Step motor if step interval has been reached
		if ((time - _lastStepTime) >= _usPerStep) 
		{
			OneStep(Direction());
			_lastStepTime = time;

			// If runnings steps then update step count and
			// stop if the step count has been reached.
			if (_stepCount > 0)
			{
				if (--_stepCount == 0) Release();
			}
		}
	}
}


void StepperMotor::Release(void) 
{
	Stop();
    digitalWrite(_motorState.pinA1, LOW);
    digitalWrite(_motorState.pinA2, LOW);
    digitalWrite(_motorState.pinB1, LOW);
    digitalWrite(_motorState.pinB2, LOW);
}


void StepperMotor::OneStep(int8_t dir) 
{
    uint8_t latchState = 0;

    switch(_motorState.mode)
    {
        case SINGLE:
            TRACE(Logger(_classname_) << "OneStep[" << _motorState.motorNum << F("] dir=") << dir << endl);

            // Increment/decrement step number, but constrain to 0-3. 
            // If dir=+1 then it steps 0-1-2-3 order, if dir=-1 then it steps 3-2-1-0 order
            _currentStep = ((_currentStep + dir) + 4) % 4;

            switch (_currentStep) 
            {
                case 0:
                    latchState |= 0x3; // energize coil 1+2
                    break;
                    
                case 1:
                    latchState |= 0x6; // energize coil 2+3
                    break;

                case 2:
                    latchState |= 0xC; // energize coil 3+4
                    break;
            
                case 3:
                    latchState |= 0x9; // energize coil 1+4
                    break;
            }

            break;
            
        case DOUBLE:
            TRACE(Logger(_classname_) << "OneStep[" << _motorState.motorNum << F("] dir=") << dir << endl);

            // Increment/decrement step number, but constrain to 0-3. 
            // If dir=+1 then it steps 0-1-2-3 order, if dir=-1 then it steps 3-2-1-0 order
            _currentStep = ((_currentStep + dir) + 4) % 4;    

            switch (_currentStep) 
            {
                case 0:
                    latchState |= 0x3; // energize coil 1+2
                    break;
                    
                case 1:
                    latchState |= 0x6; // energize coil 2+3
                    break;

                case 2:
                    latchState |= 0xC; // energize coil 3+4
                    break;
            
                case 3:
                    latchState |= 0x9; // energize coil 1+4
                    break;
            }

            break;
            
        case INTERLEAVE:
            TRACE(Logger(_classname_) << "OneStep[" << _motorState.motorNum << F("] dir=") << dir << endl);

            // Increment/decrement step number, but constrain to 0-7. 
            // If dir=+1 then it steps 0-1-2-3-4-5-6-7 order, if dir=-1 then it steps 7-6-5-4-3-2-1-0 order
            _currentStep = ((_currentStep + dir) + 8) % 8;

            switch (_currentStep) 
            {
                case 0:
                    latchState |= 0x1; // energize coil 1 only
                    break;
                    
                case 1:
                    latchState |= 0x3; // energize coil 1+2
                    break;
                    
                case 2:
                    latchState |= 0x2; // energize coil 2 only
                    break;
                    
                case 3:
                    latchState |= 0x6; // energize coil 2+3
                    break;
                    
                case 4:
                    latchState |= 0x4; // energize coil 3 only
                    break; 
                    
                case 5:
                    latchState |= 0xC; // energize coil 3+4
                    break;
                    
                case 6:
                    latchState |= 0x8; // energize coil 4 only
                    break;
            
                case 7:
                    latchState |= 0x9; // energize coil 1+4
                    break;
            }
            
            break;
    }
    
    digitalWrite(_motorState.pinA2, (latchState & 0x1)  ? HIGH : LOW);
    digitalWrite(_motorState.pinB1, (latchState & 0x2)  ? HIGH : LOW);
    digitalWrite(_motorState.pinA1, (latchState & 0x4)  ? HIGH : LOW);
    digitalWrite(_motorState.pinB2, (latchState & 0x8)  ? HIGH : LOW);
}


void StepperMotor::Speed(int16_t rpm) 
{
	_speed = rpm;

	auto stepSpeed = abs(_speed) * _stepsPerRev;

    _usPerStep = (stepSpeed > 0) ? (60000000UL / (uint32_t)stepSpeed) : 0;
}


int16_t StepperMotor::Speed() 
{ 
    return _speed;
}

