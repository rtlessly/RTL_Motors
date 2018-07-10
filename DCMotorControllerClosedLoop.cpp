/*******************************************************************************
 DCMotorControllerClosedLoop.cpp
 A high-level, hardware-independent controller for a DC motor that includes
 motor acceleration and closed-loop feedback using rotation sensors.
*******************************************************************************/
#define DEBUG 0
#define DEBUG_FEEDBACK 0

#include <Arduino.h>
#include <RTL_Debug.h>
#include "DCMotorControllerClosedLoop.h"


static const long pollingInterval = 200;


DEFINE_CLASSNAME(DCMotorControllerClosedLoop);


DCMotorControllerClosedLoop::DCMotorControllerClosedLoop(IDCMotor& motor, RotationSensor& rotSensor)
{
    _id = "DCMotorControllerClosedLoop";
    _pMotor = &motor;
    _pRotSensor = &rotSensor;

    _currentSpeed = 0;
    _targetSpeed = 0;
    _throttle = 0;
    _brakeCount = 0;
    _acceleration = 1000;
    //_applyAcceleration = true;
    _intervalStartTime = 0;

    //_throttleController.SetCoefficients(0.65, 0.10, 0.10);
    //_throttleController.SetRange(-255, 255);

    // Rotation sensor variables
    _lastSampleCount = 0;
    _lastSampleTime = 0;
        
    // PID controller variables
     _errSum = 0.0;
     _lastdc = 0;
    _lastRPM = 0.0;
}


//**************************************************************************
/// Performs one-time initialization of the motor controller.
//**************************************************************************
void DCMotorControllerClosedLoop::Initialize()
{
    TRACE(Logger(_classname_, __func__, this) << '[' << _pMotor->ID() << ']' << endl);

    // Ensure rotation sensor is enabled
    _pRotSensor->Enable();
}


void DCMotorControllerClosedLoop::Poll()
{
    TRACE(Logger(_classname_, __func__, this) << '[' << _pMotor->ID() << ']' << endl);

    uint32_t now = millis();

    if ((now - _intervalStartTime) >= pollingInterval)
    {
        Run();
        _intervalStartTime = now;
    }
}


//******************************************************************************
/// Runs the motor at the desired speed and direction. This method makes use of
/// the rotation sensor to continually adjust the motor to keep it at the correct
/// RPM.
///
/// This method should be called frequently to ensure the motor runs at the right
/// speed.
///
/// Returns true if the motor is still running.
//******************************************************************************
bool DCMotorControllerClosedLoop::Run()
{
    TRACE(Logger(_classname_, __func__, this) << '[' << _pMotor->ID() << ']' << endl);

    if (_brakeCount > 0)
    {
        if (--_brakeCount > 0) return (_throttle != 0);

        ReleaseBrake();
    }

    // Get sensor data and calculate raw data values
    auto    data = _pRotSensor->Read();
    int32_t count = data.Count - _lastSampleCount;                  // Sensor counts since last reading
    auto    endTime = (count > 0) ? data.LastCountTime : micros();  // End time for current reading
    int32_t dtMicros = endTime - _lastSampleTime;                   // Time interval for current reading

    // Compute current RPM value
    _currentSpeed = ((count * 60000000L) / (dtMicros * (int32_t)data.CountsPerRev)) * Direction();

    // Calculate new throttle setting
    auto outThrottle = ComputeThrottleSetting(_throttle, _currentSpeed, _targetSpeed, dtMicros/1000000.0);

    SetThrottle(outThrottle);

    // Update interval variables
    _lastSampleTime  = endTime;
    _lastSampleCount = data.Count;

    //_applyAcceleration = IsAtSpeed();
    TRACE(Logger(_classname_, __func__, this) << '[' << _pMotor->ID() << F("] _targetSpeed=") << _targetSpeed 
	                                          << F(", _throttle=") << _throttle << F(", outThrottle=") << outThrottle << endl);

#if DEBUG_FEEDBACK
    // DebugControlData debugData;
    // debugData.MotorID     = _pMotor->ID();
    // debugData.TargetSpeed = _targetSpeed;
    // debugData.Rt          = Rt;
    // debugData.Ra          = Ra;
    // debugData.Error       = e;
    // debugData.Pterm       = pterm;
    // debugData.Iterm       = iterm;
    // debugData.Dterm       = dterm;
    // debugData.T0          = t0;
    // debugData.T1          = outThrottle;
    // debugData.Throttle    = _throttle;
    // QueueEvent(PROXIMITY_EVENT, (void*)&debugData);
#endif

    return (_throttle != 0);
}


void DCMotorControllerClosedLoop::Stop()
{
    TRACE(Logger(_classname_, __func__, this) << '[' << _pMotor->ID() << ']' << endl);
    SetSpeed(0);
}


void DCMotorControllerClosedLoop::StopNow()
{
    TRACE(Logger(_classname_, __func__, this) << '[' << _pMotor->ID() << ']' << endl);
    _targetSpeed = 0;
    _currentSpeed = 0;
    SetThrottle(0);
    SetBrake();
}


void DCMotorControllerClosedLoop::SetBrake(uint16_t holdTime)
{
    _pMotor->Run(DCMotorMode::BRAKE);
    _brakeCount = holdTime / pollingInterval;
}


void DCMotorControllerClosedLoop::ReleaseBrake()
{
    _pMotor->Run(DCMotorMode::RELEASE);
    _brakeCount = 0;
}


bool DCMotorControllerClosedLoop::IsAtSpeed()
{
    // Must be within 3% of target speed
    bool isAtSpeed = abs(_targetSpeed - _currentSpeed) < abs(0.05 * _targetSpeed);

    return isAtSpeed;
}


//**************************************************************************
/// Checks to see if the motor is currently running
/// \return true if the current speed is not zero
//**************************************************************************
bool DCMotorControllerClosedLoop::IsRunning() 
{
    //Logger(_classname_, __func__, this) << '[' << _pMotor->ID() << F("] _currentSpeed=") << _currentSpeed << endl;
	
	return (_currentSpeed != 0.0 && _targetSpeed != 0.0); 
	//return (_targetSpeed != 0.0); 
}


//*****************************************************************************
/// Sets the motor throttle position. The throttle position must be in the
/// range -255 (max backward) to +255 (max forward).
///
/// If the throttle position value is outside the +/-255 range, this method will
/// constrain it to this range.
//******************************************************************************
void DCMotorControllerClosedLoop::SetThrottle(const int16_t value)
{
    TRACE(Logger(_classname_, __func__, this) << '[' << _pMotor->ID() << F("] value=") << value << endl);

    // Constrain to allowed throttle values
    auto t1 = constrain(value, -255, 255);

    // Reset the brake count to discontinue any braking in progress
    if (t1 != 0) _brakeCount = 0;

    // If already at this throttle setting then nothing to do, so exit
    if (_throttle == t1) return;

    _throttle = t1;

    auto direction = SIGN(_throttle);

    TRACE(Logger(_classname_, __func__, this) << '[' << _pMotor->ID() << F("] _throttle=") << _throttle 
	                                          << F("] direction=") << (int)direction  << endl);

    _pMotor->Speed(abs(_throttle));
    _pMotor->Run((direction > 0) ? DCMotorMode::FORWARD  :
                 (direction < 0) ? DCMotorMode::BACKWARD :
                                   DCMotorMode::RELEASE);
}


//*****************************************************************************
/// Computes the new throttle setting using a PID algorithm.
/// PID=Proportional-Integral-Derivative
//******************************************************************************
int16_t DCMotorControllerClosedLoop::ComputeThrottleSetting(int16_t inThrottle, float actualRPM, float targetRPM, float dt)
{
    const float Kp = 0.5;
    const float Ki = 0.1;
    const float Kd = 0;
    const int16_t minThrottle = -255;
    const int16_t maxThrottle = 255;
    
    int16_t accel  = _acceleration;
    auto e  = targetRPM - actualRPM;            // Error signal
    auto di = (actualRPM - _lastRPM) / dt;      // Change in input RPM since last interval

    // Adjust error to constrain total RPM change to the acceleration rate for the interval
    if (_acceleration > 0)
    {
        auto dvMax = _acceleration * dt;        // Max delta RPM for the current time interval

        e = constrain(e, -dvMax, dvMax);
    }

    // As long the error remains in the same direction we allow the error sum to accumulate
    // for the I-term. But if the direction changes then we reset the error sum to 0, as we are 
    // have either reached the target exactly or overshot (either too high or too low). This 
    // prevents the I-term from overdriving the output.
    auto dc = (int16_t)SIGN(e);                 // Compute the direction for the error signal

    if (dc != _lastdc) _errSum = 0.0;           // Reset error sum if error changed directions

    // Compute PID terms
    auto pterm = (Kp * e);                      // Proportional term (P) - Feedback proportional to the error signal
    auto iterm = (Ki * e * dt) + _errSum;       // Integral term (I) - Feedback proportional to the accumulated error
    auto dterm = -Kd * di;                      // Derivative term (D) - Feedback proportional to the change in error

    // Calculate the new throttle setting
    int16_t outThrottle = inThrottle + pterm + iterm + dterm;

    // Constrain the output throttle setting to the valid range
    outThrottle = constrain(outThrottle, minThrottle, maxThrottle);

    // Update interval variables
    // If the output throttle value is saturated then reset error sum to 0. Once the throttle
    // is saturated the actual RPM value can never reach the target RPM value, so it will do
    // no good to continue accumulating the error as we cannot drive the throttle any harder. 
    // Left unchecked, the continued error summing would just create a huge I-term value that
    // would cause future outputs to lag when changing speed until the accumulated error has 
    // been canceled out or "unwound" (this is called "integral windup").
    _lastRPM = actualRPM;
    _lastdc  = dc;
    _errSum  = (outThrottle == minThrottle || outThrottle == maxThrottle) ? 0.0 : iterm;

    TRACE(Logger(_classname_, __func__, this) << '[' << _pMotor->ID() 
                                              << F("] inThrottle=") << inThrottle
                                              << F(", actualRPM=") << actualRPM  
                                              << F(", targetRPM=") << targetRPM  
                                              << F(", _acceleration=") << _acceleration  
                                              << F(", dt=") << dt  
                                              << F(", e=") << e  
                                              << F(", di=") << di  
                                              << F(", pterm=") << pterm  
                                              << F(", iterm=") << iterm  
                                              << F(", dterm=") << dterm
                                              << F(", outThrottle=") << outThrottle
                                              << endl);

    return outThrottle;
}
