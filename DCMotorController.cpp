/*******************************************************************************
 DCMotorController.cpp
 High-level, hardware-independent DC motor contorller
*******************************************************************************/

#define DEBUG 0
#define DEBUG_FEEDBACK 0

#include <Arduino.h>
#include <RTL_Debug.h>
#include <EventDispatcher.h>
#include "DCMotorController.h"


static DebugHelper Debug("DCMotorController");


EVENT_ID DCMotorController::DebugEvent = GenerateEventID();


DCMotorController::DCMotorController(IDCMotor* pMotor, RotationSensor* pRotSensor)
{
    _pMotor = pMotor;
    _pRotSensor = pRotSensor;

    _currentSpeed = 0;
    _targetSpeed = 0;
    _currentPos = 0;
    _targetPos = 0;
    _throttle = 0;
    _direction = STOPPED;
    _acceleration = 1000;
    _isRunningToPosition = false;
    _intervalStartTime = 0;

    // variables for tracking the roation sensor
    _lastSampleCount = 0;
    _lastSampleTime = 0;
    _lastPositionCount = 0;
    _prevError = 0;
    
    // Allows the Poll() method to be automatically called
    EventDispatcher::Add(*this);
}


static const long pollingInterval = 200;


void DCMotorController::Poll()
{
    UpdatePosition();    

    if (_isRunningToPosition)
    {
        // Stop when we are at or past the target position
        if (_currentPos >= _targetPos) StopFast();
    }
    
    uint32_t now = millis();

    if (_intervalStartTime == 0)
    {
        Debug.Log("[%i].%s Initializing rotation sensor", _pMotor->ID(), __func__);

        if (_pRotSensor != NULL) _pRotSensor->Enable();

        _intervalStartTime = now;
    }
    else if ((now - _intervalStartTime) >= pollingInterval) 
    {
        Debug.Log("[%i].%s _intervalStartTime=%l, now=%l", _pMotor->ID(), __func__, _intervalStartTime, now);
        Run();
        _intervalStartTime = now;
    }
}


//******************************************************************************
//******************************************************************************
// Logic to run the motor
//******************************************************************************
//******************************************************************************

void DCMotorController::RunToPosition(long targetPosition)
{
    Debug.Log("[%i].%s(%l)", _pMotor->ID(), __func__, targetPosition);
    TargetPosition(targetPosition);
    _isRunningToPosition = true;
}


//******************************************************************************
/// Runs the motor at the desired speed and direction. This method makes use of
/// the rotation sensor to continually adjust the motor to keep it at the correct
/// RPM.
///
/// This method should be called frequently to ensure the motor runs at the right 
/// speed.
///
/// NOTE: If there is no rotation sensor then this just makes sure the motor speed 
///       is set. In this case, the target speed setting is interpreted as a raw 
///       motor speed value rather than as RPM.
/// 
/// Returns true if the motor is still running.
//******************************************************************************
bool DCMotorController::Run()
{
    Debug.Log("[%i].%s", _pMotor->ID(), __func__);
    
    if (_pRotSensor != NULL) 
        UpdateThrottleWithFeedback();
    else
        UpdateThrottleNoFeedback();

    return (_throttle != 0);
}


//******************************************************************************
/// Updates the throttle setting based on feedback from the current RPM of the 
/// motor and target RPM. Takes acceleration into account as well as changes of 
/// direction. Used when the motor has a rotation sensor.
//******************************************************************************
void DCMotorController::UpdateThrottleWithFeedback()
{
    long now = micros();

    // Get sensor data and calculate raw data values
    RotationSensor::CountData data = _pRotSensor->Read();

    uint32_t count    = data.Count - _lastSampleCount;            // Sensor counts since last reading
    uint32_t endTime  = (count > 0) ? data.LastCountTime : now;   // End time for current reading
    uint32_t interval = endTime - _lastSampleTime;                // Time interval for current reading

    //_currentPos  += count;
    _currentSpeed = ((count * 60000000L) / (interval * data.CountsPerRev)) * _direction;

    // Calculate input variables
    int Ra = _currentSpeed;                        // Actual RPM
    int Rt = _targetSpeed;                         // Target RPM
    int dc = SIGN(Rt - Ra);                        // Direction of change (accelerating or decelerating)
    int t0 = _throttle;                            // Input throttle setting
    int a  = interval * _acceleration / 1000000L;  // Acceleration factor for the current time interval
    
    // Apply acceleration - Adjust the target RPM, if needed, to keep the total change 
    // less than or equal to the acceleration rate for the interval
    if (abs(Rt - Ra) > a) Rt = Ra + (a*dc);

    // If reversing directions then we want to force a pass through 0 first
    if ((SIGN(Ra) != SIGN(Rt)) && (Ra != 0)) Rt = 0;
    //if ((Ra ^ Rt) & 0x8000) && (Ra != 0)) Rt = 0;

    // Calculate PID control settings
    const float Kp = 0.65;                         // Proportional term coefficient
    const float Ki = 0.10;                         // Integral term coefficient
    const float Kd = 0.10;                         // Derivative term coefficient
                                        
    int e  = Rt - Ra;                              // Error signal
    float pterm = Kp * e;                          // Proportional term (P) - Control function that computes feedback proportional to the error signal
    float iterm = Ki * (e + _prevError);           // Integral term (I)     - Forcing function that enhances weak feedback (minimizes lagging)
    float dterm = Kd * (e - _prevError);           // Derivative term (D)   - Damping function that suppresses strong feedback (minimizes ringing)

    // Calculate new throttle setting
    int t1 = t0 + (pterm + iterm + dterm);

    if (Ra == 0 && Rt == 0) t1 = 0;                // Force throttle setting to 0, if needed

    // Update interval variables
    _prevError = e;
    _lastSampleTime  = endTime;
    _lastSampleCount = data.Count;

    // Set new throttle setting
    SetThrottle(t1);

    Debug.Log("[%i].%s,%i,%i,%i,%i,%f,%f,%f,%i,%i,%i", _pMotor->ID(), __func__, _targetSpeed, Rt, Ra, e, pterm, iterm, dterm, t0, t1, _throttle);

#if DEBUG_FEEDBACK 
    DebugControlData debugData;
    debugData.MotorID     = _pMotor->ID();
    debugData.TargetSpeed = _targetSpeed;
    debugData.Rt          = Rt;
    debugData.Ra          = Ra;
    debugData.Error       = e;
    debugData.Pterm       = pterm;
    debugData.Iterm       = iterm;
    debugData.Dterm       = dterm;
    debugData.T0          = t0;
    debugData.T1          = t1;
    debugData.Throttle    = _throttle;    
    DispatchEvent(DebugEvent, (void*)&debugData);    
#endif
}


//******************************************************************************
/// Updates the throttle setting directly from the target speed value. This 
/// interprets the target speed value as a raw motor speed rather than as RPM.
/// Takes acceleration into account as well as changes of direction. 
/// Used when the motor does not have a rotation sensor.
//******************************************************************************
void DCMotorController::UpdateThrottleNoFeedback()
{
    if (_throttle == _targetPos) return _throttle;
    
    uint32_t now = millis();
    
    int count = now - _lastSampleTime;            // Simulate count value as milliseconds since last sample
    int t0 = _throttle;                           // Actual throttle setting
    int t1 = _targetSpeed;                        // Target throttle setting
    int e  = t1 - t0;                             // Throttle error 
    int dc = SIGN(t1 - t0);                       // Direction of change (accelerating or decelerating)
    int a  = count * _acceleration / 1000;        // Acceleration factor for the current time interval
    
    // Apply acceleration - Adjust the target speed, if needed, to keep the total change 
    // less than or equal to the acceleration rate for the interval
    if (abs(e) > a) t1 = t0 + (a*dc);
    
    // If reversing directions then we want to force a pass through 0 first
    if ((SIGN(t1) != SIGN(t0)) && (t0 != 0)) t1 = 0;

    _currentPos     += count;
    _lastSampleCount = now;
    _lastSampleTime  = now;
    
    // Set new throttle setting
    SetThrottle(t1);
}


//*****************************************************************************
/// Sets the motor throttle position. The throttle position must be in the
/// range -255 (max backward) to +255 (max forward). 
/// 
/// If the throttle position value is outside the +/-255 range, this method will 
/// constrain it to this range.
//******************************************************************************
void DCMotorController::SetThrottle(const int16_t value)
{
  Debug.Log("[%i].%s %i", _pMotor->ID(), __func__, value);
  
  int t1 = constrain(value, -255, 255);   // Constrain to allowed throttle values 
  
  // If already at this throttle setting then nothing to do, so exit
  if (_throttle == t1) return;
  
  _throttle = t1;
  _direction = SIGN(_throttle);
  
  // If we don't have a rotation sensor then set the current speed to the throttle setting
  if (_pRotSensor == NULL) _currentSpeed = t1;

  Debug.Log("[%i].%s throttle=%i, dir=%i", _pMotor->ID(), __func__, _throttle, _direction);
  
  _pMotor->Speed(abs(_throttle));
  _pMotor->Run((_direction > 0) ? DCMotorMode::FORWARD  : 
               (_direction < 0) ? DCMotorMode::BACKWARD :
                                  DCMotorMode::RELEASE);
}


/******************************************************************************
 Updates the motor position. The motor position is culmulative number of steps
 the motor has turned (regardless of direction) since the last reset.
 This is always a positive number.
******************************************************************************/
void DCMotorController::UpdatePosition()
{
    if (_pRotSensor != NULL)
    {
        // We have a rotation sensor, so interpret position as sensor counts
        uint32_t count = _pRotSensor->ReadCount();
        
        _currentPos += (count - _lastPositionCount);
        _lastPositionCount = count;
    }
    else
    {
        // No rotation sensor, so interpret position as time
        uint32_t ttg = _targetPos - _currentPos;      // Time to go
        uint32_t v   = _currentSpeed;                 // Speed as raw throttle setting
        uint32_t a   = _acceleration;                 // Acceleration in throttle steps / sec
        uint32_t tts = abs(v) / a;                    // Time it takes to stop
        
        if (tts >= ttg) Stop();                       // Stop when time to stop >= time to go
    }
}


void DCMotorController::Stop()
{
    Debug.Log("[%i].%s", _pMotor->ID(), __func__);
    TargetSpeed(0);
}


void DCMotorController::StopFast()
{
    //Debug.Log("[%i].%s", _pMotor->ID(), __func__);
    TargetSpeed(0);
    SetThrottle(0);
    _isRunningToPosition = false;
}


//******************************************************************************
// Properties

bool DCMotorController::IsAtTargetSpeed()
{
    // Must be within 3% of target speed
    bool isAtTargetSpeed = abs(_targetSpeed - _currentSpeed) < abs(0.03 * _targetSpeed);
    
    return isAtTargetSpeed;
}

