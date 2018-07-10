#ifndef _DCMotorController_h_
#define _DCMotorController_h_

#include <inttypes.h>
#include <RTL_Stdlib.h>
#include <RTL_Math.h>
#include <RTL_EventFramework.h>
#include <RotationSensor.h>
#include "IDCMotor.h"
#include "IMotorController.h"


//******************************************************************************
/// \class DCMotorController DCMotorController.h <DCMotorController.h>
/// \brief Support for dc motors with acceleration and rotation sensors.
///
/// Defines a high-level, hardware-independent controller for a DC motor.
///
/// \par Operation
///
/// \par Positioning
///
/// \par Caveats
///
/// \par Performance
///
//******************************************************************************
class DCMotorController : public IMotorController, public EventSource
{
    public: static const EVENT_ID DEBUG_EVENT = (EventSourceID::DCMotorController | EventCode::DebugInfo);
    
    //**************************************************************************
    /// Constructor.
    //**************************************************************************
    public: DCMotorController(IDCMotor* pMotor, RotationSensor* pRotSensor=NULL);

    //==========================================================================
    // IPollable Methods
    //==========================================================================
    public: void Poll();

    //==========================================================================
    // IMotorController Methods
    //==========================================================================

    //**************************************************************************
    /// Runs the motor at the previously specified speed and direction, taking
    /// acceleration into account. Unlike stepper motors, DC  motors do not need 
    /// need to be constantly updated to keep moving. However, the DC motor needs 
    /// to be regularly monitored to compensate for speed variations in the motor 
    /// This also allows for controlled acceleration/deceleration. As such, this
    /// method should be called frequently to maintain good operation.
    ///
    /// \return true While the motor is running; false if it is stopped.
    //**************************************************************************
    public: bool Run();

    //**************************************************************************
    /// Instructs the motor to run until the specified motor position is reached. 
    /// Once the target position is reached the motor will stop.
    ///
    /// Note: For DC motor controllers with a rotational sensor, the 'position'
    ///       is interpreted as sensor counts. For all other DC motors, the 
    ///       recommended implementation is to interpret the target position
    ///       as the number of milliseconds to run.
    //**************************************************************************
    public: void RunToPosition(long targetPosition);

    //**************************************************************************
    /// Stops the motor by setting the target speed to zero and allowing
    /// acceleration to bring it to a stop. The motor is still considered
    /// running when this method returns, so the run() method still needs
    /// to be called to actually run the motor through the steps to decelerate
    /// and stop.
    //**************************************************************************
    public: void Stop();

    //**************************************************************************
    /// Stops the motor immediately by setting both current and target speed to 0.
    /// The motor is not considered running when this method returns.
    //**************************************************************************
    public: void StopFast();

    //==========================================================================
    // Properties
    //==========================================================================

    //**************************************************************************
    /// Checks to see if the motor is currently running
    /// \return true if the current speed is not zero
    //**************************************************************************
    public: inline bool IsRunning() { return (_currentSpeed != 0.0 || _isRunningToPosition); };

    //**************************************************************************
    /// Checks to see if the motor is currently running to a position
    /// \return true if the motor is running to a position
    //**************************************************************************
    public: inline bool IsRunningToPosition() { return _isRunningToPosition; };

    //**************************************************************************
    /// Checks to see if the motor is running at the target speed
    /// \return true if the current speed is equal to the target speed
    /// Since the speed of a DC motor continuously varies, the motor is
    /// considered 'at speed' if it is within 10% of the target speed.  
    //**************************************************************************
    public: bool IsAtTargetSpeed();

    //**************************************************************************
    /// Checks to see if the motor is at the target position
    /// \return true if the motor is at the target position
    //**************************************************************************
    public: inline bool IsAtTargetPosition() { return (_currentPos >= _targetPos); };

    //**************************************************************************
    /// Gets the current direction the motor is turning.
    /// \return +1 = FORWARD, -1 = BACKWARD, 0 = STOPPED
    //**************************************************************************
    public: inline int8_t Direction() { return SIGN(_throttle); };

    //**************************************************************************
    /// Gets or sets the number of sensors pulses per motor revolution.
    /// If the motor does not have a rotation sensor then 1 is returned.
    //**************************************************************************
    public: inline uint16_t StepsPerRev() { return _pRotSensor == NULL ? 1 : _pRotSensor->Resolution(); };

    //**************************************************************************
    /// Gets or sets the current motor speed. Positive speeds are FORWARD, negative
    /// speeds are BACKWARD. If the motor has a rotation sensor then the speed is
    /// in RPM. Otherwise, it is a raw motor speed value in the +/-255 range.
    ///
    /// \param[in] value The desired motor speed.
    /// \return The motor speed.
    //**************************************************************************
    public: inline float Speed() { return _currentSpeed; };
    public: inline void  Speed(float value) { _targetSpeed = value; };

    //**************************************************************************
    /// Gets or sets the target motor speed in RPM. The Run() function
    /// will accelerate up to the speed set by this function.
    ///
    /// \param[in] value The desired target speed in RPM.
    /// \return The currently set target motor speed in RPM.
    ///
    /// If the motor has a rotation sensor then the speed is in RPM. Otherwise, 
    /// it is the raw motor speed value.
    //**************************************************************************
    public: inline float TargetSpeed() { return _targetSpeed; };
    public: inline void  TargetSpeed(float value) { _targetSpeed = (_pRotSensor != NULL) ? value : constrain(value, -255, 255); };

    //**************************************************************************
    /// Gets or sets the currently motor position. Position values are always considered 
    /// positive.
    ///
    /// \param[in] value The new position.
    /// \return The current motor position.
    ///
    /// When setting the current position of the motor, wherever the motor happens
    /// to be at that moment is considered to be the new specified position.
    ///
    /// Note: For DC motor controllers with a rotational sensor, the 'position'
    ///       is interpreted as sensor counts. For all other DC motors, the 
    ///       recommended implementation is to interpret the position as the 
    ///       number of milliseconds to run.
    //**************************************************************************
    public: inline long Position() { return _currentPos; };
    public: inline void Position(long value) { _targetPos = _currentPos = abs(value); };

    //**************************************************************************
    /// Gets or sets the set target position. Position values are always considered 
    /// positive.
    ///
    /// \param[in] value The desired target position.
    /// \return The target position.
    ///
    /// Note: For DC motor controllers with a rotational sensor, the 'position'
    ///       is interpreted as sensor counts. For all other DC motors, the 
    ///       recommended implementation is to interpret the position as the 
    ///       number of milliseconds to run.
    //**************************************************************************
    public: inline long TargetPosition() { return _targetPos; };
    public: inline void TargetPosition(long value) { _targetPos = abs(value); };

    //**************************************************************************
    /// Gets or sets the acceleration rate. Will always be a positive value.
    ///
    /// \param[in] value The desired acceleration in RPM/sec. Must be > 0.0.
    ///                  Negative values are converted to positive.
    /// \return current acceleration rate in RPM/sec.
    ///
    /// Setting the acceleration to 0 causes acceleration to be ignored and
    /// motor speed changes happen as fast as the motor can make them.
    //**************************************************************************
    public: inline float Acceleration() { return _acceleration; };
    public: inline void  Acceleration(float value) { _acceleration = fabs(value); };

    //==========================================================================
    // Internal implementation
    //==========================================================================

    //**************************************************************************
    /// Updates the motor position. The motor position is culmulative number of 
    /// steps the motor has turned (regardless of direction) since the last position 
    /// reset. This is always a positive number.
    //**************************************************************************
    private: void UpdatePosition();

    //*****************************************************************************
    /// Sets the motor throttle position. The throttle position must be in the
    /// range -255 (max backward) to +255 (max forward). 
    /// 
    /// If the throttle position value is outside the +/-255 range, this method will 
    /// constrain it to this range.
    //******************************************************************************
    private: void SetThrottle(const int16_t throttle);

    private: void UpdateThrottleWithFeedback();

    private: void UpdateThrottleNoFeedback();

    private: void SetBrake();

    private: void ReleaseBrake();

    //**************************************************************************
    // Internal state
    //**************************************************************************
    /// The motor
    private: IDCMotor* _pMotor;

    /// The rotation sensor
    private: RotationSensor* _pRotSensor;

    /// The current absolute position.
    private: int32_t _currentPos;

    /// The target position.
    private: int32_t _targetPos;

    /// The last computed speed value. Positive=FORWARD, negative=BACKWARD.
    private: int16_t _currentSpeed;

    /// The target speed. Positive=FORWARD, negative=BACKWARD.
    private: int16_t _targetSpeed;

    /// The acceleration rate in RPM/second. Must be >= 0
    private: uint16_t _acceleration;

    /// The last sample time in microseconds.
    /// Used for calculating motor speed from sensor pulses.
    private: int16_t  _prevError;
    private: uint32_t _lastSampleTime;
    private: uint32_t _lastSampleCount;
    private: uint32_t _lastPositionCount;
    private: uint32_t _intervalStartTime;
    
    /// The current motor throttle setting in raw motor speed units (-255 to +255)
    private: int16_t _throttle;

    /// Indicates if we are running to a position.
    private: bool _isRunningToPosition;

    /// Counts the number of polling cycles to hold a brake.
    private: uint8_t _brakeCount;

    public: struct DebugControlData
    {
        uint8_t MotorID;
        int16_t TargetSpeed;
        int16_t Rt;
        int16_t Ra;
        int16_t Error;
        int16_t T0;
        int16_t T1;
        int16_t Throttle;
        float   Pterm;
        float   Iterm;
        float   Dterm;
    };
};

#endif
