#ifndef _DCMotorController_h_
#define _DCMotorController_h_

#include <inttypes.h>
#include <RTL_Stdlib.h>
#include <RTL_Math.h>
#include <RTL_EventFramework.h>
#include <RotationSensor.h>

#include "IDCMotor.h"
#include "IMotorController2.h"


//******************************************************************************
/// \class DCMotorControllerClosedLoop DCMotorControllerClosedLoop.h <DCMotorControllerClosedLoop.h>
/// \brief A high-level, hardware-independent controller for a DC motor that includes
///        motor acceleration and closed-loop feedback using rotation sensors.
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
class DCMotorControllerClosedLoop : public IMotorController2, public EventSource
{
	DECLARE_CLASSNAME;

    public: static const EVENT_ID DEBUG_EVENT = (EventSourceID::DCMotorController | EventCode::DebugInfo);
    
    //**************************************************************************
    /// Constructor.
    //**************************************************************************
    public: DCMotorControllerClosedLoop(IDCMotor& motor, RotationSensor& rotSensor);

    //==========================================================================
    // IPollable Methods
    //==========================================================================
    public: void Poll();

    //==========================================================================
    // IMotorController Methods
    //==========================================================================

    //**************************************************************************
    /// Performs one-time initialization of the motor controller.
    //**************************************************************************
    public: void Initialize();
    
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
    public: void StopNow();

    //==========================================================================
    // Properties
    //==========================================================================

    //**************************************************************************
    /// Checks to see if the motor is currently running
    /// \return true if the current speed is not zero
    //**************************************************************************
    public: bool IsRunning();

    //**************************************************************************
    /// Checks to see if the motor is running at the most recently set speed.
    ///
    /// \return true if the current speed is equal to the desired speed.
    ///
    /// Since the speed of a DC motor continuously varies, the motor is
    /// considered 'at speed' if it is within 5% of the target speed.  
    //**************************************************************************
    public: bool IsAtSpeed();

    //**************************************************************************
    /// Gets the current direction the motor is turning.
    /// \return +1 = FORWARD, -1 = BACKWARD, 0 = STOPPED
    //**************************************************************************
    public: int8_t Direction() { return SIGN(_throttle); };

    //**************************************************************************
    /// Gets or sets the number of sensors pulses per motor revolution.
    /// If the motor does not have a rotation sensor then 1 is returned.
    //**************************************************************************
    public: uint16_t StepsPerRev() { return _pRotSensor->Resolution(); };

    //**************************************************************************
    /// Gets the current, actual motor speed, in RPM. Positive speeds are FORWARD, 
    /// negative speeds are BACKWARD.
    ///
    /// The actual motor speed is measured by the rotation sensor. Since a motor's 
    /// speed naturally varies somewhat over time (due to variations in motor loads, 
    /// accelerations, current draw, mechanical instability, etc), the value returned 
    /// by this method is somewhat variable, although it should always be close 
    /// to the set speed (assuming the motor is not saturated). 
    ///
    /// \return The motor speed.
    //**************************************************************************
    public: int16_t ActualSpeed() { return _currentSpeed; };
    
    //**************************************************************************
    /// Sets the motor speed. Positive speeds are FORWARD, negative speeds are 
    /// BACKWARD. 
    ///
    /// \param[in] value The desired motor speed.
    ///
    /// When the speed is set via this method, acceleration is applied to the
    /// speed will gradually change to the new speed based on the acceleration
    /// value.
    //**************************************************************************
    public: void SetSpeed(int16_t value) { _targetSpeed = value; };
    
    //**************************************************************************
    /// Gets the motor set speed. Positive speeds are FORWARD, negative
    /// speeds are BACKWARD. The "set speed" is the speed value that was set by 
    /// the SetSpeed() method. Unlike the value returned by the ActualSpeed() method, 
    /// the set speed is not affected by the variations in the actual motor speed, 
    /// so it remains constant until explicitly changed by a method call.
    ///
    /// \return The motor set speed.
    //**************************************************************************
    public: int16_t GetSpeed() { return _targetSpeed; };
	
	public: uint32_t GetCount() { return _pRotSensor->ReadCount(); };
    
    public: void SetSpeedNow(int16_t value) { _targetSpeed = value; /*_applyAcceleration = false;*/ };

    public: void SetBrake(uint16_t holdTime=800);

    public: void ReleaseBrake();

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
    public: float Acceleration() { return _acceleration; };
    public: void  Acceleration(float value) { _acceleration = fabs(value); };

    //==========================================================================
    // Internal implementation
    //==========================================================================

    //*****************************************************************************
    /// Computes the new throttle setting.
    //******************************************************************************
    private: int16_t ComputeThrottleSetting(int16_t inputThrottle, float inputRPM, float setpointRPM, float dt);

    //*****************************************************************************
    /// Sets the motor throttle position. The throttle position must be in the
    /// range -255 (max backward) to +255 (max forward). 
    /// 
    /// If the throttle position value is outside the +/-255 range, this method will 
    /// constrain it to this range.
    //******************************************************************************
    private: void SetThrottle(const int16_t throttle);

    //**************************************************************************
    // Internal state
    //**************************************************************************
    /// The motor
    private: IDCMotor* _pMotor;

    /// The rotation sensor
    private: RotationSensor* _pRotSensor;

    /// The current speed value. Positive=FORWARD, negative=BACKWARD.
    private: int16_t _currentSpeed;

    /// The target speed. Positive=FORWARD, negative=BACKWARD.
    private: int16_t _targetSpeed;
    
    /// The throttle setting (-255 to +255)
    private: int16_t _throttle;

    /// The acceleration rate in RPM/second. Must be >= 0
    private: uint16_t _acceleration;

    /// Counts the number of polling cycles to hold a brake.
    private: uint8_t _brakeCount;

    // Start of timing interval
    private: uint32_t _intervalStartTime;

    /// Used for calculating RPM
    private: int32_t  _lastSampleCount;
    private: uint32_t _lastSampleTime;

    // PID Controller variables
    private: float   _errSum;
    private: float   _lastRPM;
    private: int16_t _lastdc;
    
    // Debug data
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
