/*******************************************************************************
 StepperMotorController.h
 Defines the abstract interface for a motor controller
*******************************************************************************/

#ifndef StepperMotorController_h
#define StepperMotorController_h

#include <inttypes.h>
#include <IPollable.h>
#include "IStepperMotor.h"
#include "IMotorController.h"


// These defs cause trouble on some versions of Arduino
#undef round


/////////////////////////////////////////////////////////////////////
/// \class StepperMotorController StepperMotorController.h <StepperMotorController.h>
/// \brief Support for 4-wire stepper motors with acceleration, etc.
///
/// This defines a controller for a single 4 wire stepper motor with optional 
/// acceleration, deceleration, absolute positioning commands, etc.
///
/// \par Operation
/// This module operates by computing a step time in microseconds. The step
/// time is recomputed after each step and after speed and acceleration
/// parameters are changed by the caller. The time of each step is recorded in
/// microseconds. The run() function steps the motor once if a new step is due.
/// The run() function must be called frequently until the motor is in the
/// desired position, after which time run() will do nothing.
///
/// \par Positioning
/// Positions are specified by a signed long integer. At construction time,
/// the current position of the motor is consider to be 0. Positive
/// positions are clockwise from the initial position; negative positions are
/// anticlockwise. The current position can be altered for instance after
/// initialization positioning.
///
/// \par Caveats
/// This is an open loop controller: If the motor stalls or is over-sped,
/// StepperMotorController will not have a correct idea of where the motor 
/// really is (since there is no feedback of the motor's real position. We only 
/// know where we _think_ it is, relative to the initial starting point).
///
/// \par Performance
/// The fastest motor speed that can be reliably supported is about 4000 steps
/// per second at a clock frequency of 16 MHz on Arduino such as Uno etc. Faster
/// processors can support faster stepping speeds. However, any speed less than
/// that, down to very slow speeds (much less than one per second), are also
///  supported, provided the run() function is called frequently enough to step
/// the motor whenever required for the speed set. Calling Acceleration() is
/// expensive, since it requires a square root to be calculated.
///
/// Gregor Christandl reports that with an Arduino Due and a simple test program,
/// he measured 43163 steps per second using runStep(),
/// and 16214 steps per second using run();
class StepperMotorController : public IPollable
{
    //**************************************************************************
    /// Motor direction
    //**************************************************************************
    public: enum MotorDirection
    {
        FORWARD  = IStepperMotor::FORWARD,
        STOPPED  = IStepperMotor::STOPPED,
        BACKWARD = IStepperMotor::BACKWARD
    };

    /// Minimum motor step size in microseconds.
    /// The minimum practical step size is approximately 20 microseconds.
    /// Times less than 20 microseconds will usually result in 20 microseconds or so.
    public: const long MIN_STEP_SIZE = 20;


    //**************************************************************************
    /// Constructor.
    //**************************************************************************
    public: StepperMotorController(IStepperMotor* pMotor);

    //==========================================================================
    // IPollable Methods
    //==========================================================================

    public: void Poll();

    //==========================================================================
    // Methods
    //==========================================================================

    //**************************************************************************
    /// Runs the motor by stepping it one step at the appropriate time, implementing
    /// accelerations and decelerations. Each call to this method will make at 
    /// most one step, and then only when a step is due, based on the current speed
    /// and time since the last step. You must call this as frequently as possible, 
    /// but at least once per minimum step time interval, preferably in your main 
    /// loop.
    ///
    /// The motor will continue to run as long as you call this method until the
    /// motor speed is set to zero or one of the stop methods are called.
    ///
    /// \return true While the motor is running; false if it is stopped.
    //**************************************************************************
    public: bool Run();

    //**************************************************************************
    /// Runs the motor by stepping it one step at the appropriate time, implementing
    /// accelerations and decelerations, until the target step position is reached.
    /// Each call to this method will make at most one step, and then only when 
    /// a step is due, based on the current speed and time since the last step. 
    /// You must call this as frequently as possible, but at least once per 
    /// minimum step time interval, preferably in your main loop.
    ///
    /// Once the target position is reached the motor will stop.
    ///
    /// \return true as long as the motor is running; false when the motor has 
    ///         reached the target and stopped.
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
    public: bool IsRunning();

    //**************************************************************************
    /// Checks to see if the motor is currently running to a position
    /// \return true if the motor is running to a position
    //**************************************************************************
    public: bool IsRunningToPosition();

    //**************************************************************************
    /// Checks to see if the motor is running at the target speed
    /// \return true if the current speed is equal to the target speed
    //**************************************************************************
    public: bool IsAtTargetSpeed();

    //**************************************************************************
    /// Checks to see if the motor is at the target position
    /// \return true if the motor is at the target position
    //**************************************************************************
    public: bool IsAtTargetPosition();

    //**************************************************************************
    /// Gets the current direction the motor is turning.
    /// \return +1 = FORWARD, -1 = BACKWARD
    //**************************************************************************
    public: int8_t Direction();

    //**************************************************************************
    /// Gets the current direction the motor is turning.
    /// \return +1 = FORWARD, -1 = BACKWARD
    //**************************************************************************
    public: uint16_t StepsPerRev() { return _motor->StepsPerRev(); };

    //**************************************************************************
    /// Gets or sets the current motor speed. Positive speeds are FORWARD, negative
    /// speeds are BACKWARD. When setting the speed, acceleration is bypassed and 
    /// the motor speed immediately becomes the set value. 
    ///
    /// \param[in] value The desired motor speed in steps/second.
    /// \return The motor speed in steps/second.
    ///
    /// Very slow speeds may be set, e.g., 0.00027777 for once per hour, approximately.
    /// Speed accuracy depends on the Arduino crystal. Jitter depends on how 
    /// frequently you call the Run() function.
    ///
    /// Caution: the maximum speed achievable depends on your processor and clock 
    ///          speed. Speeds that exceed the maximum speed supported by the 
    ///          processor may result in non-linear accelerations and decelerations 
    ///          or motor stalls.
    ///          Speeds of more than 1000 steps per second may be unreliable.
    //**************************************************************************
    public: float Speed();
    public: void Speed(float value);

    //**************************************************************************
    /// Gets or sets the target motor speed in steps/second. The Run() function 
    /// will accelerate up to the speed set by this function.
    ///
    /// \param[in] value The desired target speed in steps/second.
    /// \return The currently set target motor speed in steps/second.
    ///
    /// Caution: the maximum speed achievable depends on your processor and clock 
    ///          speed. Speeds that exceed the maximum speed supported by the 
    ///          processor may result in non-linear accelerations and decelerations 
    ///          or motor stalls.
    ///          Speeds of more than 1000 steps per second may be unreliable.
    //**************************************************************************
    public: float TargetSpeed();
    public: void TargetSpeed(float value);

    //**************************************************************************
    /// Gets or sets the currently motor position, in steps. 
    ///
    /// \param[in] value The new position in steps.
    /// \return The current motor position in steps. 
    ///
    /// Positive values are in the FORWARD direction from the 0 position, negative 
    /// values are in the BACKWARD direction.
    /// When setting the current position of the motor, wherever the motor happens
    /// to be at that moment is considered to be the new specified position. 
    //**************************************************************************
    public: long Position();
    public: void Position(long value);

    //**************************************************************************
    /// Gets or sets the set target position in steps. Subsequent calls to the
    /// RunToPosition() function will step the motor from the current position 
    /// to the target position.
    ///
    /// \param[in] value The desired target step position. 
    /// \return The target position in steps. 
    ///
    /// Positive values are in the FORWARD direction from the 0 position, negative 
    /// values are in the BACKWARD direction.
    ///
    /// Caution: Setting the target position also recalculates the speed for the 
    ///          next step. If you are trying to use constant speed movements, 
    ///          you should call CurrentSpeed() after calling TargetPosition(().
    //**************************************************************************
    public: long TargetPosition();
    public: void TargetPosition(long value);

    //**************************************************************************
    /// Gets or sets the acceleration rate. Will always be a positive value.
    ///
    /// \param[in] value The desired acceleration in steps/sec/sec. Must be > 0.0. 
    ///                  Negative values are converted to positive.
    /// \return current acceleration rate in steps/sec/sec.
    ///
    /// Setting the acceleration is an expensive call since it requires a square 
    /// root to be calculated. Don't call more often than needed.
    //**************************************************************************
    public: float Acceleration();
    public: void Acceleration(float value);
    
    //==========================================================================
    // Internal implementation
    //==========================================================================

    //**************************************************************************
    /// Steps the motor one step at the appropriate time, employing acceleration.
    /// Each call to this method will make at most one step, and then only when 
    /// a step is due, based on the current speed and time since the last step.
    //**************************************************************************
    private: bool OneStep();

    //**************************************************************************
    /// Runs the motor by stepping it one step at the appropriate time, implementing
    /// accelerations and decelerations, until the target step position is reached.
    /// Each call to this method will make at most one step, and then only when 
    /// a step is due, based on the current speed and time since the last step. 
    /// You must call this as frequently as possible, but at least once per 
    /// minimum step time interval, preferably in your main loop.
    ///
    /// Once the target position is reached the motor will stop.
    ///
    /// \return true as long as the motor is running; false when the motor has 
    ///         reached the target and stopped.
    //**************************************************************************
    private: void RunToTargetPosition();

    //**************************************************************************
    /// Computes a new instantaneous speed based on the current acceleration and
    /// sets it as the current speed. It is called by:
    /// \li  after each motor step in the Run() method
    /// \li  after each motor step in the RunToPostiton() method
    /// \li  after any change to target speed through TargetSpeed setter
    //**************************************************************************
    private: void ComputeNewSpeed();

    //**************************************************************************
    /// Computes new acceleration parameters whenever the current or target speed
    /// is changed. It is called by:
    /// \li  Speed setter
    /// \li  TargetSpeed setter
    //**************************************************************************
    private: void UpdateAccelerationParameters();

    //**************************************************************************
    // Controller state settings
    //**************************************************************************
    
    /// The stepper motor
    private: IStepperMotor* _motor;
    
    /// The current motor speed in steps per second. Positive=FORWARD, negative=BACKWARD.
    private: float _currentSpeed;

    /// The target speed in steps per second. Positive=FORWARD, negative=BACKWARD.
    private: float _targetSpeed;

    /// The current absolution position in steps. 
    private: long _currentPos;

    /// The target position in steps.
    private: long _targetPos;

    /// The current interval between steps in microseconds.
    /// 0 means the motor is currently stopped with _currentSpeed == 0
    private: unsigned long _currentStepSize;

    /// Target step size in microseconds
    private: float _targetStepSize;

    /// The last step time in microseconds. Used for timing motor step pulses.
    private: unsigned long _lastStepTime;

    /// The acceleration rate in steps per second per second. Must be >= 0
    private: float _acceleration;

    /// Acceleration step 0 size in microseconds (C0)
    private: float _accelStepSize0;

    /// Last acceleration step size in microseconds (Cn)
    private: float _accelStepSize;

    /// The current step number on the acceleration curve (n)
    private: long _accelStep;
    
    /// Indicates if we are running to a position.
    private: bool _isRunningToPosition;
};

#endif
