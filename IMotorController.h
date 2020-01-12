/*******************************************************************************
 IMotorController.h
 Defines the abstract interface for a motor controller
*******************************************************************************/

#ifndef _IMotorController_h_
#define _IMotorController_h_

#include <inttypes.h>
#include <RTL_Stdlib.h>


// These defs cause trouble on some versions of Arduino
#undef round


class IMotorController
{
    //**************************************************************************
    /// Motor direction
    //**************************************************************************
    public: enum MotorDirection
    {
        FORWARD  = 1,
        STOPPED  = 0,
        BACKWARD = -1
    };


    //**************************************************************************
    /// Constructor.
    //**************************************************************************
    protected: IMotorController() { };
    
    //==========================================================================
    // Methods
    //==========================================================================

    //**************************************************************************
    /// Runs the motor one increment at a time. This method is intended to be 
    /// called from within the sketch loop() function.
    ///
    /// This method normally MUST be implemented by stepper motor controllers to
    /// step the motor at the appropriate time. It is optional for DC motor controllers 
    /// but may still be used if the motor controller implements acceleration or
    /// has a rotational sensor. 
    ///
    /// The motor will continue to run as long as you call this method until the
    /// motor speed is set to zero or one of the stop methods are called.
    ///
    /// \return true While the motor is running; false if it is stopped.
    //**************************************************************************
    public: virtual bool Run() { };

    //**************************************************************************
    /// Runs the motor until the target position is reached. This method must be 
    /// continually called (preferably in the main sketch loop) to check if the 
    /// target position has been reached. Once the target position is reached the
    /// motor will stop.
    ///
    /// \return true as long as the motor is running; false when the motor has 
    ///         reached the target and stopped.
    ///
    /// Note: The behavior of this method can vary depending on the type of 
    ///       motor and motor controller. For stepper motors the target position
    ///       is the number of motor steps relative to the current zero position.
    ///       For DC motor controllers with a rotational sensor, this would be
    ///       the number of pulses from the sensor. For all other DC motors, the
    ///       recommended implementation is to interpret the target position as
    ///       the number of milliseconds to run.
    //**************************************************************************
    public: virtual void RunToPosition(long targetPosition) = 0;

    //**************************************************************************
    /// Stops the motor by setting the target speed to zero and allowing 
    /// acceleration to bring it to a stop. The motor is still considered
    /// running when this method returns, so the Run() method still needs 
    /// to be called to actually run the motor through the steps to decelerate 
    /// and stop.
    //**************************************************************************
    public: virtual void Stop() = 0;

    //**************************************************************************
    /// Stops the motor immediately by setting both current and target speed to 0.
    /// The motor is not considered running when this method returns.
    //**************************************************************************
    public: virtual void StopFast() = 0;

    //==========================================================================
    // Properties
    //==========================================================================

    //**************************************************************************
    /// Checks to see if the motor is currently running
    /// \return true if the current speed is not zero
    //**************************************************************************
    public: virtual bool IsRunning() = 0;

    //**************************************************************************
    /// Checks to see if the motor is currently running to a position
    /// \return true if the motor is running to a position
    //**************************************************************************
    public: virtual bool IsRunningToPosition() = 0;

    //**************************************************************************
    /// Checks to see if the motor is running at the target speed
    /// \return true if the current speed is equal to the target speed
    //**************************************************************************
    public: virtual bool IsAtTargetSpeed() = 0;

    //**************************************************************************
    /// Checks to see if the motor is at the target position
    /// \return true if the motor is at the target position
    //**************************************************************************
    public: virtual bool IsAtTargetPosition() = 0;

    //**************************************************************************
    /// Gets the current direction the motor is turning.
    /// \return +1 = FORWARD, -1 = BACKWARD
    //**************************************************************************
    public: virtual int8_t Direction() = 0;

    //**************************************************************************
    /// Returns the number of steps in one motor revolution. For stepper motors
    /// this is simply the number of steps in one revolution of the motor. For 
    /// DC motors it is a bit more complicated. If the DC motor has a rotation 
    /// sensor then this is the number of pulses per revolution. Otherwise, DC 
    /// motors should just return 1.
    //**************************************************************************
    public: virtual uint16_t StepsPerRev() = 0;

    //**************************************************************************
    /// Gets or sets the current motor speed. Positive speeds are FORWARD, negative
    /// speeds are BACKWARD. When setting the speed, acceleration is bypassed and 
    /// the motor speed immediately becomes the set value. 
    ///
    /// \param[in] value The desired motor speed.
    /// \return The current motor speed.
    ///
    /// Caution: the maximum speed achievable depends on the type of motor (DC motor 
    ///          or stepper motor). In addition, stepper motor speed is highly 
    ///          dependent on your processor and clock speed. Attempting to set
    ///          the speed faster than the motor can handle will have no effect.
    //**************************************************************************
    public: virtual float Speed() = 0;
    public: virtual void Speed(float value) = 0;

    //**************************************************************************
    /// Gets or sets the target motor speed in steps/second. 
    ///
    /// \param[in] value The desired target speed in steps/second.
    /// \return The currently set target motor speed in steps/second.
    ///
    /// Note: This property is provided to support motor acceleration, which is 
    ///       primarily used by stepper motor controllers (but DC motor controllers
    ///       may also implement acceleration). If the motor controller is not
    ///       using acceleration then this property should defer to the Speed 
    ///       property. 
    //**************************************************************************
    public: virtual float TargetSpeed() = 0;
    public: virtual void TargetSpeed(float value) = 0;

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
    ///
    /// Note: This property is provided primarily to support positioning of stepper 
    ///       motors. However, DC motor controllers that have rotational sensors 
    ///       could also support positioning via pulse counting (albeit less accurately). 
    ///       If the motor controller does not support positioning then this property 
    ///       will have no effect. 
    ///
    /// Note: The behavior of this method can vary depending on the type of 
    ///       motor and motor controller. For stepper motors the position is the 
    ///       number of motor steps relative to the current zero position. For 
    ///       DC motor controllers with a rotational sensor, this would be the 
    ///       number of pulses from the sensor. For all other DC motors, the
    ///       recommended implementation is to interpret the position as
    ///       milliseconds.
    //**************************************************************************
    public: virtual long Position() = 0;
    public: virtual void Position(long value) = 0;

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
    /// Note: This property is provided primarily to support positioning of stepper 
    ///       motors. However, DC motor controllers that have rotational sensors 
    ///       could also support positioning via pulse counting (albeit less accurately). 
    ///
    /// Note: The behavior of this method can vary depending on the type of 
    ///       motor and motor controller. For stepper motors the position is the 
    ///       number of motor steps relative to the current zero position. For 
    ///       DC motor controllers with a rotational sensor, this would be the 
    ///       number of pulses from the sensor. For all other DC motors, the
    ///       recommended implementation is to interpret the position as
    ///       milliseconds.
    //**************************************************************************
    public: virtual long TargetPosition() = 0;
    public: virtual void TargetPosition(long value) = 0;

    //**************************************************************************
    /// Gets or sets the acceleration rate. Will always be a positive value.
    ///
    /// \param[in] value The desired acceleration in steps/sec/sec. Must be > 0.0. 
    ///                  Negative values are converted to positive.
    /// \return current acceleration rate in steps/sec/sec.
    ///
    /// Setting the acceleration can be an expensive call. Don't call more often 
    /// than needed.
    //**************************************************************************
    public: virtual float Acceleration() = 0;
    public: virtual void Acceleration(float value) = 0;
};

#endif
