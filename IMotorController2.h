/*******************************************************************************
 IMotorController.h
 Defines the abstract interface for a motor controller
*******************************************************************************/

#ifndef _IMotorController2_h_
#define _IMotorController2_h_

#include <inttypes.h>
#include <RTL_Stdlib.h>
#include <IPollable.h>


// These defs cause trouble on some versions of Arduino
#undef round


class IMotorController2
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
    protected: IMotorController2() { };
    
    //==========================================================================
    // Methods
    //==========================================================================

    //**************************************************************************
    /// Performs any one-time initialization needed for the motor. 
    //**************************************************************************
    public: virtual void Initialize() { };

    //**************************************************************************
    /// Runs the motor one increment at a time. This method is intended to be 
    /// called from within the sketch loop() function.
    ///
    /// This method normally MUST be implemented by stepper motor controllers to
    /// step the motor at the appropriate time. It is optional for DC motor controllers 
    /// but may still be used if the motor controller implements acceleration or
    /// has a closed-loop speed control system. 
    ///
    /// The motor will continue to run as long as you call this method until the
    /// motor speed is set to zero or one of the stop methods are called.
    ///
    /// \return true While the motor is running; false if it is stopped.
    //**************************************************************************
    public: virtual bool Run() { };

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
    public: virtual void StopNow() = 0;

    //==========================================================================
    // Properties
    //==========================================================================

    //**************************************************************************
    /// Checks to see if the motor is currently running
    /// \return true if the current speed is not zero
    //**************************************************************************
    public: virtual bool IsRunning() = 0;

    //**************************************************************************
    /// Checks to see if the motor is running at the most recently set speed.
    /// \return true if the current speed is equal to the target speed
    //**************************************************************************
    public: virtual bool IsAtSpeed() = 0;

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
    /// Returns the current step count. For stepper motors this is simply the 
	/// number of steps the motor has turned. For DC motors with a rotation sensor
	/// it is the number rotation sensor steps. Otherwise, this property is undefined.
    //**************************************************************************
    public: virtual uint32_t GetCount() { return 0; };

    //**************************************************************************
    /// Gets the actual motor speed. Positive speeds are FORWARD, negative
    /// speeds are BACKWARD. 
    /// 
    /// The actual motor speed is the speed of the motor as measured by a speed 
    /// sensor on the motor. Since the motor speed naturally varies somewhat over 
    /// time (due to variations in motor loads, accelerations, current draw, 
    /// mechanical instability, etc), the value returned by this method is somewhat 
    /// variable, although it should always be close to the set speed (assuming 
    /// the motor is not saturated). 
    ///
    /// The meaning of the actual speed value is implementation-dependent but 
    /// nominally should be either steps/sec (primarily for stepper motors) or 
    /// RPM (stepper or DC motors).
    ///
    /// If the motor does not have a speed sensing mechanism then the value 
    /// returned by this method is the same as the GetSpeed() method.
    ///
    /// \return The current motor speed.
    ///
    /// Caution: the maximum speed achievable depends on the type of motor (DC motor 
    ///          or stepper motor). In addition, stepper motor speed is highly 
    ///          dependent on your processor and clock speed. Attempting to set
    ///          the speed faster than the motor can handle will have no effect.
    //**************************************************************************
    public: virtual int16_t ActualSpeed() = 0;
    
    //**************************************************************************
    /// Sets the motor speed. Positive speeds are FORWARD, negative speeds are 
    /// BACKWARD. 
    ///
    /// \param[in] value The desired motor speed.
    ///
    /// When the speed is set via this method, acceleration is applied so that
    /// the speed will gradually change to the new speed based on the acceleration
    /// value.
    ///
    /// Caution: the maximum speed achievable depends on the type of motor (DC motor 
    ///          or stepper motor). In addition, stepper motor speed is highly 
    ///          dependent on your processor and clock speed. Attempting to set
    ///          the speed faster than the motor can handle will have no effect.
    //**************************************************************************
    public: virtual void SetSpeed(int16_t value) = 0;

    //**************************************************************************
    /// Gets the motor set speed. Positive speeds are FORWARD, negative
    /// speeds are BACKWARD. The "set speed" is the speed value that was set by 
    /// the SetSpeed() method. Unlike the value returned by the ActualSpeed() 
    /// method, the set speed is not affected by the variations in the actual 
    /// motor speed. So it remains constant until explicitly changed by a method 
    /// call.
    ///
    /// \return The motor set speed.
    //**************************************************************************
    public: virtual int16_t GetSpeed() = 0;
    
    //**************************************************************************
    /// Sets the motor speed, ignoring acceleration. Positive speeds are FORWARD, 
    /// negative speeds are BACKWARD. 
    ///
    /// \param[in] value The desired motor speed.
    ///
    /// When the speed is set via this method the motor speed is immediately
    /// changed to the desired value, bypassing acceleration. Of course, the
    /// mechanical characteristics of the motor determine how quickly the motor
    /// can actually achieve the new speed setting.
    ///
    /// Caution: the maximum speed achievable depends on the type of motor (DC motor 
    ///          or stepper motor). In addition, stepper motor speed is highly 
    ///          dependent on your processor and clock speed. Attempting to set
    ///          the speed faster than the motor can handle will have no effect.
    //**************************************************************************
    public: virtual void SetSpeedNow(int16_t value) = 0;

    //**************************************************************************
    /// Sets the motor brake, if so equipped. 
    ///
    /// \param[in] holdTime How long the brake should be held, in milliseconds. The
    ///                     default is 800ms.
    ///
    /// Braking helps the motor to stop spinning faster. It typically only applies 
	/// to DC motors since a DC motor controller can often be configured to short 
	/// the motor windings together, inducing a magnetic field in the motor that 
	/// opposes the motor's rotation.
	///
	/// However, holding the brake too long can cause excessive current draw by 
	/// the motor, so the braking function is designed to automatically release 
	/// the brake after a predetermined interval, as specified by the holdTime 
	/// parameter. Since the holdTime parameter is a 16-bit unsigned value, the 
	/// longest hold time is about 65 seconds.
    ///
    /// If the motor does not support a braking function then this method should 
    /// be implemented to do nothing.
    //**************************************************************************
    public: virtual void SetBrake(uint16_t holdTime=800) = 0;

    //**************************************************************************
    /// Releases the motor brake, if so equipped. 
    ///
    /// If the motor does not support a braking function then this method should 
    /// be implemented to do nothing.
    //**************************************************************************
    public: virtual void ReleaseBrake() = 0;

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
