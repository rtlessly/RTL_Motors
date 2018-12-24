/*******************************************************************************
 IStepperMotor.h
 Defines the abstract interface for a stepper motor object
*******************************************************************************/

#ifndef _IStepperMotor_h_
#define _IStepperMotor_h_

#include <inttypes.h>


//******************************************************************************
/// The abstract interface for a stepper motor. Defines the basic operations that
/// a stepper motor needs to support to be controlled by the StepperMotorController
/// class.
//******************************************************************************
class IStepperMotor 
{
    /// The constructor is protected to enforce abstract base class semantics.
    protected: IStepperMotor(void) { };

    
    //**************************************************************************
    /// Stepper motor modes
    //**************************************************************************
    public: enum MotorMode
    {
        SINGLE,      // Run motor in single coil mode
        DOUBLE,      // Run motor in double coil mode
        INTERLEAVE,  // Run motor in interleaved (half-step) coil mode
        //MICROSTEP    // run motor in micro-step mode
    };

    //**************************************************************************
    /// Motor direction
    /// NOTE: Directions are aribtrary, i.e., FORWARD just means one direction 
    ///       and BACKWARD means the opposite direction. The actual direction the 
    ///       motor turns depends on how the motor is wired up to the Arduino. 
    //**************************************************************************
    public: enum MotorDirection
    {
        FORWARD  =  1,
        STOPPED  =  0,
        BACKWARD = -1
    };

    
    /*--------------------------------------------------------------------------
    Methods
    --------------------------------------------------------------------------*/

    //**************************************************************************
    /// Starts the motor. The motor will not run until this method is called.
    //**************************************************************************
	public: virtual void Start() = 0;

    //**************************************************************************
    /// Stops the motor.
    //**************************************************************************
	public: virtual void Stop() = 0;
    
    //**************************************************************************
    /// Releases the motor. In this configuration the motor is not energized and 
    /// will free-run.
    //**************************************************************************
    public: virtual void Release(void) = 0;
	
    //**************************************************************************
    /// Runs the motor for the specified number of steps. The direction is 
	/// determined by the speed parameter, if it is positive the direction is 
	/// FORWARD, if it is negative the direction is BACKWARD.
    ///
    /// NOTE: This is a blocking call - it will not return until the run has 
    ///       completed. In the meantime, your Arduino will be doing nothing else.
    ///       This method is provided primarily for simple testing of stepper
    ///       motors. The StepeprMotorController class provides an interface for 
    ///       more robust, non-blocking control of the motor.
    ///
    /// NOTE: The number of steps is always full motor steps for SINGLE, DOUBLE,
    ///       and MICROSTEP modes, and half-steps for INTERLEAVE mode.
    //**************************************************************************
    public: virtual void RunSteps(uint32_t steps) = 0;
    
    //**************************************************************************
    /// Advances the motor exactly one step in the specified direction. 
    /// This is the key method used by the StepperMotorController class.
    //**************************************************************************
    public: virtual void OneStep(uint8_t dir) = 0;
 
 
    /*--------------------------------------------------------------------------
    Properties
    --------------------------------------------------------------------------*/
    //**************************************************************************
    /// Gets the motor ID (motor number). 
    //**************************************************************************
    public: virtual uint8_t ID() = 0;
  
    //**************************************************************************
    /// Gets the number of steps per one motor revolution. 
    //**************************************************************************
    public: virtual uint16_t StepsPerRev() = 0;
  
    //**************************************************************************
    /// Gets or sets the motor mode.
    //**************************************************************************
    public: virtual MotorMode Mode() = 0;
    public: virtual void Mode(MotorMode mode) = 0;

    //**************************************************************************
    /// Gets or sets the current motor position, in steps. 
    ///
    /// \param[in] value The new position in steps.
    /// \return The current motor position in steps. 
    ///
    /// Positive values are in the FORWARD direction from the 0 position, negative 
    /// values are in the BACKWARD direction.
    /// When setting the current position of the motor, wherever the motor happens
    /// to be at that moment is considered to be the new specified position. 
    //**************************************************************************
    public: virtual int32_t Position() = 0;
    public: virtual void Position(int32_t value) = 0;

    //**************************************************************************
    /// Gets or sets the speed of the motor in RPM.
    //**************************************************************************
    public: virtual int16_t Speed() = 0;
    public: virtual void Speed(int16_t rpm) = 0;

    //**************************************************************************
    /// Gets the current direction the motor is turning.
    /// \return +1 = FORWARD, -1 = BACKWARD
    //**************************************************************************
    public: virtual int8_t Direction() = 0;

    //**************************************************************************
    /// Indicates if the motor is currently running.
    /// \return true if the motor is running; otherwise false is returned.
    //**************************************************************************
    public: virtual bool IsRunning() = 0;
};

#endif
