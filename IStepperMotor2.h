/*******************************************************************************
 IStepperMotor2.h
 Defines the abstract interface for a stepper motor object
*******************************************************************************/

#ifndef _IStepperMotor2_h_
#define _IStepperMotor2_h_

#include <inttypes.h>


//******************************************************************************
/// The abstract interface for a stepper motor. Defines the basic operations that
/// a stepper motor needs to perform to be controlled by the StepperMotorController
/// class.
//******************************************************************************
class IStepperMotor2 
{
    /// The constructor is protected to enforce abstract base class semantics.
    protected: IStepperMotor2(void) { };
    
    //**************************************************************************
    /// Stepper motor modes
    //**************************************************************************
    public: enum MotorMode
    {
        SINGLE,      // Run motor in single coil mode
        DOUBLE,      // Run motor in double coil mode
        INTERLEAVE,  // Run motor in interleaved (half-step) coil mode
        MICROSTEP    // run motor in micro-step mode
    };

    //**************************************************************************
    /// Motor direction
	/// NOTE: Directions are aribtrary, i.e., FORWARD just means one direction 
    /// and BACKWARD means the opposite direction. The actual direction the motor
    /// turns depends on how the motor is wired up to the Arduino. 
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
    /// Runs the motor for the specified number of steps and in the specified mode. 
    /// The default mode, if not specified, is SINGLE (single-coil). If the steps 
    /// parameter is negative, the direction is BACKWARD, otherwise it is FORWARD.
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
    public: virtual void Run(int32_t steps, MotorMode mode = SINGLE, uint16_t speed=0) = 0;
    
    //**************************************************************************
    /// Advances the motor exactly one step in the specified direction.
    /// This is the key method used by the StepperMotorController class.
    //**************************************************************************
    public: virtual void OneStep(int dir) = 0;
    
    //**************************************************************************
    /// Releases the motor. In this configuration the motor is not energized and 
    /// will free-run.
    //**************************************************************************
    public: virtual void Release(void) = 0;
 
 
    /*--------------------------------------------------------------------------
    Properties
    --------------------------------------------------------------------------*/
    //**************************************************************************
    /// Gets the motor ID (motor number). 
    //**************************************************************************
    public: virtual uint8_t GetID() = 0;
  
    //**************************************************************************
    /// Gets the number of steps per one motor revolution. 
    //**************************************************************************
    public: virtual uint16_t StepsPerRev() = 0;
  
    //**************************************************************************
    /// Gets or sets the motor mode.
    //**************************************************************************
    public: virtual MotorMode GetMode() = 0;
    public: virtual void SetMode(MotorMode mode) = 0;
};

#endif
