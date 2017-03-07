/*******************************************************************************
 IStepperMotor.h
 Defines the abstract interface for a stepper motor object
*******************************************************************************/

#ifndef IStepperMotor_h
#define IStepperMotor_h

#include <inttypes.h>


//******************************************************************************
/// A class that defines the abstract interface for a stepper motor. It defines
/// the basic operations that a stepper motor needs to perform to be controlled
/// by the StepperMotorController class.
//******************************************************************************
class IStepperMotor 
{
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
    //**************************************************************************
    public: enum MotorDirection
    {
        FORWARD  =  1,
        STOPPED  =  0,
        BACKWARD = -1
    };

    //**************************************************************************
    /// Default constructor.
    /// The constructor is protected to enforce abstract base class semantics.
    /// Call the Initialize() method to perform the necessary motor configuration.
    //**************************************************************************
    protected: IStepperMotor(void) { };

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
    /// Releases the motor by setting all drive lines to LOW. In this configuration
    /// the motor will free-run.
    //**************************************************************************
    public: virtual void Release(void) = 0;
  
    /*--------------------------------------------------------------------------
    Properties
    --------------------------------------------------------------------------*/
    //**************************************************************************
    /// Gets the motor ID (motor number). 
    //**************************************************************************
    public: virtual uint16_t ID() = 0;
  
    //**************************************************************************
    /// Gets the number of steps per one motor revolution. 
    //**************************************************************************
    public: virtual uint16_t StepsPerRev() = 0;
  
    //**************************************************************************
    /// Gets or sets the motor mode.
    //**************************************************************************
    public: virtual MotorMode Mode() = 0;
    public: virtual void Mode(MotorMode mode) = 0;
};

#endif
