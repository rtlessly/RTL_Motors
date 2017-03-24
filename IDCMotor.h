/*******************************************************************************
 IDCMotor.h
 Defines the abstract interface for a DC motor object
*******************************************************************************/

#ifndef _IDCMotor_h_
#define _IDCMotor_h_

#include <inttypes.h>

//**************************************************************************
// DC motor modes
//**************************************************************************
typedef enum DCMotorMode_enum
{
    FORWARD  = 1,   // Run motor in forward direction
    BACKWARD = 2,   // Run motor in backward direction
    BRAKE    = 3,   // Brakes the motor 
    RELEASE  = 4    // Stops the motor and allows it to free-run
} 
DCMotorMode;


//******************************************************************************
/// A class that defines the abstract interface for a DC motor. It defines
/// the basic operations that a DC motor needs to perform to be controlled
/// by the DCMotorController class.
//******************************************************************************
class IDCMotor 
{
    //**************************************************************************
    /// Default constructor.
    /// The constructor is protected to enforce abstract base class semantics.
    //**************************************************************************
    protected: IDCMotor(void) { };
    
    /*--------------------------------------------------------------------------
    Methods
    --------------------------------------------------------------------------*/

    //**************************************************************************
    /// Runs the motor in the specified mode. The commanded mode can be one of
    /// the MotorMode enum values (FORWARD, BACKWARD, BRAKE, or RELEASE).
    //**************************************************************************
    public: virtual void Run(DCMotorMode command) = 0;
  
    /*--------------------------------------------------------------------------
    Properties
    --------------------------------------------------------------------------*/
    //**************************************************************************
    /// Gets the motor ID (motor number). 
    //**************************************************************************
    public: virtual uint16_t ID() = 0;
      
    //**************************************************************************
    /// Gets or sets the speed of the motor. The speed is 0-255.
    //**************************************************************************
    public: virtual uint8_t Speed() = 0;
    public: virtual void Speed(uint8_t speed) = 0;
  
    //**************************************************************************
    /// Gets the current motor mode.
    /// Returns one of the MotorMode enum values (FORWARD, BACKWARD, BRAKE, or RELEASE).
    //**************************************************************************
    public: virtual DCMotorMode Mode() = 0;
};

#endif
