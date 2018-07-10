/*******************************************************************************
 IDCMotor2.h
 Defines the abstract interface for a DC motor object
*******************************************************************************/

#ifndef _IDCMotor2_h_
#define _IDCMotor2_h_

#include <inttypes.h>


//******************************************************************************
/// A class that defines the abstract interface for a DC motor. It defines
/// the basic operations that a DC motor needs to perform.
//******************************************************************************
class IDCMotor2
{
    //**************************************************************************
    /// Default constructor.
    /// The constructor is protected to enforce abstract base class semantics.
    //**************************************************************************
    protected: IDCMotor2(void) { };
    
    /*--------------------------------------------------------------------------
    Methods
    --------------------------------------------------------------------------*/
    //**************************************************************************
    /// Runs the motor at the specified speed. 
    /// 
    /// Parameter: speed
    /// The speed to run the motor. The speed is constrained to be in the range
    /// +/- 255, where +255 is max forward speed, -255 is max backward speed, 
    /// and 0 is stopped.
    /// 
    /// Note: Other than the fact that they should be opposite directions, the 
    /// concept of "forward" and "backward" is implementation dependent, i.e., 
    /// the implementation can decide which direction is "forward" or "backward" 
    /// by wiring the motor appropriately.
    //**************************************************************************
    public: virtual void Run(int16_t speed) = 0;

  
    /*--------------------------------------------------------------------------
    Properties
    --------------------------------------------------------------------------*/

    //**************************************************************************
    /// Gets the motor ID.
    //**************************************************************************
    public: virtual uint16_t GetID() = 0;

    //**************************************************************************
    /// Gets the current speed of the motor. 
    ///
    /// Returns: 
    /// The currently set speed of the motor, in the range +/- 255, where +255 
    /// is max forward speed, -255 is max backward speed, and 0 is stopped.
    //**************************************************************************
    public: virtual int16_t GetSpeed() = 0;
};

#endif
