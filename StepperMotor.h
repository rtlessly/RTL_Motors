#ifndef _StepperMotor_h_
#define _StepperMotor_h_

#include <inttypes.h>
#include <RTL_EventFramework.h>
// #include "IStepperMotor.h"


//******************************************************************************
/// 4-Wire interface for a stepper motor connected to an Arduino. This interface 
/// requires 4 digital pins on the Arduino. It is assumed that the motor is
/// connected to the Arduino via a simple pass-thru power controller such as an
/// ULN2003 or ULN2004 Darlington array. 
///  
///  !!! NEVER CONNECT STEPPER MOTOR LEADS DIRECTLY TO THE ARDUINO!!! 
///  
///  Written by R. Terry Lessly 2018-12-03.
//******************************************************************************
class StepperMotor : public IPollable
{
    DECLARE_CLASSNAME;

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
    ///
    ///       If the motor is running backwards from the desired direction you 
    ///       can either:
    ///       1) Swap FORWARD and BACKWARD directions in your code
    ///       2) Swap the "A" leads and "B" leads on the motor (NOTE: DO NOT swap
    ///          the "A" leads with the "B" leads, just switch A1 with A2 and 
    ///          B1 with B2).
    //**************************************************************************
    public: enum MotorDirection
    {
        FORWARD  =  1,
        STOPPED  =  0,
        BACKWARD = -1
    };

    //**************************************************************************
    /// Default constructor.
    //**************************************************************************
    public: StepperMotor(uint16_t stepsPerRev, uint8_t pinA1, uint8_t pinA2, uint8_t pinB1, uint8_t pinB2);

    /*--------------------------------------------------------------------------
    IPollable interface
    --------------------------------------------------------------------------*/
    public: void Poll();


    //**************************************************************************
    /// Starts the motor. The motor will not run until this method is called,
    /// and then only if the speed is non-zero. If the steps parameter is non-
    /// zero then the motor will only run for the specified number of steps and
    /// then stop. Otherwise, it runs indefinitely until Stop() is called.
    //**************************************************************************
    public: void Start(uint32_t steps=0) { _isRunning = (_speed != 0); if (steps > 0) _stepCount = steps; };

    //**************************************************************************
    /// Stops the motor.
    //**************************************************************************
    public: void Stop()  { _isRunning = false; _stepCount = 0; };

    //**************************************************************************
    /// Releases the motor by stopping it and setting all drive lines to LOW. 
    /// In this configuration the motor draws no current and will spin freely.
    //**************************************************************************
    public: void Release(void);

    //**************************************************************************
    /// Advances the motor exactly one step in the specified direction. 
    /// This is the key method used operate the motor.
    //**************************************************************************
    public: void OneStep(int8_t dir);

    //**************************************************************************
    /// Gets the motor ID (motor number).
    //**************************************************************************
    public: inline void ID(uint8_t id) { _motorState.motorNum = id & 0x03; };
    public: inline uint8_t ID() { return _motorState.motorNum; };

    //**************************************************************************
    /// Gets the number of steps per one motor revolution.
    //**************************************************************************
    public: inline uint16_t StepsPerRev() { return _stepsPerRev; };

    //**************************************************************************
    /// Gets or sets the motor mode.
    //**************************************************************************
    public: inline MotorMode Mode() { return (MotorMode)_motorState.mode; };
    public: inline void Mode(MotorMode mode) { _motorState.mode = mode; };

    //**************************************************************************
    /// Gets or sets the speed of the motor in RPM.
    //**************************************************************************
    public: int16_t Speed();
    public: void Speed(int16_t rpm);

    //**************************************************************************
    /// Gets the motor step count. The step count is only valid after the 
    /// RunSteps() method has been called and only until the step count reaches 
    /// zero. At all other times the step count is 0.
    ///
    /// \return The current motor step count. This is the number of steps until
    ///         the count reaches 0 rather than the number of steps taken so far.
    //**************************************************************************
    public: inline uint32_t Position() { return _stepCount; };

    //**************************************************************************
    /// Gets the current direction the motor is turning.
    /// \return +1 = FORWARD, -1 = BACKWARD, 0 = STOPPED
    //**************************************************************************
    public: inline int8_t Direction() { return _speed > 0 ? FORWARD : (_speed < 0 ? BACKWARD : STOPPED); };
    
    //**************************************************************************
    /// Indicates if the motor is currently running.
    /// \return true if the motor is running; otherwise false is returned.
    //**************************************************************************
    public: inline bool IsRunning() { return _isRunning; };
    
    //**************************************************************************
    /// Indicates if the motor is running for a specified number of steps.
    //**************************************************************************
    public: inline bool IsRunningSteps() { return _stepCount > 0; };

    /*--------------------------------------------------------------------------
    Internal state (size: 20 bytes)
    --------------------------------------------------------------------------*/
    private: struct                     // 2 bytes
    {
        uint16_t motorNum :  2;         // The motor number (0 - 3)
        uint16_t mode     :  2;         // The motor mode
        uint16_t pinA1    :  4;         // Motor 'A' coil lead 1 Arduino pin
        uint16_t pinA2    :  4;         // Motor 'A' coil lead 2 Arduino pin
        uint16_t pinB1    :  4;         // Motor 'B' coil lead 1 Arduino pin
        uint16_t pinB2    :  4;         // Motor 'B' coil lead 2 Arduino pin
    }
    _motorState;

    private: bool _isRunning;           // Indicates if the motor is running                1 byte
    private: uint8_t _currentStep;      // The current step number in the stepping cycle    1 byte
    private: int16_t _speed;            // The set speed of the motor in rpm                2 bytes
    private: uint16_t _stepsPerRev;     // Number of steps per motor revolution             2 bytes
    private: uint32_t _usPerStep;       // microseconds per step                            4 bytes
    private: uint32_t _lastStepTime;    // The last step time in microseconds               4 bytes
    private: uint32_t _stepCount;       // The current step count                           4 bytes
};

#endif
