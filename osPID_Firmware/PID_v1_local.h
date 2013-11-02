#if !defined PID_v1_h
#define PID_v1_h
#define LIBRARY_VERSION	1.0.0

#include "ospDecimalValue.h"

class PID
{
  public:

  //Constants used in some of the functions below
  static const byte AUTOMATIC = 1;
  static const byte MANUAL    = 0;
  static const byte DIRECT    = 0;
  static const byte REVERSE   = 1;

  //commonly used functions **************************************************************************
    PID(double*, double*, double*,        // * constructor.  links the PID to the Input, Output, and 
        ospDecimalValue<3>,               //   Setpoint.  Initial tuning parameters are also set here
        ospDecimalValue<3>,
        ospDecimalValue<3>, int);
	
    void SetMode(byte Mode);              // * sets PID to either Manual (0) or Auto (non-0)

    void Compute();                       // * performs the PID calculation.  it should be
                                          //   called every time loop() cycles. ON/OFF and
                                          //   calculation frequency can be set using SetMode
                                          //   SetSampleTime respectively

    void SetOutputLimits(double, double); //clamps the output to a specific range. 0-255 by default, but
					  //it's likely the user will want to change this depending on
					  //the application
	


  //available but not commonly used functions ********************************************************
    void SetTunings(ospDecimalValue<3>,   // * While most users will set the tunings once in the    
        ospDecimalValue<3>,         	  //   constructor, this function gives the user the option  
        ospDecimalValue<3>);              //   of changing tunings during runtime for Adaptive control
    void SetControllerDirection(byte);	  // * Sets the Direction, or "Action" of the controller. DIRECT
					  //   means the output will increase when error is positive. REVERSE
				          //   means the opposite.  it's very unlikely that this will be needed
					  //   once it is set in the constructor.
    void SetSampleTime(int);              // * sets the frequency, in Milliseconds, with which 
                                          //   the PID calculation is performed.  default is 100
										  
										  
										  
  //Display functions ****************************************************************
    ospDecimalValue<3> GetKp();           // These functions query the pid for interal values.
    ospDecimalValue<3> GetKi();	          //  they were created mainly for the pid front-end,
    ospDecimalValue<3> GetKd();           // where it's important to know what is actually 
    byte GetMode();			  //  inside the PID.
    byte GetDirection();		  //

  private:
    void Initialize();
    void Limit(double*);
	
    ospDecimalValue<3> dispKp;            // * we'll hold on to the tuning parameters in user-entered 
    ospDecimalValue<3> dispKi;            //   format for display purposes
    ospDecimalValue<3> dispKd;            //
    
    double kp;                            // * (P)roportional Tuning Parameter
    double ki;                            // * (I)ntegral Tuning Parameter
    double kd;                            // * (D)erivative Tuning Parameter

    int controllerDirection;

    double *myInput;              // * Pointers to the Input, Output, and Setpoint variables
    double *myOutput;             //   This creates a hard link between the variables and the 
    double *mySetpoint;           //   PID, freeing the user from having to constantly tell us
                                  //   what these values are.  with pointers we'll just know.
			  
	unsigned long lastTime;
	double ITerm, lastInput;

	int SampleTime;
	double outMin, outMax;
	bool inAuto;
};
#endif


