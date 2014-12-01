/**********************************************************************************************
 * PID engine with auto tune - Version 2.0.0
 * 
 * original Arduino libraries by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 * adapted by Tom Price <magicsmoke@tomprice.net> http://smokedprojects.blogspot.com
 *
 * This Library is licensed under a GPLv3 License
 **********************************************************************************************/

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "osPID_Engine.h" 

// prototype definitions

extern ospDecimalValue<1> manualOutput;
extern ospDecimalValue<3> PGain;
extern ospDecimalValue<3> IGain;
extern ospDecimalValue<3> DGain;

extern void markSettingsDirty();
extern void setOutputToManualOutput();


// source of Tyreus-Luyben and Ciancone-Marlin rules:
// "Autotuning of PID Controllers: A Relay Feedback Approach",
//  by Cheng-Ching Yu, 2nd Edition, p.18
// Tyreus-Luyben is more conservative than Ziegler-Nichols
// and is preferred for lag dominated processes
// Ciancone-Marlin is preferred for delay dominated processes
// Ziegler-Nichols is intended for best disturbance rejection
// can lack robustness especially for lag dominated processes

// source for Pessen Integral, Some Overshoot, and No Overshoot rules:
// "Rule-Based Autotuning Based on Frequency Domain Identification" 
// by Anthony S. McCormack and Keith R. Godfrey
// IEEE Transactions on Control Systems Technology, vol 6 no 1, January 1998.
// as reported on http://www.mstarlabs.com/control/znrule.html

// order must be match enumerated type for auto tune methods
PROGMEM Tuning tuningRule[PID::NO_OVERSHOOT_PID + 1] =
{  
  { {  44, 24,   0 } },  // ZIEGLER_NICHOLS_PI
  { {  34, 40, 160 } },  // ZIEGLER_NICHOLS_PID
  { {  64,  9,   0 } },  // TYREUS_LUYBEN_PI
  { {  44,  9, 126 } },  // TYREUS_LUYBEN_PID
  { {  66, 80,   0 } },  // CIANCONE_MARLIN_PI
  { {  66, 88, 162 } },  // CIANCONE_MARLIN_PID
  { {  28, 50, 133 } },  // PESSEN_INTEGRAL_PID
  { {  60, 40,  60 } },  // SOME_OVERSHOOT_PID
  { { 100, 40,  60 } }   // NO_OVERSHOOT_PID
};

/*Constructor (...)*********************************************************
* The parameters specified here are those for for which we can't set up
* reliable defaults, so we need to have the user set them.
***************************************************************************/
PID::PID(double* Input, double* Output, double* Setpoint,
        ospDecimalValue<3> Kp, ospDecimalValue<3> Ki, ospDecimalValue<3> Kd, byte ControllerDirection)
{
  // default output limit corresponds to
  // the arduino pwm limits
  //this is unneeded as you setOutputLimits(0,100) in void.setup 
  // redefining as 255 messes up the math behind PID
  //PID::setOutputLimits(0, 255); 
 
  // default Controller Sample Time is 0.1 seconds
  sampleTime = DEFAULT_LOOP_SAMPLE_TIME;

  // set PID parameters
  myOutput = Output;
  myInput = Input;
  mySetpoint = Setpoint;
  setTunings(Kp, Ki, Kd);
  controllerDirection = ControllerDirection;

  // initialize internal variables
  lastTime = millis() - (unsigned long) sampleTime;
  isTuning = false;
}
 
/* compute() **********************************************************************
* This, as they say, is where the magic happens. this function should be called
* every time "void loop()" executes. the function will decide for itself whether a new
* pid Output needs to be computed
**********************************************************************************/
void PID::compute()
{
  // is it time yet
  unsigned long now = millis();
  unsigned long timeChange = (now - lastTime);
  if (timeChange < (unsigned long) sampleTime)
  {
    return;
  }
  lastTime = now;
  
  if (isTuning)
  {
    // run auto tuner
    bool finishedTuning = autoTune();
    if (finishedTuning)
    {
      isTuning = false;
      completeAutoTune();
    }
    return;
  }

  if (mode == MANUAL) 
  {
   return;
  }  
  
  // compute output using PID engine
  
  // compute all the working error variables
  double input = *myInput;
  double error = *mySetpoint - input;
  iTerm += (ki * error);
  limit(&iTerm);
  double dInput = (input - lastInput);
 
  // compute PID Output
  double output = kp * error + iTerm - kd * dInput;
      
  limit(&output); 
  *myOutput = output;

  // remember some variables for next time
  lastInput = input;
}

/* limit(...)*****************************************************************
*  Applies outMin and outMax limits to the supplied variable
******************************************************************************/
void PID::limit(double *var)
{
  if (*var > outMax) 
  {
    *var = outMax;
  }
  else if (*var < outMin) 
  {
    *var = outMin;
  }
}

/* setTunings(...)*************************************************************
* This function allows the controller's dynamic performance to be adjusted.
* it's called automatically from the constructor, but tunings can also
* be adjusted on the fly during normal operation
******************************************************************************/
void PID::setTunings(ospDecimalValue<3> Kp, ospDecimalValue<3> Ki, ospDecimalValue<3> Kd)
{
  if (
    (Kp < (ospDecimalValue<3>){0.0}) || 
    (Ki < (ospDecimalValue<3>){0.0}) || 
    (Kd < (ospDecimalValue<3>){0.0}) 
  )
  {
    return;
  }
 
  dispKp = Kp; 
  dispKi = Ki; 
  dispKd = Kd;
   
  double sampleTimeInSec = sampleTime * 0.001;
  kp = double(Kp);
  ki = double(Ki) * sampleTimeInSec;
  kd = double(Kd) / sampleTimeInSec;
 
  if (controllerDirection == REVERSE)
  {
    kp = (0.0 - kp);
    ki = (0.0 - ki);
    kd = (0.0 - kd);
  }
}
  
/* setSampleTime(...) *********************************************************
* sets the period, in Milliseconds, at which the calculation is performed
******************************************************************************/
void PID::setSampleTime(int newSampleTime)
{
  if (newSampleTime > 0)
  {
    double ratio = (double) newSampleTime / (double) sampleTime;
    ki *= ratio;
    kd /= ratio;
    sampleTime = newSampleTime;
  }
}
 
/* setOutputLimits(...)****************************************************
* This function will be used far more often than SetInputLimits. while
* the input to the controller will generally be in the 0-1023 range (which is
* the default already,) the output will be a little different. maybe they'll
* be doing a time window and will need 0-8000 or something. or maybe they'll
* want to clamp it from 0-125. who knows. at any rate, that can all be done
* here.
**************************************************************************/
void PID::setOutputLimits(double Min, double Max)
{
  if (Min >= Max) 
  {
    return;
  }
  outMin = Min;
  outMax = Max;
 
  if (mode == AUTOMATIC)
  {
    limit(myOutput);
    limit(&iTerm);  
  }
}

/* setMode(...)****************************************************************
* Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
* when the transition from manual to auto occurs, the controller is
* automatically initialized
******************************************************************************/
void PID::setMode(byte newMode)
{
  if (newMode != mode)
  { 
    // we just went from manual to auto
    PID::initialize();
  }
  mode = newMode;
}
 
/* initialize()****************************************************************
* does all the things that need to happen to ensure a bumpless transfer
* from manual to automatic mode.
******************************************************************************/
void PID::initialize()
{
  iTerm = *myOutput;
  lastInput = *myInput;
  limit(&iTerm);
}

/* SetControllerDirection(...)*************************************************
* The PID will either be connected to a DIRECT acting process (+Output leads
* to +Input) or a REVERSE acting process(+Output leads to -Input.) we need to
* know which one, because otherwise we may increase the output when we should
* be decreasing. This is called from the constructor.
******************************************************************************/
void PID::setControllerDirection(byte newDirection)
{
  if ((mode == AUTOMATIC) && (newDirection != controllerDirection))
  {
    kp = (0.0 - kp);
    ki = (0.0 - ki);
    kd = (0.0 - kd);
    controllerDirection = newDirection;
  }
}

/* Status Functions************************************************************
* Just because you set the Kp=-1 doesn't mean it actually happened. these
* functions query the internal state of the PID. they're here for display
* purposes. this are the functions the PID Front-end uses for example
******************************************************************************/
ospDecimalValue<3> PID::getKp() { return dispKp; }
ospDecimalValue<3> PID::getKi() { return dispKi; }
ospDecimalValue<3> PID::getKd() { return dispKd; }
byte PID::getMode() { return mode; }
byte PID::getDirection() { return controllerDirection; }

 /*******************************************************************************
 *
 *
 *                       AUTO  TUNE  SET  &  GET  METHODS
 *
 *
 *******************************************************************************/
 
double PID::getAtuneKp()
{
  return Kp;
}

double PID::getAtuneKi()
{
  return Kp / Ti;
}

double PID::getAtuneKd()
{
  return Kp * Td; 
}

void PID::setAtuneOutputStep(ospDecimalValue<1> newStep)
{
  oStep = double(newStep);
}

double PID::getAtuneOutputStep()
{
  return oStep;
}

void PID::setAtuneControlType(byte newType) 
{
  controlType = newType;
}

byte PID::getAtuneControlType()
{
  return controlType;
}

void PID::setAtuneNoiseBand(ospDecimalValue<3> newBand)
{
  noiseBand = double(newBand);
}

double PID::getAtuneNoiseBand()
{
  return noiseBand;
}

void PID::setAtuneLookBackSec(int value)
{
  if (value < 1) 
  {
    value = 1;
  }
  nLookBack = (int) (value * 1000) / sampleTime;
  if (nLookBack > 100)
  {
    nLookBack = 100;
  }
}

int PID::getAtuneLookBackSec()
{
  return (int) ((double) (nLookBack * sampleTime) / 1000.0f);
}
 
 /*******************************************************************************
 *
 *
 *                           MAIN  AUTO  TUNE  METHODS
 *
 *
 *******************************************************************************/
 
void PID::startAutoTune(byte aTuneMethod, ospDecimalValue<1> aTuneStep, 
  ospDecimalValue<3> aTuneNoise, int aTuneLookBack)
{
  // start auto tune
  
  // save mode and output
  ATuneModeRemember = mode;
  manualOutputRemember = manualOutput; 
  
  // calculate step value, avoiding output limits 
  ospDecimalValue<1> s    = aTuneStep;
  ospDecimalValue<1> out  = makeDecimal<1>(*myOutput);
  ospDecimalValue<1> oMin = makeDecimal<1>(outMin);
  ospDecimalValue<1> oMax = makeDecimal<1>(outMax);
  if (s > (out - oMin))
  {
    s = out - oMin; 
  }
  if (s > (oMax - out))
  {
    s = (oMax - out);
  }

  // set auto tune parameters
  setAtuneOutputStep(s);
  setAtuneControlType(aTuneMethod); 
  setAtuneNoiseBand(aTuneNoise);
  setAtuneLookBackSec(aTuneLookBack);
  
  // initialize auto tune
  mode = MANUAL;
  isTuning = true;
  state = AUTOTUNE_OFF;
}

void PID::completeAutoTune()
{
  // auto tune finished, set the tuning parameters
  PGain = makeDecimal<3>(getAtuneKp());
  IGain = makeDecimal<3>(getAtuneKi());
  DGain = makeDecimal<3>(getAtuneKd());

  // set the PID controller to accept the new gain settings
  // use whatever direction of control is currently set
  mode = PID::AUTOMATIC;

  if (PGain < (ospDecimalValue<3>){0})
  {
    // the auto-tuner found a negative gain sign: convert the coefficients
    // to positive and change the direction of controller action
    PGain = -PGain;
    IGain = -IGain;
    DGain = -DGain;
    if (controllerDirection == PID::DIRECT)
    {
      controllerDirection = PID::REVERSE;
    }
    else
    {
      controllerDirection = PID::DIRECT;
    }      
  }
 
  // set tunings
  setTunings(PGain, IGain, DGain);

  // restore user-requested PID controller mode
  stopAutoTune();

  markSettingsDirty(); 
}

void PID::stopAutoTune()
{
  // stop auto tune
  state = AUTOTUNE_OFF;
  isTuning = false;
  mode = ATuneModeRemember;

  // restore the output to the last manual command; it will be overwritten by the PID
  // if the loop is active
  manualOutput = manualOutputRemember;
  setOutputToManualOutput();
}

 /*******************************************************************************
 *
 *
 *                           PRIVATE AUTO  TUNE  METHODS
 *
 *
 *******************************************************************************/
 
bool PID::zero(double x)
{
  return (x < 1e-10);
}

bool PID::autoTune()
{
  unsigned long now = lastTime;
  if (state == AUTOTUNE_OFF)
  { 
    // initialize working variables the first time around
    peakType = NOT_A_PEAK;
    inputCount = 0;
    peakCount = 0;
    lastPeakTime[0] = now;
    setpoint = *myInput;
    inputOffset = setpoint;
    inputOffsetChange = (ospDecimalValue<3>){0};
    outputStart = *myOutput;
    workingNoiseBand = noiseBand;
    workingOstep = oStep;

#if defined (AUTOTUNE_AMIGOF_PI)  
    newWorkingNoiseBand = workingNoiseBand;  
#endif
    
#if defined (AUTOTUNE_RELAY_BIAS) 
    relayBias = 0.0;
    stepCount = 0;   
    lastStepTime[0] = now;
    sumInputSinceLastStep[0] = 0.0;
#endif    
    
    // move to new state

#if defined (AUTOTUNE_AMIGOF_PI)    
    if (controlType == AMIGOF_PI)
    {
      state = AUTOTUNE_STEADY_STATE_AT_BASELINE;
    }
    else
    {
      state = AUTOTUNE_RELAY_STEP_UP;
    }
#else
    state = AUTOTUNE_RELAY_STEP_UP;
#endif

  }

  // get new input
  double refVal = *myInput;

#if defined (AUTOTUNE_RELAY_BIAS) 
  // used to calculate relay bias
  sumInputSinceLastStep[0] += refVal;
#endif  

  // local flag variable
  bool justChanged = false; 

  // check input and change relay state if necessary
  if ((state == AUTOTUNE_RELAY_STEP_UP) && (refVal > setpoint + workingNoiseBand))
  {
    state = AUTOTUNE_RELAY_STEP_DOWN;
    justChanged = true;
  }
  else if ((state == AUTOTUNE_RELAY_STEP_DOWN) && (refVal < setpoint - workingNoiseBand))
  {
    state = AUTOTUNE_RELAY_STEP_UP;
    justChanged = true;
  }
  if (justChanged)
  {

#if defined (AUTOTUNE_AMIGOF_PI)       
    workingNoiseBand = newWorkingNoiseBand;
#endif    
    
#if defined (AUTOTUNE_RELAY_BIAS)
    // check symmetry of oscillation
    // and introduce relay bias if necessary
    if (stepCount > 4)
    {
      // don't need to divide by 2 to get the average, we are interested in the ratio
      double avgStep1 = (double) ((lastStepTime[0] - lastStepTime[1]) + (lastStepTime[2] - lastStepTime[3]));
      double avgStep2 = (double) ((lastStepTime[1] - lastStepTime[2]) + (lastStepTime[3] - lastStepTime[4]));
      if (!zero(avgStep1) && !zero(avgStep2))
      {
        double asymmetry = (avgStep1 > avgStep2) ?
                           (avgStep1 - avgStep2) / avgStep1 : (avgStep2 - avgStep1) / avgStep2;
                           
#if defined (AUTOTUNE_DEBUG) || defined (USE_SIMULATOR)
        Serial.print(F("asymmetry "));
        Serial.println(asymmetry);
#endif

        if (asymmetry > AUTOTUNE_STEP_ASYMMETRY_TOLERANCE)
        {
          // relay steps are asymmetric
          // calculate relay bias using
          // "Autotuning of PID Controllers: A Relay Feedback Approach",
          //  by Cheng-Ching Yu, 2nd Edition, equation 7.39, p. 148

          // calculate change in relay bias
          double deltaRelayBias = - processValueOffset(avgStep1, avgStep2) * workingOstep;
          if (state == AUTOTUNE_RELAY_STEP_DOWN)
          {
            deltaRelayBias = -deltaRelayBias;
          }
          
          if (abs(deltaRelayBias) > workingOstep * AUTOTUNE_STEP_ASYMMETRY_TOLERANCE)
          {
            // change is large enough to bother with
            relayBias += deltaRelayBias;
            
            // adjust step height with respect to output limits
            // necessarily know what the output limits are
            double relayHigh = outputStart + workingOstep + relayBias;
            double relayLow  = outputStart - workingOstep + relayBias;
            if (relayHigh > outMax)
            {
              relayHigh = outMax;
            }
            if (relayLow  < outMin)
            {
              relayHigh = outMin;
            }
            workingOstep = 0.5 * (relayHigh - relayLow);
            relayBias = relayHigh - outputStart - workingOstep;
          
#if defined (AUTOTUNE_DEBUG) || defined (USE_SIMULATOR)
            Serial.print(F("deltaRelayBias "));
            Serial.println(deltaRelayBias);
            Serial.print(F("relayBias "));
            Serial.println(relayBias);
            Serial.print(F("workingOstep "));
            Serial.println(workingOstep);
#endif

            // reset relay step counter
            // to give the process value oscillation
            // time to settle with the new relay bias value
            stepCount = 0;
          }
        }
      }
    } // if justChanged

    // shift step time and integrated process value arrays
    for (byte i = (stepCount > 4 ? 4 : stepCount); i > 0; i--)
    {
      lastStepTime[i] = lastStepTime[i - 1];
      sumInputSinceLastStep[i] = sumInputSinceLastStep[i - 1];
    }
    stepCount++;
    lastStepTime[0] = now;
    sumInputSinceLastStep[0] = 0.0;
    
#if defined (AUTOTUNE_DEBUG)
    for (byte i = 1; i < (stepCount > 4 ? 5 : stepCount); i++)
    {
      Serial.print(F("step time "));
      Serial.println(lastStepTime[i]);
      Serial.print(F("step sum "));
      Serial.println(sumInputSinceLastStep[i]);
    }
#endif

#endif // if defined (AUTOTUNE_RELAY_BIAS)

  } // if justChanged

  // set output
  if (((byte) state & (AUTOTUNE_STEADY_STATE_AFTER_STEP_UP | AUTOTUNE_RELAY_STEP_UP)) > 0)
  {
    
#if defined (AUTOTUNE_RELAY_BIAS)    
    *myOutput = outputStart + workingOstep + relayBias;
#else    
    *myOutput = outputStart + workingOstep;
#endif    

  }
  else if (state == AUTOTUNE_RELAY_STEP_DOWN)
  {
    
#if defined (AUTOTUNE_RELAY_BIAS)    
    *myOutput = outputStart - workingOstep + relayBias;
#else
    *myOutput = outputStart - workingOstep;
#endif

  }
  
#if defined (AUTOTUNE_DEBUG)
  Serial.print(F("refVal "));
  Serial.println(refVal);
  Serial.print(F("setpoint "));
  Serial.println(setpoint);
  Serial.print(F("output "));
  Serial.println(*myOutput);
  Serial.print(F("state "));
  Serial.println(state);
#endif

  // store initial inputs
  // we don't want to trust the maxes or mins
  // until the input array is full
  inputCount++;
  if (inputCount <= nLookBack)
  {
    lastInputs[nLookBack - inputCount] = makeDecimal<3>(refVal - inputOffset);
    return false;
  }

  // shift array of process values and identify peaks
  inputCount = nLookBack;
  ospDecimalValue<3> iMax = lastInputs[0];
  ospDecimalValue<3> iMin = lastInputs[0];
  for (int i = inputCount - 1; i >= 0; i--)
  {
    ospDecimalValue<3> nextVal = lastInputs[i];
    if (iMax < nextVal)
    {
      iMax = nextVal;
    }
      if (iMin > nextVal)
    {
      iMin = nextVal;
    }
    lastInputs[i + 1] = nextVal - inputOffsetChange;
  }
  ospDecimalValue<3> val = makeDecimal<3>(refVal - inputOffset);
  lastInputs[0] = val - inputOffsetChange; 
  bool isMax = (val >= iMax);
  bool isMin = (val <= iMin);
  
  // recalculate temperature offset in lastInputs[]
  inputOffset += double(inputOffsetChange);
  ospDecimalValue<3> midRange  = ((iMax + iMin) * (ospDecimalValue<3>){500}).rescale<3>();
  inputOffsetChange = midRange - inputOffsetChange;

#if defined (AUTOTUNE_AMIGOF_PI)
  // for AMIGOf tuning rule, perform an initial
  // step change to calculate process gain K_process
  // this may be very slow for lag-dominated processes
  // and may never terminate for integrating processes 
  if (((byte) state & (AUTOTUNE_STEADY_STATE_AT_BASELINE | AUTOTUNE_STEADY_STATE_AFTER_STEP_UP)) > 0)
  {
    // check that all the recent inputs are 
    // equal give or take expected noise
    if (double(iMax - iMin) <= 2.0 * workingNoiseBand)
    {
      
#if defined (AUTOTUNE_RELAY_BIAS)      
      lastStepTime[0] = now;
#endif
        
#if defined (AUTOTUNE_DEBUG) || defined (USE_SIMULATOR)
        Serial.print(F("steady at "));
        Serial.print(inputOffset + double(inputOffsetChange));
        Serial.print(F(" with output "));
        Serial.println(*myOutput);
#endif

      if (state == AUTOTUNE_STEADY_STATE_AT_BASELINE)
      {
        state = AUTOTUNE_STEADY_STATE_AFTER_STEP_UP;
        lastPeaks[0] = inputOffset + double(inputOffsetChange);  
        inputCount = 0;
        inputOffset = lastPeaks[0];
        return false;
      }
      // else state == STEADY_STATE_AFTER_STEP_UP
      
      // calculate process gain
      K_process = (inputOffset + double(inputOffsetChange) - lastPeaks[0]) / workingOstep;

#if defined (AUTOTUNE_DEBUG) || defined (USE_SIMULATOR)
      Serial.print(F("Process gain "));
      Serial.println(K_process);
#endif

      // bad estimate of process gain
      if (zero(K_process))
      {
        state = AUTOTUNE_FAILED;
        return false;
      }
      state = AUTOTUNE_RELAY_STEP_DOWN;

#if defined (AUTOTUNE_RELAY_BIAS)      
      sumInputSinceLastStep[0] = 0.0;
#endif

      return false;
    }
    else
    {
      return false;
    }
  }
#endif // if defined (AUTOTUNE_AMIGOF_PI)  
  
  // increment peak count 
  // and record peak time 
  // for both maxima and minima 
  justChanged = false;
  if (isMax)
  {
    if (peakType == MINIMUM)
    {
      justChanged = true;
    }
    peakType = MAXIMUM;
  }
  else if (isMin)
  {
    if (peakType == MAXIMUM)
    {
      justChanged = true;
    }
    peakType = MINIMUM;
  }

  // update peak times and values
  if (justChanged)
  {
    peakCount++;

#if defined (AUTOTUNE_DEBUG) || defined (USE_SIMULATOR)
    Serial.println(F("peakCount "));
    Serial.println(peakCount);
    Serial.println(F("peaks"));
    for (byte i = 0; i < (peakCount > 4 ? 5 : peakCount); i++)
    {
      Serial.println(lastPeaks[i]);
    }
#endif

    // shift peak time and peak value arrays
    for (byte i = (peakCount > 4 ? 4 : peakCount); i > 0; i--)
    {
      lastPeakTime[i] = lastPeakTime[i - 1];
      lastPeaks[i] = lastPeaks[i - 1];
    }
  }
  if (isMax || isMin)
  {
    lastPeakTime[0] = now;
    lastPeaks[0] = refVal;

#if defined (AUTOTUNE_DEBUG)
    Serial.println();
    Serial.println(F("peakCount "));
    Serial.println(peakCount);
    Serial.println(F("refVal "));
    Serial.println(refVal);
    Serial.print(F("peak type "));
    Serial.println(peakType);
    Serial.print(F("isMin "));
    Serial.println(isMin);
    Serial.print(F("isMax "));
    Serial.println(isMax);
    Serial.println();
    Serial.println(F("lastInputs:"));
    for (byte i = 0; i <= inputCount; i++)
    {
      Serial.println(lastInputs[i]);
    }
    Serial.println();
#endif

  }

  // check for convergence of induced oscillation
  // convergence of amplitude assessed on last 4 peaks (1.5 cycles)
  double inducedAmplitude = 0.0;
  double phaseLag;
  if (
  
#if defined (AUTOTUNE_RELAY_BIAS)  
    (stepCount > 4) &&
#endif

    justChanged && 
    (peakCount > 4)
  )
  { 
    double absMax = lastPeaks[1];
    double absMin = lastPeaks[1];
    for (byte i = 2; i <= 4; i++)
    {
      double val = lastPeaks[i];
      inducedAmplitude += abs( val - lastPeaks[i - 1]); 
      if (absMax < val)
      {
         absMax = val;
      }
      if (absMin > val)
      {
         absMin = val;
      }
    }
    inducedAmplitude /= 6.0;

#if defined (AUTOTUNE_DEBUG) || defined (USE_SIMULATOR)
    Serial.print(F("amplitude "));
    Serial.println(inducedAmplitude);
    Serial.print(F("absMin "));
    Serial.println(absMin);
    Serial.print(F("absMax "));
    Serial.println(absMax);
    Serial.print(F("convergence criterion "));
    Serial.println((0.5 * (absMax - absMin) - inducedAmplitude) / inducedAmplitude);
#endif

#if defined (AUTOTUNE_AMIGOF_PI)
    // source for AMIGOf PI auto tuning method:
    // "Revisiting the Ziegler-Nichols tuning rules for PI control — 
    //  Part II. The frequency response method."
    // T. Hägglund and K. J. Åström
    // Asian Journal of Control, Vol. 6, No. 4, pp. 469-482, December 2004
    // http://www.ajc.org.tw/pages/paper/6.4PD/AC0604-P469-FR0371.pdf
    if (controlType == AMIGOF_PI)
    {
      phaseLag = calculatePhaseLag(inducedAmplitude);

#if defined (AUTOTUNE_DEBUG) || defined (USE_SIMULATOR)
      Serial.print(F("phase lag "));
      Serial.println(phaseLag / CONST_PI * 180.0);
#endif

      // check that phase lag is within acceptable bounds, ideally between 120° and 140°
      // but 115° to 145° will just about do, and might converge quicker
      if (abs(phaseLag - CONST_PI * 130.0 / 180.0) > (CONST_PI * 15.0 / 180.0))
      {
        // phase lag outside the desired range
        // set noiseBand to new estimate
        // aiming for 135° = 0.75 * pi (radians)
        // sin(135°) = sqrt(2)/2
        // NB noiseBand = 0.5 * hysteresis
        newWorkingNoiseBand = inducedAmplitude * 0.5 * CONST_SQRT2_DIV_2;

#if defined (AUTOTUNE_RELAY_BIAS)
        // we could reset relay step counter because we can't rely
        // on constant phase lag for calculating
        // relay bias having changed noiseBand
        // but this would essentially preclude using relay bias
        // with AMIGOf tuning, which is already a compile option
        /* 
        stepCount = 0;
        */
#endif        

#if defined (AUTOTUNE_DEBUG) || defined (USE_SIMULATOR)
        Serial.print(F("newWorkingNoiseBand "));
        Serial.println(newWorkingNoiseBand);   
#endif

        return false;
      }
    }
#endif // if defined (AUTOTUNE_AMIGOF_PI)    

    // check convergence criterion for amplitude of induced oscillation
    if (((0.5 * (absMax - absMin) - inducedAmplitude) / inducedAmplitude) < AUTOTUNE_PEAK_AMPLITUDE_TOLERANCE)
    {
      state = AUTOTUNE_CONVERGED;
    }
  }
    
  // if the autotune has not already converged
  // terminate after 10 cycles 
  // or if too long between peaks
  // or if too long between relay steps
  if (

#if defined (AUTOTUNE_RELAY_BIAS)  
    ((now - lastStepTime[0]) > AUTOTUNE_MAX_WAIT) ||
#endif

    ((now - lastPeakTime[0]) > AUTOTUNE_MAX_WAIT) ||
    (peakCount >= 20)
  )
  {
    state = AUTOTUNE_FAILED;
  }
  
  if (((byte) state & (AUTOTUNE_CONVERGED | AUTOTUNE_FAILED)) == 0)
  {
    return false;
  }

  // autotune algorithm has terminated 
  // reset autotuner variables
  *myOutput = outputStart;

  if (state == AUTOTUNE_FAILED)
  {
    // do not calculate gain parameters
    
#if defined (AUTOTUNE_DEBUG) || defined (USE_SIMULATOR)
    Serial.println("failed");
#endif

    return true;
  }

  // finish up by calculating tuning parameters
  
  // calculate ultimate gain
  double Ku = (4.0 / CONST_PI) * (workingOstep / inducedAmplitude); 

#if defined (AUTOTUNE_DEBUG) || defined (USE_SIMULATOR)
  Serial.print(F("ultimate gain "));
  Serial.println(1.0 / Ku);
#endif

  // calculate ultimate period in seconds
  double Pu = (double) ((lastPeakTime[1] - lastPeakTime[3]) + (lastPeakTime[2] - lastPeakTime[4])) / 2000.0;  
  
#if defined (AUTOTUNE_DEBUG) || defined (USE_SIMULATOR)
  Serial.print(F("ultimate period "));
  Serial.println(Pu);
#endif 

  // calculate gain parameters using tuning rules
  // NB PID generally outperforms PI for lag-dominated processes
    
#if defined (AUTOTUNE_AMIGOF_PI)
  // AMIGOf is slow to tune, especially for lag-dominated processes, because it
  // requires an estimate of the process gain which is implemented in this
  // routine by steady state change in process variable after step change in set point
  // It is intended to give robust tunings for both lag- and delay- dominated processes
  if (controlType == AMIGOF_PI)
  {
    // calculate gain ratio
    double kappa_phi = (1.0 / Ku) / K_process;

#if defined (AUTOTUNE_DEBUG) || defined (USE_SIMULATOR)
  Serial.print(F("gain ratio kappa "));
  Serial.println(kappa_phi);
#endif
  
    // calculate phase lag
    phaseLag = calculatePhaseLag(inducedAmplitude);

#if defined (AUTOTUNE_DEBUG) || defined (USE_SIMULATOR)
  Serial.print(F("phase lag "));
  Serial.println(phaseLag / CONST_PI * 180.0);
#endif
 
    // calculate tunings
    Kp = (( 2.50 - 0.92 * phaseLag) / (1.0 + (10.75 - 4.01 * phaseLag) * kappa_phi)) * Ku;
    Ti = ((-3.05 + 1.72 * phaseLag) / pow(1.0 + (-6.10 + 3.44 * phaseLag) * kappa_phi, 2)) * Pu;
    Td = 0.0;
    
    // converged
    return true;
  }
#endif // if defined (AUTOTUNE_AMIGOF_PI)    

  Kp = Ku / (double) tuningRule[controlType].divisor(AUTOTUNE_KP_DIVISOR);
  Ti = Pu / (double) tuningRule[controlType].divisor(AUTOTUNE_TI_DIVISOR);
  Td = tuningRule[controlType].PI_controller() ? 
       0.0 : Pu / (double) tuningRule[controlType].divisor(AUTOTUNE_TD_DIVISOR);

  // converged
  return true;
}

#if defined (AUTOTUNE_AMIGOF_PI)
double inline PID::fastArcTan(double x)
{
  // source: “Efficient approximations for the arctangent function”, Rajan, S. Sichun Wang Inkol, R. Joyal, A., May 2006
  //return PID_ATune::CONST_PI / 4.0 * x - x * (abs(x) - 1.0) * (0.2447 + 0.0663 * abs(x));
  
  // source: "Understanding Digital Signal Processing", 2nd Ed, Richard G. Lyons, eq. 13-107
  return x / (1.0 + 0.28125 * pow(x, 2));
}

double PID::calculatePhaseLag(double inducedAmplitude)
{ 
  // calculate phase lag
  // NB hysteresis = 2 * noiseBand;
  double ratio = 2.0 * workingNoiseBand / inducedAmplitude;
  if (ratio > 1.0)
  {
    return CONST_PI_DIV_2;
  }
  else
  {
    //return CONST_PI - asin(ratio);
    return CONST_PI - fastArcTan(ratio / sqrt( 1.0 - pow(ratio, 2)));
  }
}
#endif // if defined (AUTOTUNE_AMIGOF_PI)

#if defined (AUTOTUNE_RELAY_BIAS)
double PID::processValueOffset(double avgStep1, double avgStep2)
{
  // calculate offset of oscillation in process value
  // as a proportion of the amplitude
  // approximation assumes a trapezoidal oscillation 
  // that is stationary over the last 2 relay cycles
  // needs constant phase lag, so recent changes to noiseBand are bad 
      
  if (zero(avgStep1))
  {
    return 1.0;
  }
  if (zero(avgStep2))
  {
    return -1.0;
  }
  // ratio of step durations
  double r1 = avgStep1 / avgStep2;
  
#if defined (AUTOTUNE_DEBUG) || defined (USE_SIMULATOR)
  Serial.print(F("r1 "));
  Serial.println(r1);
#endif

  double s1 = (sumInputSinceLastStep[1] + sumInputSinceLastStep[3]);
  double s2 = (sumInputSinceLastStep[2] + sumInputSinceLastStep[4]);
  if (zero(s1))
  {
    return 1.0;
  }
  if (zero(s2))
  {
    return -1.0;
  }
  // ratio of integrated process values
  double r2 = s1 / s2;

#if defined (AUTOTUNE_DEBUG) || defined (USE_SIMULATOR)
  Serial.print(F("r2 "));
  Serial.println(r2);
#endif

  // estimate process value offset assuming a trapezoidal response curve
  //
  // assume trapezoidal wave with amplitude a, cycle period t, time at minimum/maximum m * t (0 <= m <= 1)
  // 
  // with no offset:
  // area under half wave of process value given by
  //   a * m * t/2 + a/2 * (1 - m) * t/2 = a * (1 + m) * t / 4
  //
  // now with offset d * a (-1 <= d <= 1): 
  // step time of relay half-cycle given by
  //   m * t/2 + (1 - d) * (1 - m) * t/2 = (1 - d + d * m) * t/2
  //
  // => ratio of step times in cycle given by:
  // (1) r1 = (1 - d + d * m) / (1 + d - d * m)
  //
  // area under offset half wave = a * (1 - d) * m * t/2 + a/2 * (1 - d) * (1 - d) * (1 - m) * t/2
  //                             = a * (1 - d) * (1 - d + m * (1 + d)) * t/4 
  //
  // => ratio of area under offset half waves given by:
  // (2) r2 = (1 - d) * (1 - d + m * (1 + d)) / ((1 + d) * (1 + d + m * (1 - d)))
  //
  // want to calculate d as a function of r1, r2; not interested in m
  //
  // rearranging (1) gives:
  // (3) m = 1 - (1 / d) * (1 - r1) / (1 + r1)
  //
  // substitute (3) into (2):
  // r2 = ((1 - d) * (1 - d + 1 + d - (1 + d) / d * (1 - r1) / (1 + r1)) / ((1 + d) * (1 + d + 1 - d - (1 - d) / d * (1 - r1) / (1 + r1)))   
  //
  // after much algebra, we arrive at: 
  // (4) (r1 * r2 + 3 * r1 + 3 * r2 + 1) * d^2 - 2 * (1 + r1)(1 - r2) * d + (1 - r1) * (1 - r2) = 0
  //
  // quadratic solution to (4):
  // (5) d = ((1 + r1) * (1 - r2) +/- 2 * sqrt((1 - r2) * (r1^2 - r2))) / (r1 * r2 + 3 * r1 + 3 * r2 + 1)

  // estimate offset as proportion of amplitude
  double discriminant = (1.0 - r2) * (pow(r1, 2) - r2);
  if (zero(discriminant))
  {
    // catch negative values
    discriminant = 0.0;
  }

  // return estimated process value offset
  return ((1.0 + r1) * (1.0 - r2) + ((r1 > 1.0) ? 1.0 : -1.0) * sqrt(discriminant)) / 
         (r1 * r2 + 3.0 * (r1 + r2 ) + 1.0);
} 
#endif // if defined (AUTOTUNE_RELAY_BIAS)

