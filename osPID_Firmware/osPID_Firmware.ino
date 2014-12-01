/*******************************************************************************
 * The stripboard PID Arduino shield uses firmware based on the osPID but
 * simplified hardware. Instead of swappable output cards there is a simple
 * 5V logical output that can drive an SSR. In place of output cards there
 * is a terminal block that can be used for a 1-wire bus or NTC thermistor,
 * and female pin headers that can interface a MAX31855 thermocouple amplifier
 * breakout board. Each of these inputs is supported by different
 * device drivers & libraries. The input in use is specified by a menu option.
 * This saves having to recompile the firmware when changing input sensors.
 *
 * Inputs
 * ======
 *    DS18B20+ 1-wire digital thermometer with data pin on A0, OR
 *    10K NTC thermistor with voltage divider input on pin A0, OR
 *    MAX31855KASA interface to type-K thermocouple on pins A0-A2.
 *
 * Output
 * ======
 *    1 SSR output on pin A3.
 *
 * For firmware development, there is also the ospSimulator which acts as both
 * the input and output device and simulates the controller being attached to a
 * simple system.
 *******************************************************************************/
 
 /* Thoughts looking ahead to having multiple PID controller instances.
  * 
  * Really there should be a PID_engine class and a PID_controller superclass
  *
  * the engine class is mainly (solely) concerned with calculating the output
  * 
  * profile behaviour should be part of the controller class because it 
  * manipulates the engine
  *
  * auto tuning should be a method of the engine class, in my opinion
  * i think the reason it is not currently set up this way is that 
  * configuring the method needs to happen offline in the controller
  * not a good enough reason in my view
  *
  * alarm behaviour would be easy to integrate into the engine but might 
  * make more sense as part of the controller
  *
  * I/O obviously should be controller methods, the alarm is arguably I/O
  * the trip limits are based on sensor input which is extrinsic to the engine
  * 
  * and obviously the whole millis() based timer polling loop has to go
  */
 
 /*******************************************************************************
 *
 *
 *                          INCLUDES  &  DEFINES
 *
 *
 *******************************************************************************/

#include <Arduino.h>
#include <LiquidCrystal.h>
#include <avr/pgmspace.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "ospConfig.h"
#include "ospDecimalValue.h"
#include "osPID_Engine.h"
#include "ospAnalogButton.h"
#include "ospProfile.h"

#undef BUGCHECK
#define BUGCHECK() ospBugCheck(PSTR("MAIN"), __LINE__);

#if !defined USE_SIMULATOR
#include "ospOutputDeviceSsr.h"
#include "ospInputDevice.h"
ospInputDevice theInputDevice;
ospOutputDeviceSsr theOutputDevice;
#else
#include "ospSimulator.h"
ospSimulator theInputDevice;
#define theOutputDevice theInputDevice
#endif

#define MINIMUM(a,b) (((a)<(b))?(a):(b))
#define MAXIMUM(a,b) (((b)>(a))?(a):(b))

 /*******************************************************************************
 *
 *
 *                          PROTOTYPE DEFINITIONS
 *
 *
 *******************************************************************************/

extern void __attribute__ ((noinline)) LCDprintln(const char* s);
extern void drawStartupBanner();
extern void setupEEPROM();
extern void setupSerial();
extern bool profileWasInterrupted();
extern void drawResumeProfileBanner();
extern bool startCurrentProfileStep();
extern void recordProfileCompletion();
extern bool okKeyLongPress();
extern void backKeyPress();
extern void updownKeyPress(bool);
extern void okKeyPress();
extern void markSettingsDirty();
extern void profileLoopIteration();
extern void drawMenu();
extern void saveEEPROMSettings();
extern void processSerialCommand();

 /*******************************************************************************
 *
 *
 *                        GLOBAL VARIABLE INITIALIZATION
 *
 *
 *******************************************************************************/

// static constant variables
// power-on options
enum 
{
  POWERON_DISABLE = 0,
  POWERON_CONTINUE_LOOP,
  POWERON_RESUME_PROFILE
};

// we use the LiquidCrystal library to drive the LCD screen
LiquidCrystal theLCD(lcdRsPin, lcdEnablePin, lcdD0Pin, lcdD1Pin, lcdD2Pin, lcdD3Pin);

// our AnalogButton library provides debouncing and interpretation
// of the analog-multiplexed button channel
ospAnalogButton<buttonsPin, 100, 253, 454, 657> theButtonReader;

// an in-memory buffer that we use when receiving a profile over USB
ospProfile profileBuffer;

// the 0-based index of the active profile while a profile is executing
byte activeProfileIndex;
byte currentProfileStep;
boolean runningProfile = false;

// the gain coefficients of the PID controller
ospDecimalValue<3> PGain = { 1600 }, IGain = { 200 }, DGain = { 0 };

// the 4 setpoints we can easily switch between
#if !defined (UNITS_FAHRENHEIT)
ospDecimalValue<1> setPoints[4] = { { 250 }, { 650 }, { 1000 }, { 1250 } };
#else
ospDecimalValue<1> setPoints[4] = { { 800 }, { 1500 }, { 2120 }, { 2600 } };
#endif

// the index of the selected setpoint
byte setPointIndex = 0;

// set value for PID controller
double activeSetPoint;

// the most recent measured input value
double input = NAN; 

// last good input value, used by PID controller
double lastGoodInput = 25.0;

// the output duty cycle calculated by PID controller
double output = 0.0;   

// the manually-commanded output value
ospDecimalValue<1> manualOutput = { 0 };

// temporary fixed point decimal values for display and data entry
ospDecimalValue<1> displaySetpoint    = { 250 };
ospDecimalValue<1> displayInput       = { -19999 }; // NaN
ospDecimalValue<1> displayCalibration = { 0 }; 
ospDecimalValue<1> displayWindow      = { 50 }; 

// the hard trip limits
#if !defined (UNITS_FAHRENHEIT)
ospDecimalValue<1> lowerTripLimit =   { 0 } , upperTripLimit = { 1250 };
#else
ospDecimalValue<1> lowerTripLimit = { 320 } , upperTripLimit = { 2600 };
#endif

bool tripLimitsEnabled;
bool tripped;
bool tripAutoReset;

byte powerOnBehavior = DEFAULT_POWER_ON_BEHAVIOR;

bool controllerIsBooting = true;

// the actual PID controller
PID myPID(&lastGoodInput, &output, &activeSetPoint, PGain, IGain, DGain, PID::DIRECT);

// PID auto tune parameters
byte aTuneMethod = PID::AUTOTUNE_DEFAULT_METHOD; 
ospDecimalValue<1> aTuneStep  = (ospDecimalValue<1>){PID::AUTOTUNE_DEFAULT_OUTPUT_STEP};
int aTuneLookBack             = PID::AUTOTUNE_DEFAULT_LOOKBACK_SEC;

#if !defined (UNITS_FAHRENHEIT)
ospDecimalValue<3> aTuneNoise = makeDecimal<3>(PID::AUTOTUNE_DEFAULT_NOISE_BAND_CELSIUS);
#else
ospDecimalValue<3> aTuneNoise = makeDecimal<3>(PID::AUTOTUNE_DEFAULT_NOISE_BAND_CELSIUS * 1.8);
#endif 

// timekeeping to schedule the various tasks in the main loop
unsigned long now, lcdTime, readInputTime;

extern void drawNotificationCursor(char icon);

// some constants in flash memory, for reuse
#if !defined (UNITS_FAHRENHEIT)
const __FlashStringHelper *FdegCelsius() { return F(" °C"); }
#else
const __FlashStringHelper *FdegFahrenheit() { return F(" °F"); }
#endif

PROGMEM const char Pprofile[] = "Profile ";

 /*******************************************************************************
 *
 *
 *                     INTERRUPT SERVICE ROUTINE FOR BUZZER
 *
 *
 *******************************************************************************/

#if !defined (SILENCE_BUZZER)
// buzzer 
volatile int buzz = 0; // countdown timer for intermittent buzzer
enum { BUZZ_OFF = 0, BUZZ_UNTIL_CANCEL = -769 };
#define buzzMillis(x)    buzz = (x)*4
#define buzzUntilCancel  buzz = BUZZ_UNTIL_CANCEL
#define buzzOff          buzz = BUZZ_OFF

ISR (TIMER2_COMPA_vect)
{
  const byte     buzzerPort     = digitalPinToPort(buzzerPin);
  volatile byte *buzzerOut      = portOutputRegister(buzzerPort);
  const byte     buzzerBitMask  = digitalPinToBitMask(buzzerPin);

  // buzz counts down 
  if (buzz == BUZZ_OFF)
  {
    // write buzzer pin low
    *buzzerOut &= ~buzzerBitMask;
  }
  else
  {      
    if (((unsigned int)buzz & 0x3F0) == 0) // intermittent chirp
    {
      // toggle buzzer pin to make tone
      *buzzerOut ^= buzzerBitMask;
    }
    else
    {
      // write buzzer pin low
      *buzzerOut &= ~buzzerBitMask;
    }
    buzz--;
    if (buzz == BUZZ_UNTIL_CANCEL - 1024)
    {
      buzz = BUZZ_UNTIL_CANCEL;
    }
  }
}
#endif

 /*******************************************************************************
 *
 *
 *                         FUNCTION  INITIALIZATION
 *
 *
 *******************************************************************************/

char hex(byte b)
{
  return ((b < 10) ? (char) ('0' + b) : (char) ('A' - 10 + b));
}

void __attribute__ ((noinline)) updateTimer()
{
  now = millis();
}

// check time avoiding overflow
bool __attribute__ ((noinline)) after(unsigned long targetTime)
{
  unsigned long u = (targetTime - now);
  return ((u & 0x80000000) > 0);
}

void __attribute__((noinline)) setOutputToManualOutput()
{
  output = double(manualOutput);
}

void __attribute__((noinline)) updateActiveSetPoint()
{
  displaySetpoint = setPoints[setPointIndex];
  activeSetPoint = double(displaySetpoint);
}

 /*******************************************************************************
 *
 *
 *                        CONTROLLER  INITIALIZATION
 *
 *
 *******************************************************************************/

// initialize the controller: this is called by the Arduino runtime on bootup
void setup()
{
  // initialize pins
  for (byte pin = 8; pin < 13; pin++)
  {
    pinMode(pin, INPUT_PULLUP);
  }
  pinMode(buzzerPin, OUTPUT);
  
  // set up timer2 for buzzer interrupt
#if !defined (SILENCE_BUZZER)
  cli();                   // disable interrupts
  #if defined (__AVR_ATmega328P__)
  OCR2A = 249;             // set up timer2 CTC interrupts for buzzer
  TCCR2A |= (1 << WGM21);  // CTC Mode
  TIMSK2 |= (1 << OCIE2A); // set interrupt on compare match
  GTCCR  |= (1 << PSRASY); // reset timer2 prescaler
  TCCR2B |= (7 << CS20);   // prescaler 1024
  #elif defined (__AVR_ATmega32U4__)
  OCR3AH = 0;              // set up timer3 CTC interrupts for buzzer
  OCR3AL = 249;            // set up timer3 CTC interrupts for buzzer
  TCCR3A |= (1 << WGM32);  // CTC Mode
  TIMSK3 |= (1 << OCIE3A); // set interrupt on compare match
  GTCCR  |= (1 << PSRASY); // reset timer3 prescaler
  TCCR3B |= (5 << CS30);   // prescaler 1024
  #endif
  sei();                   // enable interrupts
#endif  

  // set up the LCD,show controller name
  theLCD.begin(16, 2);
   //Color swirl on bootup
  pinMode(lcdREDPin, OUTPUT);
  pinMode(lcdGRNPin, OUTPUT);
  pinMode(lcdBLUPin, OUTPUT);
  
  brightness = 200;
  
//Code to correct the red color from overpowering the other RGB colors  
  //start red to green
  for (int i = 0; i < 255; i++) {
    setBacklight(i, 0, 255-i);
    delay(2);
  }
  //fade red to green
  for (int i = 0; i < 255; i++) {
    setBacklight(255-i, i, 0);
    delay(2);
  }
  //fade green to blue
  for (int i = 0; i < 255; i++) {
    setBacklight(0, 255-i, i);
    delay(2);
  }
  setBacklight(0,255,0);
  drawStartupBanner();

  lcdTime = 25;
  updateTimer();

  // load the EEPROM settings
  setupEEPROM();
  
  // set up the peripheral devices
  theInputDevice.initialize();
  theOutputDevice.initialize();

  // set up the serial interface
#if !defined (STANDALONE_CONTROLLER)
  setupSerial();
#endif

  long int pause = now + 2000 - millis();
  if (pause < 10)
  {
    pause = 10;
  }
  delay(pause);
  updateTimer();

  // configure the PID loop 
  updateActiveSetPoint();
  myPID.setSampleTime(PID::DEFAULT_LOOP_SAMPLE_TIME);
  myPID.setOutputLimits(0, 100);
  myPID.setTunings(PGain, IGain, DGain);

  if (powerOnBehavior == POWERON_DISABLE) 
  {
    myPID.setMode(PID::MANUAL);
    setOutputToManualOutput();
  }

  // finally, check whether we were interrupted in the middle of a profile
  if (profileWasInterrupted())
  {
    if (powerOnBehavior == POWERON_RESUME_PROFILE)
    {
      drawResumeProfileBanner();
      runningProfile = true;
      startCurrentProfileStep();
    }
    else
    {
      recordProfileCompletion(); // we don't want to pick up again, so mark it completed
    }
  }

  // kick things off by requesting sensor input
  updateTimer();
  if (theInputDevice.getInitializationStatus())
  {
    readInputTime = now + theInputDevice.requestInput();
  }

  controllerIsBooting = false;
}

 /*******************************************************************************
 *
 *
 *                           BUTTON  ROUTINES
 *
 *
 *******************************************************************************/

// Letting a button auto-repeat without redrawing the LCD in between leads to a
// poor user interface
bool lcdRedrawNeeded;

// keep track of which button is being held, and for how long
byte heldButton;
byte autoRepeatCount;
unsigned long autoRepeatTriggerTime;

// test the buttons and look for button presses or long-presses
static void checkButtons()
{
  byte button = theButtonReader.get();
  byte executeButton = BUTTON_NONE;

  if (button != BUTTON_NONE)
  {
    if (heldButton == BUTTON_NONE)
    {
      autoRepeatTriggerTime = now + AUTOREPEAT_DELAY;
    }
    else if (heldButton == BUTTON_OK)
    {
      // OK does long-press/short-press, not auto-repeat
    }
    else if (after(autoRepeatTriggerTime))
    {
      // don't auto-repeat until 100 ms after the redraw
      if (lcdRedrawNeeded)
      {
        autoRepeatTriggerTime = now + 150;
        return;
      }

      // auto-repeat
      executeButton = button;
      autoRepeatCount += 1;
      autoRepeatTriggerTime = now + AUTOREPEAT_PERIOD;
    }
    heldButton = button;
  }
  else if (heldButton != BUTTON_NONE)
  {
    if (heldButton == BUTTON_OK && (after(autoRepeatTriggerTime + 400 - AUTOREPEAT_DELAY)))
    {
      // BUTTON_OK was held for at least 400 ms: execute a long-press
      bool longPress = okKeyLongPress();

      if (!longPress)
      {
        // no long-press action defined, so fall back to a short-press
        executeButton = BUTTON_OK;
      }
    }
    else if (autoRepeatCount == 0)
    {
      // the button hasn't triggered auto-repeat yet; execute it on release
      executeButton = heldButton;
    }

    // else: the button was in auto-repeat, so don't execute it again on release
    heldButton = BUTTON_NONE;
    autoRepeatCount = 0;
  }

  if (executeButton == BUTTON_NONE)
    return;

  lcdRedrawNeeded = true;

  switch (executeButton)
  {
  case BUTTON_RETURN:
    backKeyPress();
    break;

  case BUTTON_UP:
  case BUTTON_DOWN:
    updownKeyPress(executeButton == BUTTON_UP);
    break;

  case BUTTON_OK:
    okKeyPress();
    break;
  }
}

 /*******************************************************************************
 *
 *
 *                                MAIN   LOOP
 *
 *
 *******************************************************************************/

// This is the Arduino main loop.
//
// There are two goals this loop must balance: the highest priority is
// that the PID loop be executed reliably and on-time; the other goal is that
// the screen, buttons, and serial interfaces all be responsive. However,
// since things like redrawing the LCD may take tens of milliseconds -- and responding
// to serial commands can take 100s of milliseconds at low bit rates -- a certain
// measure of cleverness is required.
//
// Alongside the real-time task of the PID loop, there are 6 other tasks which may
// need to be performed:
// 1. handling a button press
// 2. executing a step of the auto-tuner
// 3. executing a step of a profile
// 4. redrawing the LCD
// 5. saving settings to EEPROM
// 6. processing a serial-port command
//
// Characters from the serial port are received asynchronously: it is only the
// command _processing_ which needs to be scheduled.

bool settingsWritebackNeeded;
unsigned long settingsWritebackTime;

// record that the settings have changed, and need to be written to EEPROM
// as soon as they are done changing
void markSettingsDirty()
{
  settingsWritebackNeeded = true;

  // wait until nothing has changed for 5s before writing to EEPROM
  // this reduces EEPROM wear by not writing every time a digit is changed
  settingsWritebackTime = now + 5000;
}

// whether loop() is permitted to do LCD, EEPROM, or serial I/O: this is set
// to false when loop() is being re-entered during some slow operation
bool blockSlowOperations;

void realtimeLoop()
{
  if (controllerIsBooting)
  {
    return;
  }

  blockSlowOperations = true;
  loop();
  blockSlowOperations = false;
}

#if !defined (STANDALONE_CONTROLLER)
// we accumulate characters for a single serial command in this buffer
char serialCommandBuffer[33];
byte serialCommandLength;
#endif

void loop()
{
  // first up is the realtime part of the loop, which is not allowed to perform
  // EEPROM writes or serial I/O
  updateTimer();

  // highest priority task is to update the SSR output
  theOutputDevice.setOutputPercent(output);

  // read input, if it is ready
  if (theInputDevice.getInitializationStatus() && after(readInputTime))
  {
    input = theInputDevice.readInput();
    if (!isnan(input))
    {
      lastGoodInput = input;
      displayInput = makeDecimal<1>(input);
    }
    else
    {
      displayInput = (ospDecimalValue<1>){-19999}; // display Err
    }
    readInputTime += theInputDevice.requestInput();
  }
  
  // step the profile, if there is one running
  // this may call ospSettingsHelper::eepromClearBits(), but not
  // ospSettingsHelper::eepromWrite()
  if (runningProfile)
  {
    profileLoopIteration();
      
    // update displayed set point
    updateActiveSetPoint();
  }

  // update the PID
  myPID.compute();  
  
  // update the displayed output
  // unless in manual mode, in which case a new value may have been entered
  if (myPID.isTuning || (myPID.getMode() != PID::MANUAL))
  {
    manualOutput = makeDecimal<1>(output);
  }

  // after the PID has updated, check the trip limits
  if (tripLimitsEnabled)
  {
    if (tripAutoReset)
    {
      tripped = false; 
    }

    if (
      (displayInput != (ospDecimalValue<1>){-19999}) && 
      ((displayInput < lowerTripLimit) || (upperTripLimit < displayInput ) || tripped)
    )
    {
      output = 0.0;
      manualOutput = (ospDecimalValue<1>){0};
      tripped = true;
      
#if !defined (SILENCE_BUZZER)  
      if (buzz >= BUZZ_OFF)
      {
        buzzUntilCancel; // could get pretty annoying
      }
#endif      

    }

#if !defined (SILENCE_BUZZER)      
    if (!tripped)
    {
      buzzOff;
    }
#endif      
  }   
  else
  {
    tripped = false;
    
#if !defined (SILENCE_BUZZER) 
    buzzOff;
#endif    

  }  

  // after the realtime part comes the slow operations, which may re-enter
  // the realtime part of the loop but not the slow part
  if (blockSlowOperations)
    return;

  // update the time after each major operation;
  updateTimer();

  // we want to monitor the buttons as often as possible
  checkButtons();
  
  // we try to keep an LCD frame rate of 4 Hz, plus refreshing as soon as
  // a button is pressed
  updateTimer();
  if (after(lcdTime) || lcdRedrawNeeded)
  {
    drawMenu();
    lcdRedrawNeeded = false;
    lcdTime += 250;
  }

  // can't do much without input, so initializing input is next in line 
  if (!theInputDevice.getInitializationStatus())
  {
    input = NAN;
    displayInput = (ospDecimalValue<1>){-19999}; // Display Err
    theInputDevice.initialize();
  }     

  updateTimer();
  if (settingsWritebackNeeded && after(settingsWritebackTime))
  {
    // clear settingsWritebackNeeded first, so that it gets re-armed if the
    // realtime loop calls markSettingsDirty()
    settingsWritebackNeeded = false;

    // display a '$' instead of the cursor to show that we're saving to EEPROM
    drawNotificationCursor('$');
    saveEEPROMSettings();
  }

#if !defined (STANDALONE_CONTROLLER)
  // accept any pending characters from the serial buffer
  byte avail = Serial.available();
  while (avail--)
  {
    char ch = Serial.read();  
    
    if (
      (serialCommandLength == 0) &&
      ((ch == '\n') || (ch == '\r') || (ch == '\0'))
    )
      continue;
      
    if (serialCommandLength < 32)
    {
      // throw away excess characters
      serialCommandBuffer[serialCommandLength++] = ch;
    }

    if ((ch == '\n') || (ch == '\r'))
    {
      // a complete command has been received
      serialCommandBuffer[serialCommandLength - 1] = '\n';
      drawNotificationCursor('*');

      /*
      // debug
      Serial.print(F("Arduino hears:"));
      Serial.write('"');
      Serial.print(serialCommandBuffer);
      Serial.write('"');
      Serial.println();
      */

      processSerialCommand();
      
      serialCommandLength = 0;
    }
  }
#endif

}


