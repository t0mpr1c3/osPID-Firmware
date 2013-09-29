#ifndef OSPTEMPERATUREINPUTCARD_H
#define OSPTEMPERATUREINPUTCARD_H

#include "ospCards.h"
#include "ospSettingsHelper.h"

class ospTemperatureInputCard : 
  public ospBaseInputCard 
{
private:

  bool initialized;
  double calibration;


public:
  ospTemperatureInputCard() :
    ospBaseInputCard(),
    initialized(false),
    calibration(0.0f)
  { 
  }
  
  // get initialization status
  bool initialized()
  {
    return initialized;
  }

  // set initialization status
  void setInitialized(bool newInitialized)
  {
    initialized = newInitialized;
  }

  // get calibration
  double calibration()
  {
    return calibration;
  }

  // set calibration
  void setCalibration(double newCalibration)
  {
    calibration = newCalibration;
  }

  const __FlashStringHelper *cardIdentifier() { return 0; };
  double readInput() { return NAN; }; 
  unsigned long requestInput() { return 0; }; // returns conversion time in milliseconds 
  byte floatSettingsCount() { return 0; }; 
  //byte integerSettingsCount() { return 0; }; 
  double readFloatSetting(byte index) { return -1.0f; };
  //int readIntegerSetting(byte index) { return -1; };
  bool writeFloatSetting(byte index, double val) { return false; }; 
  //bool writeIntegerSetting(byte index, int val) { return false; }; 
  const __FlashStringHelper *describeFloatSetting(byte index) { return 0; }; 
  //const __FlashStringHelper *describeIntegerSetting(byte index) { return 0; }; 
  void saveSettings(ospSettingsHelper& settings) {}; 
  void restoreSettings(ospSettingsHelper& settings) {}; 
};


#endif






