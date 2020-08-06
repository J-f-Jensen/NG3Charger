#include <Arduino.h>
#include "SERIALCOMM.h"

/* Init and start CAN */
void SERIALCOMM::begin( int baudrate ) {
    init( baudrate );
}

/* Init and start CAN */
void SERIALCOMM::init( int baudrate ) {

  /* exit if CAN already is active */
  if ( _ComIsActive ) return;
  
  _ComIsActive = true;

  //Initialize hardware
  Serial.begin(baudrate);                   //Starts Serial Communication at Baud Rate 115200
}

SERIALCOMM_T SERIALCOMM::handleSerial()
{
  int timeOut = 0;
  unsigned char  _incomingByte;
  String         _incomingData;
  int            _incomingDataLength;
  SERIALCOMM_T   _output = NoData;
  
  if ( Serial.available() > 0 )
  {     
    _incomingByte = Serial.read(); // read the incoming byte:

    if (_menuload == 1)
    {      
      
      switch (_incomingByte)
      {
        Serial.println();
        
        case 'L': // Find minimum voltage the charger can handle
          Serial.println(F("You typed L - charger will after the beep output lowest possible voltage for 60 secunds"));
          Serial.println(F("LED is blinking while output is active"));
          Serial.println(F("Please measure the output voltage with a digital muiltimeter, and save this value"));
          Serial.println(F("as you need this value for the configuration"));
          _menuload = 0;
          _dataload = 0; // just return
          _output = FindMinVolt;
          break;

         case 'H': // Find maximum voltage the charger can handle
          Serial.println(F("You typed H - charger will after the beep output highest possible voltage for 30 secunds"));
          Serial.println(F("LED is blinking while output is active"));
          Serial.println(F("Please measure the output voltage with a digital muiltimeter, and save this value"));
          Serial.println(F("as you need this value for the configuration"));
          _menuload = 0;
          _dataload = 0; // just return
          _output = FindMaxVolt;
          break;
     
        case 'a': //Set charger minimum voltage
          Serial.println(F("You typed a - Type the value found using the L command"));
          _menuload = 2;
          _dataload = 'a';
          break;

         case 'b': //Set charger maximum voltage
          Serial.println(F("You typed b - Type the value found using the H command"));
          _menuload = 2;
          _dataload = 'b';
          break;

         case 'c': //Set charger mode
          Serial.println(F("You typed c - Type 0 for standalone charger or 1 for CAN controlled"));
          _menuload = 2;
          _dataload = 'c';
          break;

         case 'd': //Set minimum battery charge voltage
          Serial.println(F("You typed d - Type the minimum battery voltage allowed for the charging start"));
          _menuload = 2;
          _dataload = 'd';
          break;

         case 'e': //Set constant current value - Only relevant in standalone mode
          Serial.println(F("You typed e - Type the constant current value - used in the first charge phase"));
          _menuload = 2;
          _dataload = 'e';
          break;

         case 'f': //Set constant voltage value - Only relevant in standalone mode
          Serial.println(F("You typed f - Type the constant voltage value - used in the second charge phase"));
          _menuload = 2;
          _dataload = 'f';
          break;

         case 'g': //Set end charge current - Only relevant in standalone mode
          Serial.println(F("You typed g - Type the end charge current - used in the third charge phase"));
          _menuload = 2;
          _dataload = 'g';
          break;
      
         case 't': //Enable/disable temperature compensation - Only relevant in standalone mode
          Serial.println(F("You typed T - Type 0 to disable or 1 to enable temperature compensation"));
          _menuload = 2;
          _dataload = 'T';
          break;

         case 'h': //Set temp compensation value 1 - Only relevant in standalone mode
          Serial.println(F("You typed h - Type the voltage temp compensation value 1"));
          _menuload = 2;
          _dataload = 'h';
          break;

         case 'i': //Set temp compensation value 2 - Only relevant in standalone mode
          Serial.println(F("You typed i - Type the voltage temp compensation value 2"));
          _menuload = 2;
          _dataload = 'i';
          break;
          
         case 'j': //Set temp compensation value 3 - Only relevant in standalone mode
          Serial.println(F("You typed j - Type the voltage temp compensation value 3"));
          _menuload = 2;
          _dataload = 'j';
          break;         

         case 'k': //Set temp compensation value 1 - Only relevant in standalone mode
          Serial.println(F("You typed k - Type the current temp compensation value 1"));
          _menuload = 2;
          _dataload = 'k';
          break;

         case 'l': //Set temp compensation value 2 - Only relevant in standalone mode
          Serial.println(F("You typed l - Type the current temp compensation value 2"));
          _menuload = 2;
          _dataload = 'l';
          break;
          
         case 'm': //Set temp compensation value 3 - Only relevant in standalone mode
          Serial.println(F("You typed m - Type the current temp compensation value 3"));
          _menuload = 2;
          _dataload = 'm';
          break; 

         case 'n': //Set temp compensation temperature 1 - Only relevant in standalone mode
          Serial.println(F("You typed n - Type the temp compensation temperature 1"));
          _menuload = 2;
          _dataload = 'n';
          break;

         case 'o': //Set temp compensation temperature 2 - Only relevant in standalone mode
          Serial.println(F("You typed o - Type the temp compensation temperature 2"));
          _menuload = 2;
          _dataload = 'o';
          break; 

         case 'p': //Set temp compensation temperature 3 - Only relevant in standalone mode
          Serial.println(F("You typed p - Type the temp compensation temperature 3"));
          _menuload = 2;
          _dataload = 'p';
          break; 

         case 's': //s to save configuration
          Serial.println(F("You typed s - Configuration will be saved to eeprom"));
          _menuload = 0;
          _dataload = 0; // just return
          _output = NewConfig;
          break;
    
        case 'q': //q to go back to main menu
          Serial.println(F("You typed q"));
          _menuload = 0;
          break;

        case 'r': // Set charger offest voltage - differens between internal and external measured output voltage
          Serial.println(F("You typed r - Type the chargers POT offest voltage - Set charger POT offest voltage - differens between measured output voltage and defined volt * 10, can be a negative number exampele -12 for -1.2 volt"));
          _menuload = 2;
          _dataload = 'r';
          break;

        case 'x': // Set charger offest voltage - differens between internal and external measured output voltage
          Serial.println(F("You typed x - Type the chargers ADC offest voltage - differens between internal and external measured output voltage in volt * 10, can be a negative number exampele -12 for -1.2 volt"));
          _menuload = 2;
          _dataload = 'x';
          break;
  
        default:
          // if nothing else matches, do the default
          // default is optional
          break;
      }
    }
    
    if (_menuload == 2)
    {
      while (Serial.available() == 0)
      {
        timeOut++;
        if (timeOut == 1000)
        {
          break;
        }
        delay(50);
      }
      
      if (timeOut == 1000) // We did not recive an input
      {
        Serial.println("");
        Serial.println("No data recived, please try again");
        Serial.println("");

        return NoData;
      }
      else // We did recive an input, check what it is
      {
          _incomingDataLength = Serial.available();

          _incomingData = Serial.readString();
               
          if (_incomingData == 'q' && _incomingDataLength == 1 ) // User wanted to quit, go back to start menu
          {
            _menuload = 0;
            _incomingByte = 'S';
          }
          else // We recived data
          { 
            if ( _incomingDataLength > 4 )
            {
              Serial.println("");
              Serial.println(F("More than 4 chars recived - this is not allowed, please retry"));
              Serial.println("");
              _menuload = 0;
              return NoData;
            }
            
            Serial.println(F("You typed: "));
            Serial.print(_incomingData);
            Serial.println(F(" - remeber to save configuration by typing uppercase S followed by a lovercase s "));
            
            switch (_dataload)
            {
              case 'a': // Configure charger minimum voltage
                _menuload = 0;
                settings.CHARGER_MIN_VOLTS = (unsigned short) (_incomingData.toInt());
                break;

              case 'b': // Configure charger maximum voltage
                _menuload = 0;
                settings.CHARGER_MAX_VOLTS = (unsigned short)(_incomingData.toInt());  
                break;

              case 'c': // Set mode - Standalone or CAN controlled
                _menuload = 0;
                settings.CAN_ENABLED  = (_incomingData == '1') ? false : true;            
                break;

              case 'd': // Set minimum battery charge voltage
                _menuload = 0;
                settings.CHARGE_MINIMUM_START_VOLTAGE = (unsigned short)(_incomingData.toInt());
                break;

              case 'e': // Set constant current value - Only relevant in standalone mode
                _menuload = 0;
                settings.CHARGE_CONSTANT_CURRENT = (unsigned short)(_incomingData.toInt());
                break;

              case 'f': // Set constant voltage value - Only relevant in standalone mode
                _menuload = 0;
                settings.CHARGE_CONSTANT_VOLTAGE = (unsigned short)(_incomingData.toInt());
                break;

              case 'g': // Set end charge current - Only relevant in standalone mode
                _menuload = 0;
                settings.CHARGE_END_CURRENT = (unsigned short)(_incomingData.toInt());
                break;

              case 't': //Enable/disable temperature compensation - Only relevant in standalone mode
               _menuload = 0;
               settings.TEMP_COMPENSATED = (_incomingData == '1') ? false : true;
               break;

              case 'h': // Set voltage temp compensation value 1 - Only relevant in standalone mode
                _menuload = 0;
                settings.CHARGE_V_TEMP_COMPENSATION_VALUE1 = (signed short)(_incomingData.toInt());
                break;

              case 'i': // Set voltage temp compensation value 2 - Only relevant in standalone mode
                _menuload = 0;
                settings.CHARGE_V_TEMP_COMPENSATION_VALUE2 = (signed short)(_incomingData.toInt());
                break;

              case 'j': // Set voltage temp compensation value 3 - Only relevant in standalone mode"
                _menuload = 0;
                settings.CHARGE_V_TEMP_COMPENSATION_VALUE3 = (signed short)(_incomingData.toInt());
                break;

              case 'k': // Set current temp compensation value 1 - Only relevant in standalone mode
                _menuload = 0;
                settings.CHARGE_C_TEMP_COMPENSATION_VALUE1 = (signed short)(_incomingData.toInt());
                break;

              case 'l': // Set current temp compensation value 2 - Only relevant in standalone mode
                _menuload = 0;
                settings.CHARGE_C_TEMP_COMPENSATION_VALUE2 = (signed short)(_incomingData.toInt());
                break;

              case 'm': // Set current temp compensation value 3 - Only relevant in standalone mode"
                _menuload = 0;
                settings.CHARGE_C_TEMP_COMPENSATION_VALUE3 = (signed short)(_incomingData.toInt());
                break;

              case 'n': // Set temp compensation temperature 1 - Only relevant in standalone mode
                _menuload = 0;
                settings.CHARGE_TEMP_COMPENSATION_TEMPERATURE1 = (signed char)(_incomingData.toInt());
                break;

              case 'o': // Set temp compensation temperature 2 - Only relevant in standalone mode
                _menuload = 0;
                settings.CHARGE_TEMP_COMPENSATION_TEMPERATURE2 = (signed char)(_incomingData.toInt());
                break;

              case 'p': // Set temp compensation temperature 3 - Only relevant in standalone mode"
                _menuload = 0;
                settings.CHARGE_TEMP_COMPENSATION_TEMPERATURE3 = (signed char)(_incomingData.toInt());
                break;

             case 'r': // Set charger POT offest voltage - differens between measured output voltage and defined voltage
                _menuload = 0;
                settings.CHARGER_VOLTAGE_OFFSET_POT = (signed char)(_incomingData.toInt());
                break;

             case 'x': // Set charger ADC offest voltage - differens between internal and external measured output voltage
                _menuload = 0;
                settings.CHARGER_VOLTAGE_OFFSET_ADC = (signed char)(_incomingData.toInt());
                break;

              default:
                // if nothing else matches, do the default
                // default is optional
                break;
            }
          }
        }
      }
  
    if (_incomingByte == 'S' & _menuload == 0)
    {
      Serial.println();
      Serial.println(F("Welcome to the charger setup menu"));
      Serial.println(F("Before you start using the Zivian NG charger with the new board, you must run a configuration cycle"));
      Serial.println(F("and define if it should run as a standalone or CAN bus controlled charger."));
      Serial.println(F("Values should be in the following format"));
      Serial.println(F("Messaured value round of to 1 decimal place, multiplied by 10"));
      Serial.println(F("e.g. if you measured 86.51 volt you should type 865 followed by enter/send"));
      Serial.println();
      Serial.println(F("You have the following options, first time you should start with L and H to find the min and max voltage"));
      Serial.println(F("L - Find charger minimum output voltage"));
      Serial.println(F("H - Find charger maximum output voltage"));     
      Serial.println(F("a - Set charger minimum voltage"));
      Serial.println(F("b - Set charger maximum voltage"));
      Serial.println(F("r - Set charger POT offest voltage - differens between measured output voltage and defined voltage"));
      Serial.println(F("x - Set charger ADC offest voltage - differens between intern and external measured output voltage"));
      Serial.println(F("c - Set mode - Standalone or CAN controlled"));
      Serial.println(F("d - Set minimum battery charge voltage - Only relevant in standalone mode"));
      Serial.println(F("e - Set constant current value - Only relevant in standalone mode"));
      Serial.println(F("f - Set constant voltage value - Only relevant in standalone mode"));
      Serial.println(F("g - Set end charge current - Only relevant in standalone mode"));
      /* - not implemented
      Serial.println(F("t - Enable/disable temperature compensation - Only relevant in standalone mode"));
      Serial.println(F("h - Set voltage temp compensation value 1 - Only relevant in standalone mode"));
      Serial.println(F("i - Set voltage temp compensation value 2 - Only relevant in standalone mode"));
      Serial.println(F("j - Set voltage temp compensation value 3 - Only relevant in standalone mode"));
      Serial.println(F("k - Set current temp compensation value 1 - Only relevant in standalone mode"));
      Serial.println(F("l - Set current temp compensation value 2 - Only relevant in standalone mode"));
      Serial.println(F("m - Set current temp compensation value 3 - Only relevant in standalone mode"));      
      Serial.println(F("n - Set temp compensation temperature 1 - Only relevant in standalone mode"));
      Serial.println(F("o - Set temp compensation temperature 2 - Only relevant in standalone mode"));
      Serial.println(F("p - Set temp compensation temperature 3 - Only relevant in standalone mode"));
      */
      Serial.println(F("s - Save configuration to EEPROM"));
      Serial.println(F("q - exit menu"));
      _menuload = 1;
    }

    // Show current measured values
    if (_incomingByte == '?' & _menuload == 0)
    {
      Serial.println();
      Serial.println(F("Values measured by charger based on current configuration"));
      Serial.print(F("Voltage: "));
      Serial.print((float)GlobalData.Voltage/10);
      Serial.println(F(" Volt"));
      Serial.print(F("Current: "));
      Serial.print((float)GlobalData.Current/10);
      Serial.println(F(" Ampere"));
      Serial.print(F("Raw measured current AD bits: "));
      Serial.println(GlobalData.RawCurrent);
      Serial.print(F("Extern temerature: "));
      Serial.print(GlobalData.ExternalTemperature);
      Serial.println(F(" degree Celtius"));
      Serial.print(F("Extern measured temerature AD bits: "));
      Serial.print(GlobalData.RawTemperature);
      Serial.println(F(" data (char)"));
      Serial.println();
      Serial.println("Type uppercase S for configuration menu");
      Serial.println("Type uppercase C to see configuration values");
      Serial.println("Type uppercase T to run charger in test mode (charger try to output constant voltage value) - only relevant when in standalon charger mode");
    }

    if (_incomingByte == 'C' & _menuload == 0) // Send configuration values
    {
      Serial.print(F("VERSION: "));
      Serial.println(settings.VERSION);
      Serial.print(F("CONFIGURED: "));
      Serial.println(settings.CONFIGURED);
      Serial.print(F("CAN_ENABLED: "));
      Serial.println(settings.CAN_ENABLED);
      Serial.print(F("CAN_SPEED: "));
      Serial.println(settings.CAN_SPEED);
      Serial.print(F("CHARGER_MIN_VOLTS: "));
      Serial.println(settings.CHARGER_MIN_VOLTS);
      Serial.print(F("CHARGER_MAX_VOLTS: "));
      Serial.println(settings.CHARGER_MAX_VOLTS);
      Serial.print(F("CHARGER_VOLTAGE_AD8402_BITS: "));
      Serial.println(settings.CHARGER_VOLTAGE_AD8402_BITS);
      Serial.print(F("CHARGER_VOLTAGE_AD_BITS: "));
      Serial.println(settings.CHARGER_VOLTAGE_AD_BITS);
      Serial.print(F("CHARGER_CURRENT_AD8402_mA_PER_BIT: "));
      Serial.println(settings.CHARGER_CURRENT_AD8402_mA_PER_BIT);
      Serial.print(F("CHARGER_CURRENT_AD_mA_PER_BIT: "));
      Serial.println(settings.CHARGER_CURRENT_AD_mA_PER_BIT);
      Serial.print(F("CHARGER_MAX_POWER_10A: "));
      Serial.println(settings.CHARGER_MAX_POWER_10A);
      Serial.print(F("CHARGER_MAX_POWER_16A: "));
      Serial.println(settings.CHARGER_MAX_POWER_16A);
      Serial.print(F("CHARGER_VOLTAGE_OFFSET_POT: "));
      Serial.println(settings.CHARGER_VOLTAGE_OFFSET_POT);
      Serial.print(F("CHARGER_VOLTAGE_OFFSET_ADC: "));
      Serial.println(settings.CHARGER_VOLTAGE_OFFSET_ADC);
      Serial.print(F("CHARGE_MINIMUM_START_VOLTAGE: "));
      Serial.println(settings.CHARGE_MINIMUM_START_VOLTAGE);
      Serial.print(F("CHARGE_CONSTANT_VOLTAGE: "));
      Serial.println(settings.CHARGE_CONSTANT_VOLTAGE);
      Serial.print(F("CHARGE_CONSTANT_CURRENT: "));
      Serial.println(settings.CHARGE_CONSTANT_CURRENT);
      Serial.print(F("CHARGE_END_CURRENT: "));
      Serial.println(settings.CHARGE_END_CURRENT);
      Serial.print(F("TEMP_COMPENSATED: "));
      Serial.println(settings.TEMP_COMPENSATED);
      Serial.print(F("CHARGE_V_TEMP_COMPENSATION_VALUE1: "));
      Serial.println(settings.CHARGE_V_TEMP_COMPENSATION_VALUE1);
      Serial.print(F("CHARGE_V_TEMP_COMPENSATION_VALUE2: "));
      Serial.println(settings.CHARGE_V_TEMP_COMPENSATION_VALUE2);
      Serial.print(F("CHARGE_V_TEMP_COMPENSATION_VALUE3: "));
      Serial.println(settings.CHARGE_V_TEMP_COMPENSATION_VALUE3);
      Serial.print(F("CHARGE_C_TEMP_COMPENSATION_VALUE1: "));
      Serial.println(settings.CHARGE_C_TEMP_COMPENSATION_VALUE1);
      Serial.print(F("CHARGE_C_TEMP_COMPENSATION_VALUE2: "));
      Serial.println(settings.CHARGE_C_TEMP_COMPENSATION_VALUE2);
      Serial.print(F("CHARGE_C_TEMP_COMPENSATION_VALUE3: "));
      Serial.println(settings.CHARGE_C_TEMP_COMPENSATION_VALUE3);
      Serial.print(F("CHARGE_TEMP_COMPENSATION_TEMPERATURE1: "));
      Serial.println(settings.CHARGE_TEMP_COMPENSATION_TEMPERATURE1);
      Serial.print(F("CHARGE_TEMP_COMPENSATION_TEMPERATURE2: "));
      Serial.println(settings.CHARGE_TEMP_COMPENSATION_TEMPERATURE2);
      Serial.print(F("CHARGE_TEMP_COMPENSATION_TEMPERATURE3: "));
      Serial.println(settings.CHARGE_TEMP_COMPENSATION_TEMPERATURE3);
      Serial.println(F(""));
    }

    if (_incomingByte == 'T' & _menuload == 0) // Send configuration values
    {
      Serial.println(F(""));
      Serial.println(F("Charger change to test mode - you should not run this with batteries connected"));
      Serial.println(F("Charger switch to idle after 60 sec."));
      Serial.println(F(""));
      _output = TestMode;
    }
  }
  
  return _output;
} 
