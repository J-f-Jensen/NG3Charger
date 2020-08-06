#ifndef _CONFIG_H_
#define _CONFIG_H_

#define EEPROM_VERSION      0x00    //update any time EEPROM struct below is changed or hardcoded default values are changed.

// Eeprom data structure
typedef struct {
  unsigned char   VERSION;
  bool            CONFIGURED;
  bool            CAN_ENABLED;
  unsigned char   CAN_SPEED;
  unsigned short  CHARGER_MIN_VOLTS;
  unsigned short  CHARGER_MAX_VOLTS;
  unsigned char   CHARGER_VOLTAGE_AD8402_BITS;
  unsigned short  CHARGER_VOLTAGE_AD_BITS;
  unsigned short  CHARGER_CURRENT_AD8402_mA_PER_BIT;
  unsigned short  CHARGER_CURRENT_AD_mA_PER_BIT;
  unsigned short  CHARGER_MAX_POWER_10A;
  unsigned short  CHARGER_MAX_POWER_16A;
  signed char     CHARGER_VOLTAGE_OFFSET_POT;
  signed char     CHARGER_VOLTAGE_OFFSET_ADC;
  unsigned short  CHARGE_MINIMUM_START_VOLTAGE;
  unsigned short  CHARGE_CONSTANT_VOLTAGE;
  unsigned short  CHARGE_CONSTANT_CURRENT;
  unsigned short  CHARGE_END_CURRENT;
  bool            TEMP_COMPENSATED;
  unsigned char   CHARGE_V_TEMP_COMPENSATION_VALUE1;
  unsigned char   CHARGE_V_TEMP_COMPENSATION_VALUE2;
  unsigned char   CHARGE_V_TEMP_COMPENSATION_VALUE3;
  unsigned char   CHARGE_C_TEMP_COMPENSATION_VALUE1;
  unsigned char   CHARGE_C_TEMP_COMPENSATION_VALUE2;
  unsigned char   CHARGE_C_TEMP_COMPENSATION_VALUE3;
  signed char     CHARGE_TEMP_COMPENSATION_TEMPERATURE1;
  signed char     CHARGE_TEMP_COMPENSATION_TEMPERATURE2;
  signed char     CHARGE_TEMP_COMPENSATION_TEMPERATURE3;
} EEPROMSettings;

// global data structure
typedef struct {
  signed char     ExternalTemperature;
  unsigned short  RawTemperature;
  unsigned short  Voltage;
  unsigned short  Current;
  unsigned short  RawCurrent;
} GlobalStrukture;
 

#endif // End of file
