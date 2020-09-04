///////////////////////////////////////////////////////////
//NG3 Charger controller
//DATE: 2020.06.17
//AUTHOR version 1.1: CCS
//AUTHOR version 2.0: Jens
//VERSION: 2.0
//REVISION HISTORY:
//v1.1: Initial public version using a PIC MCU, see link below
//v2.0: Major change, platform changed to AVR/Arduino, serial configuration added, CAN control option added
//
// Charger can operate either in standalone mode or be controlled by BMS via CAN messages
// 
// Elcon Charger CAN protocol is used at 500kbs, speed should be possible to changed via serial protocol - comming feature
//
// Most voltage and current values in configuration and formel 
// are multiplied by 10 to get 100mV/100mA precision
//
// This code is started on code from the following tread
// https://www.diyelectriccar.com/forums/showthread.php/ng3-chargers-cant-current-limit-lithium-64827p2.html?highlight=zivan
//
///////////////////////////////////////////////////////////

#include <Arduino.h>
#include <EEPROM.h>
#include <SPI.h>
#include "CONFIG.h"
#include <mcp_can.h>
#include "SERIALCOMM.h"
#include "thermfactor.c" // Temperature array

//PROTOTYPE FUNCTIONS
void set_cv(unsigned char bits);
void set_cc(unsigned char bits);
void LoadInitialSettings(void);
void CanControlledCharger(void);
void StandaloneCharger(void);
void CalibrateVoltage( unsigned char lowHigh );
void ChargerTestMode(void);

// These define the digital output pins used to control various
// features of the board.
#define CS_NG3_PIN          9  //Chip select for the digital potentiometer
#define CS_CAN_PIN          8  //Chip select for the MCP2515 CAN controller
#define CAN_MSG_PIN         6  //Input pin used to check if MCP2515 CAN controller have recived a message
#define SOFT_START_PIN      4  //Soft start pin, used to enable/disable charging
#define WDO_PIN             5  //Watchdog Output pin, must toggle to keep charger running
#define FANS_PIN            7  //Enables the fans
#define AUX1_PIN            15 //Controls aux relay #1
#define AUX2_PIN            14 //Controls aud relay #2
#define BUZZER_PIN          10 //Controls digital sounder, requires a square wave for sound
#define LEDG_PIN            2  //Controls the green LED (LED1)
#define LEDR_PIN            3  //Controls the red LED (LED2)
// SS                  10 // Slave select is used for the BUZZER
// MOSI                11
// MISO                12
// SCK                 13

// These are the ports used for the analog to digital converter
#define AD_OVER_TEMP   A2
#define AD_EXT_TEMP    A3
#define AD_VOLTAGE     A4
#define AD_CURRENT     A5

// Declare CAN module
MCP_CAN CAN0(CS_CAN_PIN);

// Declare serial communication 
SERIALCOMM SerialCom;

SERIALCOMM_T _SerialComStatus = NoData;

// Temperature table
extern const unsigned char PROGMEM THERMFACTOR[];

// Stored configuration
EEPROMSettings settings;

// GlobalData used in the project
GlobalStrukture GlobalData;

unsigned short over_temp_bits = 0;
unsigned short ext_temp_bits = 0;
unsigned short voltage_bits = 0;
unsigned short current_bits = 0;
unsigned char digital_pot_bits = 0;

unsigned short voltage_min_bits = 0;
unsigned short voltage_max_bits = 0;
unsigned char  digital_pot_min_bits = 0;
unsigned char  digital_pot_max_bits = 0;
unsigned char  calibrationStatus = 0;

float over_temp = 0;
float ext_temp = 0;
unsigned short voltage = 0;
unsigned short current = 0;
 
#define IDLE_STATE             0
#define SOFT_START       1
#define CONSTANT_CURRENT 2
#define CONSTANT_VOLTAGE 3
#define TEST_MODE        4

bool CAN_Status = false;
bool CAN_TimeOut = true;
unsigned char CAN_TimeCount = 0;

unsigned short CAN_VoltageValue;
unsigned short CAN_CurrentValue;

unsigned char state = IDLE_STATE;
unsigned long i = 0;

unsigned short ChargerVoltagePrAD8402Bit = 0;
unsigned short ChargerVoltagePrADBit = 0;

// Task timer variables
unsigned long looptime_1s = 0; //ms

unsigned char testModeTimer = 0;



///////////////////////////////////////////////////////////////////////////
//Setup
///////////////////////////////////////////////////////////////////////////
void setup()
{

  //Initialize input ports
  pinMode(MISO,INPUT);   // This should be default when loading the SPI module
  pinMode(CAN_MSG_PIN,INPUT);
 
  //Initialize output ports
  pinMode(MOSI,OUTPUT);  // This should be default when loading the SPI module
  pinMode(SS,OUTPUT);    // This should be default when loading the SPI module
  pinMode(CS_NG3_PIN,OUTPUT);
  pinMode(CS_CAN_PIN,OUTPUT);
  pinMode(SOFT_START_PIN,OUTPUT);
  pinMode(WDO_PIN,OUTPUT);
  pinMode(FANS_PIN,OUTPUT);
  pinMode(AUX1_PIN,OUTPUT);
  pinMode(AUX2_PIN,OUTPUT);
  pinMode(BUZZER_PIN,OUTPUT);
  pinMode(LEDG_PIN,OUTPUT);
  pinMode(LEDR_PIN,OUTPUT);
 
  digitalWrite(SOFT_START_PIN,HIGH);       
  digitalWrite(SS,HIGH);
  digitalWrite(CS_NG3_PIN,HIGH); 
  digitalWrite(CS_CAN_PIN,HIGH);
  digitalWrite(BUZZER_PIN, LOW);

  // Starte Serial
  SerialCom.begin();
  
  // Load stored configuration
  EEPROM.get(0, settings);

  // Read extern temperature - value should be 210-40 if sensor not is connected
  ext_temp_bits = (unsigned char)(analogRead( AD_EXT_TEMP )/4);
  
  // Convert to temperature
  GlobalData.ExternalTemperature = (signed char)(pgm_read_word(&THERMFACTOR[ext_temp_bits])-40);

  // Check if EEPROM version fit this program version, if not update/initilize the EEPROM
  if (settings.VERSION != EEPROM_VERSION)
  {
    LoadInitialSettings();
  }

  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV8);

  // Load charger settings from EEPROM
  ChargerVoltagePrAD8402Bit = (unsigned short)(((float)( settings.CHARGER_MAX_VOLTS - settings.CHARGER_MIN_VOLTS ) / settings.CHARGER_VOLTAGE_AD8402_BITS) * 100 ) ; // Calculate the generated voltage value pr. AD8402 bit, mV
  ChargerVoltagePrADBit = (unsigned short)(((float)( settings.CHARGER_MAX_VOLTS - settings.CHARGER_MIN_VOLTS ) / settings.CHARGER_VOLTAGE_AD_BITS) * 1000 ) ; // Calculate the mesared voltage value pr. AD bit, mV * 10
 
  if (settings.CAN_ENABLED)
  {
    // Initialize MCP2515 running at 8MHz with a baudrate according to eeprom settings, extended IDs ONLY
    if(CAN0.begin(MCP_EXT, settings.CAN_SPEED, MCP_8MHZ) == CAN_OK) CAN_Status = true;
  
    if (CAN_Status) // Continue CAN configuration if CAN init is successful
    {
      // Set CAN message filters to only accept messages with ID 0x1806E5F4
      CAN0.init_Mask(0,1,0x1FFFFFFF);                // Init first mask...
      CAN0.init_Filt(0,1,0x1806E5F4);                // Init first filter...
  
      CAN0.setMode(MCP_NORMAL); // Change to normal mode to allow messages to be transmittet
    }

    // In CAN mode we set output voltage and current to minimum at start 
    set_cv(0);
    set_cc(0);
  }
  else
  {
    //Set the constant voltage settings of the AD8402
    digital_pot_bits  = (unsigned char)(((settings.CHARGE_CONSTANT_VOLTAGE + settings.CHARGER_VOLTAGE_OFFSET_POT) - settings.CHARGER_MIN_VOLTS)*100 / (ChargerVoltagePrAD8402Bit) );
    set_cv(digital_pot_bits);
    
    //Set the constant current settings of the AD8402
    digital_pot_bits  = (unsigned char)( settings.CHARGE_CONSTANT_CURRENT*100 / settings.CHARGER_CURRENT_AD8402_mA_PER_BIT );
    set_cc(digital_pot_bits);
    
    // Standalone mode, set state to SOFT_START
    state = SOFT_START;
  }

  //Sound the buzzer at 2000Hz in 1 sec - just for the heck of it
  tone(BUZZER_PIN, 2000, 1000);

}


///////////////////////////////////////////////////////////////////////////
//Continue running the main processing loop
///////////////////////////////////////////////////////////////////////////
void loop() {

    //Sample the currents and voltage inputs
    voltage_bits = analogRead( AD_VOLTAGE );
    current_bits = analogRead( AD_CURRENT );

    GlobalData.RawCurrent = current_bits; // Debug

    // Sample the temperature inputs
    over_temp_bits = analogRead( AD_OVER_TEMP );
    ext_temp_bits = (unsigned char)(analogRead( AD_EXT_TEMP )/4);

    GlobalData.RawTemperature = ext_temp_bits; // Used from debuging

    // Convert to temperature - this have to be corrected as the NG3 only have a very narrow temperature range
    GlobalData.ExternalTemperature = (signed char)(pgm_read_word(&THERMFACTOR[ext_temp_bits])-40);
  
    // Voltage equation
    GlobalData.Voltage = (unsigned short)(((unsigned long)voltage_bits * ChargerVoltagePrADBit) / 1000 ) + settings.CHARGER_MIN_VOLTS + settings.CHARGER_VOLTAGE_OFFSET_ADC;

    // Current equation
    GlobalData.Current = (unsigned short)((current_bits * settings.CHARGER_CURRENT_AD_mA_PER_BIT) / 100);

    _SerialComStatus = SerialCom.handleSerial();

    // Check if we need to hande the output from SerialCom
    switch (_SerialComStatus)
    {
      case FindMinVolt: // Find minimum output voltage the charger can handle
        CalibrateVoltage(LOW);
        break;

      case FindMaxVolt: // Find max output voltage the charger can handle
        CalibrateVoltage(HIGH);
        break;

      case NewConfig: // Save to EEPROM
        EEPROM.put(0, settings);
        break;

      case TestMode: // Switch charger to Test mode
        state = TEST_MODE;
        break;
        
      default:
        // if nothing else matches, do the default
        // default is optional
        break;
    }

    if (state != TEST_MODE)
    {
      if (settings.CAN_ENABLED)
      {
          CanControlledCharger(); // Charger controlled by CAN messages
      }
      else
      {
          StandaloneCharger(); // Standalone - charge according to EEPROM values
      }
    }
    else
    {
      ChargerTestMode(); // Run charger i test mode for 60 sec
    }
    
    // Toggle the NG3 watchdog timer, it must be toggled frequently to keep the logic
    // board's reset controller from thinking the microprocessor is locked up and
    // shutting down the output.
    digitalWrite(WDO_PIN,1);
    digitalWrite(WDO_PIN,0);

}/////END OF LOOP

         

////////////////////////////////////////////////////////////////////////
//SUBROUTINES

////////////////////////////////////////////////////////////////////////


//Set the connstant voltage setting
void set_cv(unsigned char bits)
{
  //Set the constant voltage settings of the AD8402
  digitalWrite(CS_NG3_PIN, 0);
  SPI.transfer(0x01);
  SPI.transfer(bits);
  digitalWrite(CS_NG3_PIN, 1);
}

//Set the constant current setting
void set_cc(unsigned char bits)
{
  //Set the constant current settings of the AD8402
  digitalWrite(CS_NG3_PIN, 0);
  SPI.transfer(0x00);
  SPI.transfer(bits);
  digitalWrite(CS_NG3_PIN, 1);
}

void LoadInitialSettings(void)
{
  settings.VERSION = EEPROM_VERSION;
  settings.CONFIGURED = false;
  settings.CAN_ENABLED = false;
  settings.CAN_SPEED = CAN_500KBPS;
  settings.CHARGER_MIN_VOLTS = 732;
  settings.CHARGER_MAX_VOLTS = 1005;
  settings.CHARGER_VOLTAGE_AD8402_BITS = 250;
  settings.CHARGER_VOLTAGE_AD_BITS = 1015;
  settings.CHARGER_CURRENT_AD8402_mA_PER_BIT = 111;
  settings.CHARGER_CURRENT_AD_mA_PER_BIT = 26;
  settings.CHARGER_MAX_POWER_10A = 2300;
  settings.CHARGER_MAX_POWER_16A = 3000;
  settings.CHARGER_VOLTAGE_OFFSET_POT = 3;
  settings.CHARGER_VOLTAGE_OFFSET_ADC = -3;
  settings.CHARGE_MINIMUM_START_VOLTAGE = 0;
  settings.CHARGE_CONSTANT_VOLTAGE = 850;
  settings.CHARGE_CONSTANT_CURRENT = 200;
  settings.CHARGE_END_CURRENT = 10;
  settings.TEMP_COMPENSATED = false;
  settings.CHARGE_V_TEMP_COMPENSATION_VALUE1 = 90;
  settings.CHARGE_V_TEMP_COMPENSATION_VALUE2 = 100;
  settings.CHARGE_V_TEMP_COMPENSATION_VALUE3 = 105;
  settings.CHARGE_C_TEMP_COMPENSATION_VALUE1 = 60;
  settings.CHARGE_C_TEMP_COMPENSATION_VALUE2 = 100;
  settings.CHARGE_C_TEMP_COMPENSATION_VALUE3 = 90;
  settings.CHARGE_TEMP_COMPENSATION_TEMPERATURE1 = -10;
  settings.CHARGE_TEMP_COMPENSATION_TEMPERATURE2 = 0;
  settings.CHARGE_TEMP_COMPENSATION_TEMPERATURE3 = 35;

  // Save to EEPROM
  EEPROM.put(0, settings);
}

// Standalone control functions
void StandaloneCharger(void)
{ 
  if ( GlobalData.Voltage >= settings.CHARGE_MINIMUM_START_VOLTAGE ) // Check if battery voltage is higher than defined minimum voltage - if battery voltage is lower then the charger should not start
  {
    // If first time through, go to the CONSTANT_CURRENT state and
    // enable the charger output
    if( state == SOFT_START ) {
       state = CONSTANT_CURRENT; //Now we are in constant current state

       // If SOFT_START_PIN not is enabled e.g. = 0 then enable it
       if (digitalRead(SOFT_START_PIN) == 1)
       {
        digitalWrite(SOFT_START_PIN, 0); // Setting SOFT_START pin to low enables the actual output of the charger
       }
 
       // Set the LED and AUX relays any way you want for the CC state
       // Remember, setting an LED to 0 actually turns it on. Setting it to 1 turns it off.
       // Both LEDs set to 0 gives the YELLOW color.
        digitalWrite(LEDG_PIN, 1);
        digitalWrite(LEDR_PIN, 0);
        digitalWrite(AUX1_PIN, 0);
        digitalWrite(AUX2_PIN, 1);
  //      digitalWrite(FANS_PIN,1); // Start the fans

     // Currently in the constant current mode
    } else if ( state == CONSTANT_CURRENT ) {

      // stay in constant current mode until the voltage >= CV setpoint then go to CV state
      if( GlobalData.Voltage >= settings.CHARGE_CONSTANT_VOLTAGE ) {
        //Now we are in the constant voltage state
        state = CONSTANT_VOLTAGE; 

        // Set the LED and AUX relays any way you want for the CV state
        // Remember, setting an LED to 0 actually turns it on. Setting it to 1 turns it off.
        // Both LEDs set to 0 gives the YELLOW color.
        digitalWrite(LEDG_PIN, 0);
        digitalWrite(LEDR_PIN, 0);
        digitalWrite(AUX1_PIN, 1);
        digitalWrite(AUX2_PIN, 1);
      }

    //  Currently in the constant voltage mode
    } else if ( state == CONSTANT_VOLTAGE ) {
  
      //Stay in CV state until current drops to Imin then goto IDLE state
      if( GlobalData.Current <= settings.CHARGE_END_CURRENT ) {
        state = IDLE_STATE; // Now in the IDLE state, the charger is essentially done
                      // until it gets power cycled. This can be handled
                      // differently if you want.

        // Set the LED and AUX relays any way you want for the CV state
        // Remember, setting an LED to 0 actually turns it on. Setting it to 1 turns it off.
        // Both LEDs set to 0 gives the YELLOW color.
        digitalWrite(LEDG_PIN, 0);
        digitalWrite(LEDR_PIN, 1);
        digitalWrite(AUX1_PIN, 1);
        digitalWrite(AUX2_PIN, 0);
        digitalWrite(SOFT_START_PIN,1);   // Turns off the chargers output
        digitalWrite(FANS_PIN,0); // Stops the fans
      }
    }
  }
}

// CAN control function
void CanControlledCharger(void)
{
  long unsigned int rxId;
  unsigned char rxLen = 0;
  unsigned char rxBuf[8];
  unsigned char txBuf[8];
  unsigned short recivedVoltageValue;
  unsigned short recivedCurrentValue;
  
  //if ( !digitalRead(CAN_MSG_PIN) ) // If CAN_MSG_PIN pin is low, read receive buffer
  if ( CAN0.checkReceive() == CAN_MSGAVAIL ) // check if CAN message is recived - by using this function we save a pin
  {
    CAN0.readMsgBuf(&rxId, &rxLen, rxBuf);      // Read data: len = data length, buf = data byte(s)
  }

  if ( rxId == 0x1806E5F4 ) // Check if message ID is correct, it should not be possible to recive any other message but i like checking
  {
    // Reset time count
    CAN_TimeCount = 0;
    
    // Get the recived values
    recivedVoltageValue = (short)(((short)rxBuf[0]) << 8) | rxBuf[1];
    recivedCurrentValue = (short)(((short)rxBuf[2]) << 8) | rxBuf[3];

    // Check if recived values have changed - if not its just a heartbeat message 
    if ( recivedVoltageValue != CAN_VoltageValue )
    {
      //Set the constant voltage settings of the AD8402
      digital_pot_bits  = (unsigned char)(( (recivedVoltageValue + settings.CHARGER_VOLTAGE_OFFSET_POT) - settings.CHARGER_MIN_VOLTS )*100 / ChargerVoltagePrAD8402Bit );
      set_cv(digital_pot_bits);
      
      CAN_VoltageValue = recivedVoltageValue;
    }

    if ( recivedCurrentValue != CAN_CurrentValue )
    {
            
      //Set the constant current settings of the AD8402
      digital_pot_bits  = (unsigned char)( recivedCurrentValue * 100 / settings.CHARGER_CURRENT_AD8402_mA_PER_BIT );
      set_cc(digital_pot_bits); 
      
      CAN_CurrentValue = recivedCurrentValue;
    }

    if ( rxBuf[4] == 0 ) // Start charging if 0 - all other values turnoff the charger
    {
      digitalWrite(SOFT_START_PIN,0); 
    }
    else
    {
      CAN_TimeOut = true;
      digitalWrite(SOFT_START_PIN,1); 
    }
  }

  // Every 500ms tasks
  if (millis() - looptime_1s > 500UL )
  {
    looptime_1s = millis();

    // Send charger output voltage, current and status 
    txBuf[0] = highByte(GlobalData.Voltage);
    txBuf[1] = lowByte(GlobalData.Voltage);
    txBuf[2] = highByte(GlobalData.Current);
    txBuf[3] = lowByte(GlobalData.Current);
    txBuf[4] = 0; //Status byte - 0 = no errors - will be updated in comming version
    txBuf[5] = 0;
    txBuf[6] = 0;
    txBuf[7] = 0;

    //Status bits
    //0 - 0: Normal. 1: Hardware Failure
    //1 - 0: Normal. 1: Over temperature protection
    //2 - 0: Input voltage is normal. 1. Input voltage is wrong, the charger will stop working.
    //3 - 0: Charger detects battery voltage and starts charging. 1: Charger stays turned off (to prevent reverse polarity).
    //4 - 0: Communication is normal. 1: Communication receive time-out.

    // Send message
    CAN0.sendMsgBuf(0x18FF50E5, CAN_EXTID, 8, txBuf);

    // Increase counter
    CAN_TimeCount++;

    // Charger stop after 5 seconds without reciving a valid CAN message
    if ( CAN_TimeCount >= 10 ) 
    {
      CAN_TimeOut = true;
      CAN_TimeCount = 10; // No need to increase the value to more than 5
    }
  }
  
  if ( CAN_TimeOut ) // Turn off the chargers output if CAN_TimeOut is true
  { 
    digitalWrite(SOFT_START_PIN,1); 
  }
}

// Function that find the lowest/highest charger voltage value - used for calibration of the charger
void CalibrateVoltage( unsigned char lowHigh )
{
    unsigned char  voltagePotBits = 128;
    unsigned short voltageBits;

    // Turn on the red led
    digitalWrite(LEDG_PIN, 1);
    digitalWrite(LEDR_PIN, 0);

    // Enable charger output
    digitalWrite(SOFT_START_PIN, LOW);

    for (int i=0;i<127;i++)
    {
      // Set charger voltage
      set_cv( voltagePotBits );

      // Short delay to ensure the voltage is stable
      delay(100);

      // Read voltage
      voltageBits = analogRead( AD_VOLTAGE );

      // Check if min/max mesuring limit is reached - exit for loop if yes
      if ( voltageBits == 1 || voltageBits == 1023 ) 
      { 
        break;
      }

      // set next value
      if ( lowHigh == HIGH )
      {
        voltagePotBits++;  
      }
      else
      {
        voltagePotBits--;
      }

      // Keep the NG3 watchdog happy
      digitalWrite(WDO_PIN,1);
      digitalWrite(WDO_PIN,0);
    }

    // Copy to the "global" values
    if ( lowHigh == HIGH )
    {
      voltage_max_bits = voltageBits;
      digital_pot_max_bits = voltagePotBits;
      if ( calibrationStatus == 1 ) calibrationStatus = 2;
    }
    else
    {
      voltage_min_bits = voltageBits;
      digital_pot_min_bits = voltagePotBits;
      calibrationStatus = 1;
    }

    // We are ready to calculate the values, making them ready to be saved
    if ( calibrationStatus == 2 )
    {
      settings.CHARGER_VOLTAGE_AD8402_BITS = digital_pot_max_bits - digital_pot_min_bits;
      settings.CHARGER_VOLTAGE_AD_BITS = voltage_max_bits - voltage_min_bits;
    }

    // Blink the leds for 30 seconds - charger output voltage have to be measured in this periode
    for (int i=0;i<=60;i++)
    {
      // Short delay
      delay(500);

      // Blink the LED - It will blink red/yellow
      digitalWrite(LEDG_PIN, !digitalRead(LEDG_PIN));

      // Keep the NG3 watchdog happy
      digitalWrite(WDO_PIN,1);
      digitalWrite(WDO_PIN,0);
    }

    // Disable the output from the charger
    digitalWrite(SOFT_START_PIN, HIGH);
}

void ChargerTestMode(void)
{  
  //Set the constant voltage settings of the AD8402
  digital_pot_bits  = (unsigned char)( ((settings.CHARGE_CONSTANT_VOLTAGE + settings.CHARGER_VOLTAGE_OFFSET_POT) - settings.CHARGER_MIN_VOLTS)*100 / (ChargerVoltagePrAD8402Bit) );
  set_cv(digital_pot_bits);
  
  //Set the constant current settings of the AD8402
  digital_pot_bits  = (unsigned char)( settings.CHARGE_CONSTANT_CURRENT*100 / settings.CHARGER_CURRENT_AD8402_mA_PER_BIT );
  set_cc(digital_pot_bits);

  // Turn on the red led
  digitalWrite(LEDG_PIN, 1);
  digitalWrite(LEDR_PIN, 0);

  // Enable charger output
  digitalWrite(SOFT_START_PIN, LOW);

  // Every 500ms tasks
  if (millis() - looptime_1s > 500UL )
  {
    looptime_1s = millis();

    testModeTimer++;

    // Blink the LED - It will blink red/yellow
    digitalWrite(LEDG_PIN, !digitalRead(LEDG_PIN));

    if (testModeTimer == 120) // End of test mode
    {
      // Disable the output from the charger
      digitalWrite(SOFT_START_PIN, HIGH);
    
      // Switch to idle mode - only relevant in standalon mode
      state = IDLE_STATE;
    
      // Turn on the green led
      digitalWrite(LEDG_PIN, 0);
      digitalWrite(LEDR_PIN, 1);

      // ready to run next test cycle
      testModeTimer = 0;
    }
  }
}
