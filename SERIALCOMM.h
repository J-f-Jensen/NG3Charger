#ifndef _SERIALCOMM_H_
#define _SERIALCOMM_H_

#include "CONFIG.h"

#define SERIALCOMM_DEFAULT_BAUD 9600

// Stored configuration
extern EEPROMSettings settings;

// GlobalData used in the project
extern GlobalStrukture GlobalData;

enum SERIALCOMM_T { NoData, NewConfig, FindMinVolt, FindMaxVolt, TestMode };

class SERIALCOMM {
  public:
    /* user interface */    
    void begin( void ) { begin( SERIALCOMM_DEFAULT_BAUD); }; // begin with default baudrate
    void begin( int baudrate );
    SERIALCOMM_T handleSerial(void);
  
  //protected:
  
  private:
    /* function(s) */
    void             init(int baudrate);

    /* variable */
    bool             _ComIsActive = false;
    unsigned char    _menuload = 0;
    unsigned char    _dataload;
};

#endif // End of file
