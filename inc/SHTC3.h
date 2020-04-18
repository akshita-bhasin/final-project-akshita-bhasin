#ifndef SHTC3_H
#define SHTC3_H

#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <signal.h>
#include <unistd.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <assert.h>

// available from the data sheet
#define SHTC3_ADDR     0b1110000              
#define SHTC3_WRITE    0b11100000
#define SHTC3_READ     0b11100001


// commands - leveraged DATA Sheet
uint16_t SHTC3_SLEEP       =       0xB098;   // {0xB0,0x98};
uint16_t SHTC3_WAKE        =       0x3517;   // {0x35,0x17}; 
uint16_t SHTC3_SW_RESET    =       0x805D;   //{0x80,0x5D};
uint16_t SHTC3_READ_ID     =       0xEFC8;   //{0xEF,0xC8};

//measurement command 
uint16_t RHF_CSD_NM       =  0x58E0;       //Relative Humidity First, Clock Stretching Disabled, Normal Mode
uint16_t RHF_CSD_LPM       =  0x401A;       //Relative Humidity First, Clock Stretching Disabled, Low Power Mode

uint16_t RHF_CSE_NM       =  0x5C24;       //Relative Humidity First, Clock Stretching Enabled, Normal Mode
uint16_t RHF_CSE_LPM      =  0x44DE;       //Relative Humidity First, Clock Stretching Enabled, Low Power Mode

//functions

/*leverage : https://github.com/sparkfun/SparkFun_SHTC3_Arduino_Library/blob/master/src/SparkFun_SHTC3.h*/
 
float SHTC3_raw2Percent(uint16_t RH);  //prototype referred from Arduino Library 

void send_command(int file, char mode, uint16_t command);

#endif /* SHTC3_H*/



