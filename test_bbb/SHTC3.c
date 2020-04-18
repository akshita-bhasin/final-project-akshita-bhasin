/*
File:   SHTC3.c
Author: Madhukar Arora
@Brief: Source file to read Relative Humidity Values using I2C from the SHTC3 Humidity Sensor

@ Leverage Code:
https://stackoverflow.com/questions/52975817/setup-i2c-reading-and-writing-in-c-language
*/


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

int shtc3_read_single_byte(int file, unsigned char device_addr,unsigned char command, unsigned char* val);
int shtc3_write_single_byte(int file, unsigned char device_addr, uint8_t command, unsigned char val);

#endif /* SHTC3_H*/


#define I2C_DEVICE "/dev/i2c-2"

int main(void)
{
    int fd;
    int ret;
    unsigned char ack;

    uint8_t MSB,LSB;

    printf("Starting the Humidity Sensor Application\n");

    if((fd = open(I2C_DEVICE,O_RDWR)) < 0){
        perror("Could not open the I2C Device\n");
        return 1;
    }

    if((ioctl(fd,I2C_SLAVE,SHTC3_ADDR))){
        perror("Failed connection with the Humidity Sensor\n");
        return 1;
    }

    // wakeup command, measurement command, read out command , sleep command
    if((ret = shtc3_read_single_byte(fd,SHTC3_ADDR,SHTC3_WRITE,&ack)) != 0)
    {
        perror("Error reading ack, wakeup");
        return -1;
    }
    printf("Wake up write : %c\n",ack);
    MSB = (uint8_t)(SHTC3_WAKE >> 8);
    LSB = (uint8_t)(SHTC3_WAKE & 0x00FF);

    if((ret = shtc3_read_single_byte(fd,SHTC3_ADDR,MSB,&ack)) != 0)
    {
        perror("Error reading ack, wakeup");
        return -1;
    }

    printf("Wake up MSB : %c\n",ack);
    if((ret = shtc3_read_single_byte(fd,SHTC3_ADDR,LSB,&ack)) != 0)
    {
        perror("Error reading ack, wakeup");
        return -1;
    }
    printf("Wake up LSB : %c\n",ack);

}

 /*
// https://stackoverflow.com/questions/52975817/setup-i2c-reading-and-writing-in-c-language
// */
int shtc3_read_single_byte(int file, unsigned char device_addr,unsigned char command, unsigned char* val)
{
    unsigned char inbuf;
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[2];

    /*write command to the slave*/
    messages[0].addr  = device_addr;
    messages[0].flags = 0;
    messages[0].len   = sizeof(command);
    messages[0].buf = &command;

    /*read from the slave*/
    messages[1].addr = device_addr;
    messages[1].flags = I2C_M_RD; //read data from slave to master
    messages[1].len = sizeof(inbuf);
    messages[1].buf = &inbuf;

    packets.msgs = messages;
    packets.nmsgs = 2;
    
    if(ioctl(file,I2C_RDWR,&packets) < 0){
        perror("Error reading a single byte");
        return -1;
    }
    *val = inbuf;
    return 0;    
}

int shtc3_write_single_byte(int file, unsigned char device_addr, uint8_t command, unsigned char val)
{
    uint8_t outbuf[2];
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[2];
    /*write command to the slave*/
    messages[0].addr  = device_addr;
    messages[0].flags = 0;
    messages[0].len   = sizeof(outbuf);
    messages[0].buf = outbuf;

    outbuf[0] = command;
    outbuf[1] = val;

    packets.msgs = messages;
    packets.nmsgs = 1;
    if(ioctl(file,I2C_RDWR,&packets) < 0){
        perror("Error writing");
        return -1;
    }

    return 0;    
}
