/*
File:   ambient.c
Author: Madhukar Arora
@Brief: contains function to read lux values from VEML6030 

@ Leverage Code:
https://stackoverflow.com/questions/52975817/setup-i2c-reading-and-writing-in-c-language

https://github.com/shenki/linux-i2c-example/blob/master/i2c_example.c
*/

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


#define I2C_DEVICE "/dev/i2c-1"

#define VEML_ADDR 0x48
#define VEML_WRITE 0x90
#define VEML_READ 0x91


// function prototypes
int read_single_byte(int file, unsigned char device_addr, int* val);
int write_single_byte(int file, unsigned char device_addr, int command);
int read_word(int file, unsigned char device_addr,int *val);
int write_word(int file, unsigned char device_addr, int* command);


 __s32 i2c_smbus_access(int file, char read_write, __u8 command, int size, union i2c_smbus_data *data)
 {
     struct i2c_smbus_ioctl_data args;
     
     args.read_write = read_write;
     args.command    = command;
     args.size       = size;
     args.data       = data;

     return ioctl(file,I2C_SMBUS,&args);
 }

static inline __s32 i2c_smbus_read_byte_data(int file, __u8 command)
{
    union i2c_smbus_data data;
    if(i2c_smbus_access(file,I2C_SMBUS_READ,command,I2C_SMBUS_BYTE_DATA,&data))
        return -1;
    else
    {
        return 0x0FF & data.byte;
    }
    
}


static inline __s32 i2c_smbus_read_word_data(int file, __u8 command)
{
    union i2c_smbus_data data;
    if(i2c_smbus_access(file,I2C_SMBUS_READ,command,I2C_SMBUS_WORD_DATA,&data))
        return -1;
    else
    {
        return 0x0FFFF & data.byte;
    }
}

 int power_on_config = 0x800; //MSB = 0x8, LSB 0x00
 int power_saving_off = 0x0000; //PSM mode 1, disabled
 int  power_saving_on = 0x0001; //PSM mode 1, enabled

 //register address, LSB, MSB
 int  power_on_command[3] = {0x00, 0x00, 0x08}; 
 int psaveoff_command[3]  = {0x03, 0x00, 0x00};
 int psaveon_command[3]   = {0x03, 0x01, 0x00};
 int read_command         = 0x04;

 /*
// https://stackoverflow.com/questions/52975817/setup-i2c-reading-and-writing-in-c-language
// */

int write_single_byte(int file, unsigned char device_addr, int command)
{
    uint8_t outbuf;
    outbuf = command;
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg message;
    /*write command to the slave*/
    message.addr  = device_addr;
    message.flags = 0;
    message.len   = sizeof(outbuf);
    message.buf = &outbuf;
    packets.msgs = &message;
    packets.nmsgs = 1;
    if(ioctl(file,I2C_RDWR,&packets) < 0){
        perror("\n\rError writing single byte");
        return -1;
    }

    return 0;    
}




int write_word(int file, unsigned char device_addr, int* command)
{
    uint8_t outbuf[3];
    
    outbuf[0] = command[0];
    outbuf[1] = command[1];
    outbuf[2] = command[2];

    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg message;
    /*write command to the slave*/
    message.addr  = device_addr;
    message.flags = 0;
    message.len   = sizeof(outbuf);
    message.buf = outbuf;


    packets.msgs = &message;
    packets.nmsgs = 1;
    if(ioctl(file,I2C_RDWR,&packets) < 0){
        perror("Error writing a word");
        return -1;
    }

    return 0;    
}


int main(void)
{
    int i2c_fd;                         //file descriptor for i2c bus
    int status;
    uint8_t lsb, msb;                

    if((i2c_fd = open(I2C_DEVICE,O_RDWR)) < 0){
        perror("Opening I2C file error");
        return -1;
    }

    if(ioctl(i2c_fd,I2C_SLAVE,VEML_ADDR) < 0){
        perror("I2C ioctl error");
        return -1;
    }

    //power on the sensor
    if((status = write_word(i2c_fd,VEML_ADDR,power_on_command)) != 0) 
    {
        perror("Could not power on the ambient light sensor");
        return -1;
    }


    //power save command on
    if((status = write_word(i2c_fd,VEML_ADDR,psaveon_command)) != 0)
    {
        perror("Could not set up power saving mode");
        return -1;
    }

    printf("\n\rDevice setup to read values from the sensor");

    // if((status = write_single_byte(i2c_fd,VEML_ADDR,read_command)) != 0)
    // {
    //     perror("Phase 1 write for read failed");
    //     return -1;
    // }
    
    
    while(1)
    {
        lsb = i2c_smbus_read_byte_data(i2c_fd, read_command);
        msb = i2c_smbus_read_byte_data(i2c_fd,read_command);

        printf("\n\rlsb received %x   msb received %x",lsb,msb);
    }
    return 0;    
}