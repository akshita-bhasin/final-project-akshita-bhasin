/*
File:   ambient.c
Author: Madhukar Arora
@Brief: 

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





 int power_on_config = 0x800; //MSB = 0x8, LSB 0x00
 int power_saving_off = 0x0000; //PSM mode 1, disabled
 int  power_saving_on = 0x0001; //PSM mode 1, enabled

//register address, LSB, MSB
 int  power_on_command[3] = {0x00, 0x00, 0x10}; 
 int psaveoff_command[3] = {0x03, 0x00, 0x00};
 int psaveon_command[3]  = {0x03, 0x01, 0x00};
 int read_command        = 0x04;



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

int main(void)
{
    int fd;
    int status;

    //int sensor_values[2] = {0}; //index 0 LSB index 1 MSB

    uint16_t sensor_val;
    if((fd = open(I2C_DEVICE,O_RDWR)) < 0){
        perror("Opening I2C file error");
        return -1;
    }

    if(ioctl(fd,I2C_SLAVE,VEML_ADDR) < 0){
        perror("I2C ioctl error");
        return -1;
    }

    //power on the sensor
    if((status = write_word(fd,VEML_ADDR,power_on_command)) != 0) 
    {
        perror("Could not power on the ambient light sensor");
        return -1;
    }


    //power save command on
    if((status = write_word(fd,VEML_ADDR,psaveon_command)) != 0)
    {
        perror("Could not set up power saving mode");
        return -1;
    }

    printf("\n\rDevice setup to read values from the sensor");

    if((status = write_single_byte(fd,VEML_ADDR,read_command)) != 0)
    {
        perror("Phase 1 write for read failed");
        return -1;
    }
    
    // if((status = read_word(fd,VEML_ADDR,sensor_values)) != 0)
    // {
    //     perror("error reading sensor values");
    //     return -1;
    // }
    while(1)
    {
    //     if(read(fd,sensor_values,2) != 2)
    //     {
    //     perror("\n\rFailed to read from the device");
    //     }
    //     else
    //     {
    //   printf("\n\rRecieved these values from the sensor \n LSB : %d \n MSB : %d\n",sensor_values[0],sensor_values[1]);  
    //     }
            sensor_val = i2c_smbus_read_word_data(fd,read_command);
            printf("\n\r received sensor value : %d",sensor_val);
    }
    
    
    //printf("\n\rRecieved these values from the sensor \n LSB : %d \n MSB : %d\n",sensor_values[0],sensor_values[1]);

    return 0;    
}


 /*
// https://stackoverflow.com/questions/52975817/setup-i2c-reading-and-writing-in-c-language
// */
int read_single_byte(int file, unsigned char device_addr, int* val)
{
    uint8_t inbuf;
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg message;

    // /*write command to the slave*/
    // messages[0].addr  = device_addr;
    // messages[0].flags = 0;
    // messages[0].len   = sizeof(command);
    // messages[0].buf = &command;

    /*read from the slave*/
    message.addr = device_addr;
    message.flags = I2C_M_RD; //read data from slave to master
    message.len = sizeof(inbuf);
    message.buf = &inbuf;

    packets.msgs = &message;
    packets.nmsgs = 1;
    
    if(ioctl(file,I2C_RDWR,&packets) < 0){
        perror("Error reading a single byte");
        return -1;
    }
    *val = inbuf;
    return 0;    
}

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


int read_word(int file, unsigned char device_addr,int *val)
{
    uint8_t inbuf[2];
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[2];

    // /*write command to the slave*/
    // messages[0].addr  = device_addr;
    // messages[0].flags = 0;
    // messages[0].len   = sizeof(command);
    // messages[0].buf = &command;

    /*read from the slave*/
    messages[0].addr = device_addr;
    messages[0].flags = I2C_M_RD; //read data from slave to master
    messages[0].len = sizeof(inbuf[0]);
    messages[0].buf = &inbuf[0];

    // /*write command to the slave*/
    // messages[2].addr  = device_addr;
    // messages[2].flags = 0;
    // messages[2].len   = sizeof(command);
    // messages[2].buf = &command;

    /*read from the slave*/
    messages[1].addr = device_addr;
    messages[1].flags = I2C_M_RD; //read data from slave to master
    messages[1].len = sizeof(inbuf[1]);
    messages[1].buf = &inbuf[1];


    packets.msgs  = messages;
    packets.nmsgs = 2;

    if(ioctl(file,I2C_RDWR,&packets) < 0){
        perror("\n\rError reading a word");
        return -1; 
    }
    val[0] = inbuf[0];
    val[1] = inbuf[1];

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
