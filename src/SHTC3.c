/*
File:   SHTC3.c
Author: Madhukar Arora
@Brief: Source file to read Relative Humidity Values using I2C from the SHTC3 Humidity Sensor

@ Leverage Code:
https://stackoverflow.com/questions/52975817/setup-i2c-reading-and-writing-in-c-language

*/


#include "../inc/SHTC3.h"  

#define I2C_DEVICE "/dev/i2c-1"


//function to send a command to the device
void send_command(int file,char mode, uint16_t command)
{
    int ret = 0;
    uint8_t MSB, LSB;
    uint8_t flag = -1; //to check for ACK/NACK
    uint8_t slave_add;
    if(mode == 'w')
    {
        slave_add = SHTC3_WRITE;
    }
    else if(mode == 'r')
    {
        slave_add = SHTC3_READ;
    }
    //start
    if((write(file,slave_add,1))!= 1)
    {
        perror("Failed writing the start condition");
        return -1;
    }
    //check for ack/nack
    if((read(file,flag,1)!= 1)
    {
        perror("Error reading ACK/NACK Status");
        return -1;
    }
    //command
    MSB = (command >> 8);
    LSB = (command & 0x00FF);
    if((write(file,MSB,1))!= 1)
    {
        perror("Failed writing the MSB");
        return -1;
    }
    //check for ack/nack
    if((read(file,flag,1)!= 1)
    {
        perror("Error reading ACK/NACK Status");
        return -1;
    }

    if((write(file,LSB,1))!= 1)
    {
        perror("Failed writing the LSB");
        return -1;
    }
    //check for ack/nack
    if((read(file,flag,1)!= 1)
    {
        perror("Error reading ACK/NACK Status");
        return -1;
    }

    

}



int main(void)
{
    int fd;
    int ret = 0;

    printf("Starting the Humidity Sensor Application\n");

    if((fd = open(I2C_DEVICE,O_RDWR)) < 0){
        perror("Could not open the I2C Device\n");
        return 1;
    }

    if((ret = ioctl(fd,I2C_SLAVE,SHTC3_ADDR)) < 0){
        perror("Failed connection with the Humidity Sensor\n");
        return 1;
    }

    // wakeup command, measurement command, read out command , sleep command

    //wakeup the sensor
    send_command(fd,'w',SHTC3_WAKE);



}


// /*
// https://stackoverflow.com/questions/52975817/setup-i2c-reading-and-writing-in-c-language
// */
// int shtc3_read_single_byte(int file, unsigned char device_addr,unsigned char command, unsigned char* val)
// {
//     unsigned char inbuf;
//     struct i2c_rdwr_ioctl_data packets;
//     struct i2c_msg messages[2];

//     /*write command to the slave*/
//     messages[0].addr  = device_addr;
//     messages[0].flags = 0;
//     messages[0].len   = sizeof(command);
//     messages[0].buf = &command;

//     /*read from the slave*/
//     messages[1].addr = device_addr;
//     messages[1].flags = I2C_M_RD; //read data from slave to master
//     messages[1].len = sizeof(inbuf);
//     messages[1].buf = &inbuf;

//     packets.msgs = messages;
//     packets.nmsgs = 2;
    
//     if(ioctl(file,I2C_RDWR,&packets) < 0){
//         perror("Error reading a single byte")
//         return -1;
//     }
//     *val = inbuf;
//     return 0;    
// }

// int shtc3_write_single_byte(int file, unsigned char device_addr, uint8_t command, unsigned char val)
// {
//     uint8_t outbuf[2];
//     struct i2c_rdwr_ioctl_data packets;
//     struct i2c_msg messages[1];

//     /*write command to the slave*/
//     messages[0].addr  = device_addr;
//     messages[0].flags = 0;
//     messages[0].len   = sizeof(outbuf);
//     messages[0].buf = outbuf;

//     outbuf[0] = command;
//     outbuf[1] = val;

//     packets.msgs = messages;
//     packets.nmsgs = 1;
//     if(ioctl(file,I2C_RDWR,&packets) < 0){
//         perror("Error writing")
//         return -1;
//     }

//     return 0;    
// }
