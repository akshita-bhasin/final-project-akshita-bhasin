// https://github.com/jbdatko/tmp102
// https://www.kernel.org/doc/Documentation/i2c/dev-interface
// https://stackoverflow.com/questions/52975817/setup-i2c-reading-and-writing-in-c-language
// Exploring Beaglebone - Derek Molly for I2C configuration

#include <errno.h>
#include <string.h>
#include <stdio.h>
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

#define SLAVE_ADDR 0x48
int fd;

void signal_handler(int signum)
{
  assert(0 == close(fd));
  exit(signum);
}

int main(){


  char *bus = "/dev/i2c-2"; /* Pins P9_19 and P9_20 */
  int addr = SLAVE_ADDR;          /* The I2C address of TMP102 */
  char buf[2] = {0};
  int temp;
  unsigned char MSB, LSB;

  float f,c;

  if ((fd = open(bus, O_RDWR)) < 0) {
    /* ERROR HANDLING: you can check errno to see what went wrong */
    perror("Failed to open the i2c bus");
    exit(1);
  }

 if (ioctl(fd, I2C_SLAVE, addr) < 0) {
   perror("Failed to acquire bus access and/or talk to slave.\n");
   /* ERROR HANDLING; you can check errno to see what went wrong */
   exit(1);
 }

 /* Register the signal handler */
 signal(SIGINT, signal_handler);

 while(1)
   {
     // Using I2C Read
     if (read(fd,buf,2) != 2) {
       /* ERROR HANDLING: i2c transaction failed */
       perror("Failed to read from the i2c bus.\n");

   } else {

       MSB = buf[0];
       LSB = buf[1];

       /* Convert 12bit int using two's compliment */
       /* Credit: http://bildr.org/2011/01/tmp102-arduino/ */
       temp = ((MSB << 8) | LSB) >> 4;

       c = temp*0.0625;
       f = (1.8 * c) + 32;

       printf("Temp Fahrenheit: %f Celsius: %f\n", f, c);
     }
     sleep(1);
   }



}
