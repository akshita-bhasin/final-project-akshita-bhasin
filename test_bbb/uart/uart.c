//https://class.ece.uw.edu/474/peckol/code/BeagleBone/ForestExamples/Example_6_UART/uart.c

// http://man7.org/linux/man-pages/man3/termios.3.html
// Exploring Beaglebone - Derek Molly for UART pins and example C code

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdint.h>

int main(void)
{
    int fd1, count;
    struct termios options;
    char tx[20] = "Hello TM4C123GXL", rx[20];

    printf("Testing uart implementation");

    if ((fd1 = open("/dev/ttyO1", O_RDWR | O_NOCTTY | O_NDELAY)) < 0)
    {
        perror("open\n");
        return -1;
    }

    tcgetattr(fd1, &options);
    if((cfsetispeed(&options, B4800)) == -1)
    {
        perror("Input baud rate\n");
        return -1;
    }
    if((cfsetospeed(&options, B4800)) == -1)
    {
        perror("Output baud rate\n");
        return -1;
    } 

    options.c_cflag |= (CLOCAL | CS8);
    options.c_iflag &= ~(ISTRIP | IXON | INLCR | PARMRK | ICRNL | IGNBRK);
    options.c_oflag = 0;
    options.c_lflag = 0;

    tcsetattr(fd1, TCSAFLUSH, &options);

    printf("Sending: '%s'\n", tx);
    if ((count = write(fd1, &tx, 17)) < 0)
    {
        perror("write\n");
        return -1;
    }

    usleep(100000);

    printf("Receive characters\n");

    if ((count = read(fd1, (void*)rx, 17)) < 0)
    {
        perror("read\n");
        return -1;
    }

    if(count)
    {
        printf("Received-> '%s", rx);
    }

    close(fd1);
    return 0;
}