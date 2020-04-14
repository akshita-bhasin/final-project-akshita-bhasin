//https://class.ece.uw.edu/474/peckol/code/BeagleBone/ForestExamples/Example_6_UART/uart.c

// http://man7.org/linux/man-pages/man3/termios.3.html
// Exploring Beaglebone - Derek Molly for UART pins and example C code
// Use child-parent, by using fork() to transmit and receive between Tiva and BBB. UART1 and UART5

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdint.h>

int main(void)
{
    int fd1, count;
    struct termios options;
    char tx[20] = "UART Test!", rx[20];

    printf("Testing uart implementation with Tiva");

    if ((fd1 = open("/dev/ttyO1", O_RDWR | O_NOCTTY | O_NDELAY)) < 0)
    {
        perror("open\n");
        return -1;
    }

    tcgetattr(fd1, &options);

    options.c_cflag |= (B115200 | CS8 | CREAD | CLOCAL);
    options.c_iflag = IGNPAR | ICRNL;    //ignore partity errors, CR -> newline

    tcsetattr(fd1, TCSAFLUSH, &options);

    printf("Sending: '%s'\n", tx);
    if ((count = write(fd1, &tx, 17)) < 0)
    {
        perror("write\n");
        return -1;
    }

    usleep(100000); ////give the Beaglebone a chance to respond

    printf("Receive characters\n");

    if ((count = read(fd1, (void*)rx, 17)) < 0)
    {
        perror("read\n");
        return -1;
    }

    if(count)
    {
        printf("Received-> '%s'", rx);
    }

    close(fd1);
    return 0;
}