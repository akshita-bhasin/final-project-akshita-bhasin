//Referred to Mohit Rane repository to structure main.c

#include "env_mon.h"

#define TASKS 3

void uart_init(void)
{
    struct termios options;

    if ((uart_fd1 = open("/dev/ttyO1", O_RDWR | O_NOCTTY | O_NDELAY)) < 0)
    {
        perror("open\n");
        exit(1);
    }

    tcgetattr(uart_fd1, &options);

    options.c_cflag = B115200 | CS8 | CREAD | CLOCAL;
    options.c_iflag = IGNPAR | ICRNL;
    options.c_oflag = 0;
    options.c_lflag = 0;

    tcflush(uart_fd1, TCIFLUSH);
   	tcsetattr(uart_fd1, TCSANOW, &options); 
}

void uart_deinit(void)
{
    close(uart_fd1);
}

void tx_uart(void)
{
    int count;
    char tx[20] = "Hello TM4C123GXL";
    printf("Sending: '%s'\n", tx);
    if ((count = write(uart_fd1, &tx, 17)) < 0)
    {
        perror("write\n");
        exit(1);
    }
}

void rx_uart(void)
{
    int count;
    char rx[20];
    fcntl(uart_fd1, F_SETFL, 0);

    printf("Receive characters\n");

    if ((count = read(uart_fd1, (void*)rx, 20)) < 0)
    {
        perror("read\n");
        exit(1);
    }

    if(count)
    {
        printf("Received-> %s, %d chars", rx, count);
    }
}

void signal_handler(int signum)
{
  assert(0 == close(fd));
  exit(signum);
}

void tmp102_init(void)
{
  char *bus = "/dev/i2c-1"; /* Pins P9_17 and P9_18 */
  int addr = SLAVE_ADDR;          /* The I2C address of TMP102 */
  
  if ((tmp102_fd1 = open(bus, O_RDWR)) < 0) {
    /* ERROR HANDLING: you can check errno to see what went wrong */
    perror("Failed to open the i2c bus");
    exit(1);
  }

 if (ioctl(tmp102_fd1, I2C_SLAVE, addr) < 0) {
   perror("Failed to acquire bus access and/or talk to slave.\n");
   /* ERROR HANDLING; you can check errno to see what went wrong */
   exit(1);
 }

 /* Register the signal handler */
 signal(SIGINT, signal_handler);

}

void tmp102_task(void)
{
    printf("In TMP102 task\n");
    char buf[2] = {0};
    int temp;
    unsigned char MSB, LSB;

    float f,c;
    // Using I2C Read
    if (read(tmp102_fd1,buf,2) != 2) {
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
}

int main(void)
{
    pid_t func_count[TASKS];
    int tasks, num_tasks = TASKS;
    void(*func_ptr[])(void) = { tx_uart,
                                rx_uart,
                                tmp102_task };

    uart_init();
    tmp102_init();
    for(tasks=0; tasks<num_tasks; tasks++) {
        if((func_count[tasks] = fork()) < 0)
        {
            perror("fork");
            exit(EXIT_FAILURE);
        }
        else if (func_count[tasks] == 0)
        {
            (*func_ptr[tasks])();
            exit(EXIT_SUCCESS);
        }
    }

    while(tasks > 0)
    {
        tasks--;
    }

    uart_deinit();
    return EXIT_SUCCESS;
}