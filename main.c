//Referred to mohit Rane repository to structure main.c

#include "env_mon.h"

#define TASKS 2

void uart_init(void)
{
    struct termios options;

    printf("Testing uart implementation");

    if ((uart_fd1 = open("/dev/ttyO1", O_RDWR | O_NOCTTY | O_NDELAY)) < 0)
    {
        perror("open\n");
        exit(1);
    }

    tcgetattr(uart_fd1, &options);
    if((cfsetispeed(&options, B115200)) == -1)
    {
        perror("Input baud rate\n");
        exit(1);
    }
    if((cfsetospeed(&options, B115200)) == -1)
    {
        perror("Output baud rate\n");
        exit(1);
    } 

    options.c_cflag |= (CLOCAL | CS8);
    options.c_iflag &= ~(ISTRIP | IXON | INLCR | PARMRK | ICRNL | IGNBRK);
    options.c_oflag = 0;
    options.c_lflag = 0;

    tcsetattr(uart_fd1, TCSAFLUSH, &options);
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
    char rx[100];
    fcntl(uart_fd1, F_SETFL, 0);

    printf("Receive characters\n");

    if ((count = read(uart_fd1, (void*)rx, 100)) < 0)
    {
        perror("read\n");
        exit(1);
    }

    if(count)
    {
        printf("Received-> %s, %d chars", rx, count);
    }
}

int main(void)
{
    pid_t func_count[TASKS];
    int tasks, num_tasks = TASKS;
    void(*func_ptr[])(void) = { tx_uart,
                                rx_uart };

    uart_init();
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