#include "env_mon.h"

#define TASKS 2

void uart_init(void)
{
    struct termios options;

    printf("Testing uart implementation with Tiva");

    if ((uart_fd1 = open("/dev/ttyO1", O_RDWR | O_NOCTTY | O_NDELAY)) < 0)
    {
        perror("open\n");
        return -1;
    }

    tcgetattr(uart_fd1, &options);

    options.c_cflag = B9600 | CS8 | CREAD | CLOCAL;
  	options.c_iflag = IGNPAR | ICRNL; 

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
    char tx[20] = "UART Tiva";//, rx[20];
    printf("UART Transmit Started");
    while(1)
    {
        printf("Sending char: %s\n", tx);
        if ((count = write(uart_fd1, &tx, 10)) < 0)
        {
            perror("write");
            return -1;
        }
        usleep(100000);
    }
}

void rx_uart(void)
{
    int count;
    char rx[20];
    syslog(LOG_DEBUG, "UART Receive Started");
    fcntl(uart_fd1, F_SETFL, 0);

    while(1)
    {
        printf("Receive characters\n");

        if ((count = read(uart_fd1, (void *)rx, 1)) < 0)
        {
            perror("read");
            return -1;
        }

        if(count)
        {
            printf("Received-> '%s'", rx);
        }
    }
}

int main(void)
{
    pid_t func_count[TASKS];
    int tasks, num_tasks = TASKS;
    int status;
    void(*func_ptr[])(void) = { tx_uart,
                                rx_uart };

    uart_init();
    printf("Starting Tasks\n");
    for(tasks=0; tasks<num_tasks; tasks++) {
        if((func_count[tasks] = fork()) < 0)
        {
            perror("fork");
            exit(EXIT_FAILURE);
        }
        else if (func_count[tasks] == 0)          // in child
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