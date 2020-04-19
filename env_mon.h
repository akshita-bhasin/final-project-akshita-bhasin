#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <linux/fs.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <signal.h>
#include <stdint.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <syslog.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <assert.h>
#include "inc/led.h"

#define SLAVE_ADDR 0x48

int tmp102_fd1;
int i2c_fd1;
int uart_fd1;