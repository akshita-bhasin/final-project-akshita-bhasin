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
#include <semaphore.h>
#include <sys/mman.h>
#include "inc/led.h"

#define SLAVE_ADDR 0x48

#undef PDEBUG             /* undef it, just in case */
#ifdef AESD_DEBUG
#  ifdef __KERNEL__
     /* This one if debugging is on, and kernel space */
#    define PDEBUG(fmt, args...) printk( KERN_DEBUG "aesdchar: " fmt, ## args)
#  else
     /* This one for user space */
#    define PDEBUG(fmt, args...) fprintf(stderr, fmt, ## args)
#  endif
#else
#  define PDEBUG(fmt, args...) /* not debugging: nothing */
#endif

#define SENSOR_SHMEM_DEF         ("/sens_shmem")
#define SENSOR_SHMEM_PROD_COUNT  (1)

int tmp102_fd1;
int i2c_fd1;
int uart_fd1;

typedef struct {
     uint8_t sensor; 
     uint8_t value;
} sensor_shmem;

char* tmp_sem_name = "tmp102_sem";