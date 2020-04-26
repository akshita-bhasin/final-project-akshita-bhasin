#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <stddef.h>
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
#include <sys/shm.h>
#include <time.h>

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

#define SENSOR_SHMEM_DEF           ("/shm_1")
#define ACTUATOR_SHMEM_DEF         ("/shm_2")
#define SENSOR_SHMEM_PROD_COUNT    (2)
#define ACTUAT_SHMEM_PROD_COUNT    (1)

#define I2C_DEVICE "/dev/i2c-1"

#define VEML_ADDR 0x48
#define VEML_WRITE 0x90
#define VEML_READ 0x91

//socket

#define PORT "9000"
#define BACKLOG 10
#define BUFFER_SIZE 20
#define NR_OPEN 1024


uint8_t tmp_buffer[5];
uint8_t lux_buffer[5];

int tmp102_fd1;
int ambient_fd1;
int i2c_fd;
int uart_fd1;

typedef struct {
     uint8_t sensor; 
     uint8_t value;
} sensor_shmem;

typedef struct {
     uint8_t actuator; 
     uint8_t value;
} actuator_shmem;

char *buff = NULL;

int power_on_config = 0x800; //MSB = 0x8, LSB 0x00
int power_saving_off = 0x0000; //PSM mode 1, disabled
int  power_saving_on = 0x0001; //PSM mode 1, enabled

//register address, LSB, MSB
int  power_on_command[3] = {0x00, 0x00, 0x10}; 
int psaveoff_command[3]  = {0x03, 0x00, 0x00};
int psaveon_command[3]   = {0x03, 0x01, 0x00};
int read_command         = 0x04;

char* tmp_sem_name = "tmp102_sem";
char* act_sem_name = "actuat_sem";
char* buf_sem_name = "buff_w_sem";
char* amb_sem_name = "ambient_sem";

int new_fd, sockfd;
volatile int signal_set = 0;

int addr_status;
int bindfd, listenfd, setsockfd;
int option = 1;

struct addrinfo hints;
struct addrinfo *res; //point to results
struct sockaddr_in their_addr;
socklen_t addr_size;

// function prototypes
int write_single_byte(int file, unsigned char device_addr, int command);
int write_word(int file, unsigned char device_addr, int* command);