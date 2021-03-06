/*
* Author: Akshita Bhasin
* Brief: main program for aesd project
*
* References : https://www.geeksforgeeks.org/posix-shared-memory-api/
*             http://www.cse.psu.edu/~deh25/cmpsc473/notes/OSC/Processes/shm-posix-producer-orig.c
*             http://www.cse.psu.edu/~deh25/cmpsc473/notes/OSC/Processes/shm-posix-consumer.c
*             http://man7.org/training/download/posix_shm_slides.pdf
*             http://logan.tw/posts/2018/01/07/posix-shared-memory/
*             http://www.cse.cuhk.edu.hk/~ericlo/teaching/os/lab/7-IPC2/sync-pro.html
*             
*/

#include "inc/env_mon.h"

#define TASKS 5 //changed here

int tmp_count = 0, lux_count = 0;
void signal_handler(int signum)
{
    if((signum == SIGINT) || (signum == SIGTERM))
    {
        signal_set = 1;
        syslog(LOG_INFO,"Caught signal,exiting");
        shutdown(sockfd,SHUT_RDWR);
    }
    else 
    {
        exit(EXIT_FAILURE);
    }
  assert(0 == close(tmp102_fd1));
  exit(signum);
}


/*
* Function name : tmp102_init
* Parameters    : void
* Return Type   : void
* @Brief        : initialize the Temperature sensor on I2C-2 Bus of BeagleBone 
*/
void tmp102_init(void)
{
  char *bus = "/dev/i2c-2"; /* Pins P9_19 and P9_20 */
  int addr = SLAVE_ADDR;          /* The I2C address of TMP102 */
  
  if ((tmp102_fd1 = open(bus, O_RDWR)) < 0) {
    /* ERROR HANDLING: you can check errno to see what went wrong */
    syslog(LOG_ERR,"Failed to open the i2c bus");
    exit(1);
  }

 if (ioctl(tmp102_fd1, I2C_SLAVE, addr) < 0) {
   syslog(LOG_ERR,"Failed to acquire bus access and/or talk to slave.\n");
   /* ERROR HANDLING; you can check errno to see what went wrong */
   exit(1);
 }

 /* Register the signal handler */
 signal(SIGINT, signal_handler);

}



/*
* Function name : uart_init
* Parameters    : void
* Return Type   : void
* @Brief        : initialize the UART on BeagleBone 
*/
void uart_init(void)
{
    struct termios options;

    if ((uart_fd1 = open("/dev/ttyO1", O_RDWR | O_NOCTTY | O_NDELAY)) < 0)
    {
        syslog(LOG_ERR,"open\n");
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


/*
* Function name : uart_deinit
* Parameters    : void
* Return Type   : void
* @Brief        : close the UART file descriptor 
*/
void uart_deinit(void)
{
    close(uart_fd1);
}



/*
* Function name : actuator_init
* Parameters    : void
* Return Type   : void
* @Brief        : set the GPIO pins for actuation of LED and Buzzer on the Beagle Bone 
*/
void actuator_init(void)
{
    int ret = 0;

    if((ret = gpio_export(LED)) != 0)
    {
        syslog(LOG_ERR,"gpio_export");
        exit(1);
    }
    if((ret = gpio_export(BUZ)) != 0)
    {
        syslog(LOG_ERR,"gpio_export");
        exit(1);
    }

    if((ret = gpio_set_dir(LED, GPIO_DIR_OUTPUT)) != 0)
    {
        syslog(LOG_ERR,"gpio_set_dir");
        exit(1);
    }
    if((ret = gpio_set_dir(BUZ, GPIO_DIR_OUTPUT)) != 0)
    {
        syslog(LOG_ERR,"gpio_set_dir");
        exit(1);
    }
}


/*
* Function name : actuator_deinit
* Parameters    : void
* Return Type   : void
* @Brief        : deinitialize the for actuation of LED and Buzzer on the Beagle Bone
*/

void actuator_deinit(void)
{
    int ret = 0;
    if((ret = gpio_unexport(LED)) != 0)
    {
        syslog(LOG_ERR,"gpio_unexport");
        exit(1);
    }

    if((ret = gpio_unexport(BUZ)) != 0)
    {
        syslog(LOG_ERR,"gpio_unexport");
        exit(1);
    }
}


/*
* Function name : i2c_smbus_access
* Parameters    : file descriptor, read_write operation, command, size, and data
* Return Type   : __s32 of typedef signed int
* @Brief        : function to gain access of the smbus 
* Leverage      : https://github.com/shenki/linux-i2c-example/blob/master/i2c_example.c
*/
 __s32 i2c_smbus_access(int file, char read_write, __u8 command, int size, union i2c_smbus_data *data)
 {
     struct i2c_smbus_ioctl_data args;
     
     args.read_write = read_write;
     args.command    = command;
     args.size       = size;
     args.data       = data;

     return ioctl(file,I2C_SMBUS,&args);
 }
/*
* Function name : i2c_smbus_read_byte_data
* Parameters    : file descriptor, command
* Return Type   : __s32 of typedef signed int
* @Brief        : reads a byte of data from the device connected to the smbus 
* Leverage      : https://github.com/shenki/linux-i2c-example/blob/master/i2c_example.c
*/
static inline __s32 i2c_smbus_read_byte_data(int file, __u8 command)
{
    union i2c_smbus_data data;
    if(i2c_smbus_access(file,I2C_SMBUS_READ,command,I2C_SMBUS_BYTE_DATA,&data))
        return -1;
    else
    {
        return 0x0FF & data.byte;
    }
    
}

/*
* Function name : i2c_smbus_read_word_data
* Parameters    : file descriptor, command
* Return Type   : __s32 of typedef signed int
* @Brief        : reads a word of data from the device connected to the smbus 
* Leverage      : https://github.com/shenki/linux-i2c-example/blob/master/i2c_example.c
*/
static inline __s32 i2c_smbus_read_word_data(int file, __u8 command)
{
    union i2c_smbus_data data;
    if(i2c_smbus_access(file,I2C_SMBUS_READ,command,I2C_SMBUS_WORD_DATA,&data))
        return -1;
    else
    {
        return 0x0FFFF & data.byte;
    }
}

/*
* Function name : write_single_byte
* Parameters    : file descriptor, device address, command
* Return Type   : int
* @Brief        : write a byte of data to the device 
* Leverage      : https://stackoverflow.com/questions/52975817/setup-i2c-reading-and-writing-in-c-language
*/
int write_single_byte(int file, unsigned char device_addr, int command)
{
    uint8_t outbuf;
    outbuf = command;
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg message;
    /*write command to the slave*/
    message.addr  = device_addr;
    message.flags = 0;
    message.len   = sizeof(outbuf);
    message.buf = &outbuf;
    packets.msgs = &message;
    packets.nmsgs = 1;
    if(ioctl(file,I2C_RDWR,&packets) < 0){
        syslog(LOG_ERR,"\n\rError writing single byte");
        return -1;
    }

    return 0;    
}


/*
* Function name : write_word
* Parameters    : file descriptor, device address, command
* Return Type   : int
* @Brief        : write a word size of data to the device 
* Leverage      : https://stackoverflow.com/questions/52975817/setup-i2c-reading-and-writing-in-c-language
*/
int write_word(int file, unsigned char device_addr, int* command)
{
    uint8_t outbuf[3];
    
    outbuf[0] = command[0];
    outbuf[1] = command[1];
    outbuf[2] = command[2];

    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg message;
    /*write command to the slave*/
    message.addr  = device_addr;
    message.flags = 0;
    message.len   = sizeof(outbuf);
    message.buf = outbuf;


    packets.msgs = &message;
    packets.nmsgs = 1;
    if(ioctl(file,I2C_RDWR,&packets) < 0){
        syslog(LOG_ERR,"Error writing a word");
        return -1;
    }

    return 0;    
}


/*
* Function name : ambient_init
* Parameters    : void
* Return Type   : int
* @Brief        : initialize the ambient light sensor on the I2C Bus  
*/
int ambient_init(void)
{ 
    if((i2c_fd = open(I2C_DEVICE,O_RDWR)) < 0){
        syslog(LOG_ERR,"Opening I2C file error");
        return -1;
    }

    if(ioctl(i2c_fd,I2C_SLAVE,VEML_ADDR) < 0){
        syslog(LOG_ERR,"I2C ioctl error");
        return -1;
    }
    return 0;

}



/*
* Function name : ambient_on
* Parameters    : void
* Return Type   : int
* @Brief        : power on the ambient light sensor on the I2C Bus  
*/
int ambient_on(void)
{
    uint8_t status;
    //power on the sensor
    if((status = write_word(i2c_fd,VEML_ADDR,power_on_command)) != 0) 
    {
        syslog(LOG_ERR,"Could not power on the ambient light sensor");
        return -1;
    }
    return status;
 
}


/*
* Function name : ambient_power_save_on
* Parameters    : void
* Return Type   : int
* @Brief        : switch on the power save mode on the ambient light sensor on the I2C Bus  
*/
int ambient_power_save_on(void)
{
    uint8_t status;
    if((status = write_word(i2c_fd,VEML_ADDR,psaveon_command)) != 0)
    {
        syslog(LOG_ERR,"Could not set up power saving mode");
        return -1;
    }
    return status;

}


/*
* Function name : read_values
* Parameters    : void
* Return Type   : int
* @Brief        : read values from the ambient light sensor on the I2C Bus  
*/
int read_values(void)
{
    int value,status;

    if((status = write_single_byte(i2c_fd,VEML_ADDR,read_command)) != 0)
    {
        syslog(LOG_ERR,"Phase 1 write for read failed");
        return -1;
    }
    value = i2c_smbus_read_word_data(i2c_fd,read_command);
    return value;
   
}


/*
* Function name : veml_init
* Parameters    : void
* Return Type   : void
* @Brief        : Initialize the Ambient Light Sensor  
*/
void veml_init(void)
{
    int ret;
    ret = ambient_init(); 
    if(ret == -1)
    {
        syslog(LOG_ERR,"Error in initialization");
    } 

    ret = ambient_on();
    if(ret == -1)
    {
        syslog(LOG_ERR,"Error switching on ambient");
    }

    ret = ambient_power_save_on();
    if(ret == -1)
    {
        syslog(LOG_ERR,"Failed to set power saving mode");
    }
}


/*
* Function name : tmp102_task
* Parameters    : void
* Return Type   : void
* @Brief        : Producer 1 for shared memory 1   
*/

void tmp102_task(void)
{
    syslog(LOG_INFO, "In TMP102 task\n");

    char buf[2] = {0};
    int temp;
    unsigned char MSB, LSB;

    float f, c;
    // Using I2C Readgit@github.com:cu-ecen-5013/final-project-Gitanjali-Suresh.git
    if (read(tmp102_fd1,buf,2) != 2) {
        /* ERROR HANDLING: i2c transaction failed */
        syslog(LOG_ERR, "Failed to read from the i2c bus.\n");

    } else {

        MSB = buf[0];
        LSB = buf[1];

        /* Convert 12bit int using two's compliment */
        /* Credit: http://bildr.org/2011/01/tmp102-arduino/ */
        temp = ((MSB << 8) | LSB) >> 4;

        c = temp*0.0625;
        f = (1.8 * c) + 32;

        syslog(LOG_INFO, "Temp Fahrenheit: %f Celsius: %f\n", f, c);
    }

    int shm_1_fd;
    sem_t *temperature_sem;
    sensor_shmem share_mem_temp = {1, c};
    sensor_shmem *share_mem_temp_ptr = &share_mem_temp;
    sensor_shmem *share_mem_ptr = NULL;

    if((shm_1_fd = shm_open(SENSOR_SHMEM_DEF,O_RDWR, 0666)) < 0)
    {
        syslog(LOG_ERR, "SHM open");
        exit(1);
    }

    if((share_mem_ptr = (sensor_shmem *)mmap(NULL, SENSOR_SHMEM_PROD_COUNT * sizeof(sensor_shmem), PROT_READ|PROT_WRITE, MAP_SHARED, shm_1_fd, 0)) < 0)
    {
        syslog(LOG_ERR, "mmap");
        exit(1);
    }

    if((temperature_sem = sem_open(tmp_sem_name, 0, 0666, 0)) < 0)
    {
        syslog(LOG_ERR,"sem_open");
        exit(1);
    }


    memcpy((void*)(&share_mem_ptr[0]), (void*)share_mem_temp_ptr, sizeof(sensor_shmem));

    /* For sock_task */
    tmp_buffer[tmp_count++] = share_mem_ptr[0].value;
    if(tmp_count == 5)
        tmp_count = 0;

    sem_post(temperature_sem);

    if(munmap(share_mem_ptr, SENSOR_SHMEM_PROD_COUNT * sizeof(sensor_shmem)) < 0)
    {
        syslog(LOG_ERR, "munmap");
        exit(1);
    }

    sem_close(temperature_sem);

    if(close(shm_1_fd) < 0)
    {
        syslog(LOG_ERR, "close");
        exit(1);
    }
}


/*
* Function name : ambient_task
* Parameters    : void
* Return Type   : void
* @Brief        : Producer 2 for shared memory 1   
*/
void ambient_task(void)
{
    syslog(LOG_INFO, "In LUX Task");

    int shm_1_fd;
    sem_t *ambient_sem;
    sensor_shmem share_mem_veml = {0, 0};
    sensor_shmem *share_mem_veml_ptr = &share_mem_veml;
    sensor_shmem *share_mem_ptr = NULL;
    int sensor;

    if((shm_1_fd = shm_open(SENSOR_SHMEM_DEF,O_RDWR, 0666)) < 0)
    {
        syslog(LOG_ERR, "SHM open");
        exit(1);
    }

    if((share_mem_ptr = (sensor_shmem *)mmap(NULL, SENSOR_SHMEM_PROD_COUNT * sizeof(sensor_shmem), PROT_READ|PROT_WRITE, MAP_SHARED, shm_1_fd, 0)) < 0)
    {
        syslog(LOG_ERR,"mmap");
        exit(1);
    }

    if((ambient_sem = sem_open(amb_sem_name, 0, 0666, 0)) < 0)
    {
        syslog(LOG_ERR,"sem_open");
        exit(1);
    }

    // while(1)
// {
    sensor = read_values();

    share_mem_veml_ptr->sensor = 2;
    share_mem_veml_ptr->value = sensor;

    memcpy((void*)(&share_mem_ptr[1]), (void*)share_mem_veml_ptr, sizeof(sensor_shmem));
    
    lux_buffer[lux_count++] = share_mem_ptr[1].value;
    if(lux_count == 5)
        lux_count = 0;

    sem_post(ambient_sem);

    syslog(LOG_INFO ,"Sensor values: %d", sensor);

    sleep(2);
    
    if(munmap(share_mem_ptr, SENSOR_SHMEM_PROD_COUNT * sizeof(sensor_shmem)) < 0)
    {
        syslog(LOG_ERR,"munmap");
    }

    sem_close(ambient_sem);

    if(close(shm_1_fd) < 0)
    {
        syslog(LOG_ERR,"close");
        exit(1);
    }
}

/*
* Function name : tx_uart
* Parameters    : void
* Return Type   : void
* @Brief        : Consumer for shared memory 1   
*/
void tx_uart(void)
{
    syslog(LOG_INFO ,"In UART Tx task\n");
    int shm_1_fd;
    sem_t *temperature_sem, *ambient_sem;
    sensor_shmem shmem_tx;
    sensor_shmem *shmem_tx_ptr = &shmem_tx;
    sensor_shmem * share_mem_ptr= NULL;
    int ret=1, count;

    if((shm_1_fd = shm_open(SENSOR_SHMEM_DEF, O_RDWR, 0666)) < 0)
    {
        syslog(LOG_ERR,"shm_open");
        exit(1);
    }

    if((share_mem_ptr = (sensor_shmem *)mmap(NULL, sizeof(sensor_shmem), PROT_READ, MAP_SHARED, shm_1_fd, 0)) < 0)
    {
        syslog(LOG_ERR,"mmap");
        exit(1);
    }

    temperature_sem = sem_open(tmp_sem_name, 0, 0600, 0);
    ambient_sem = sem_open(amb_sem_name, 0, 0600, 0);

    while(ret!=0)
    {
        ret = sem_wait(temperature_sem);
        
        if (ret == 0)
        {
            memcpy((void*)shmem_tx_ptr, (void*)(&share_mem_ptr[0]), sizeof(sensor_shmem));
            syslog(LOG_INFO ,"Temperature Sensor = %d\n", shmem_tx.sensor);
            syslog(LOG_INFO ,"Temperature Sensor value = %d\n", shmem_tx.value);
            if((count = write(uart_fd1, shmem_tx_ptr, sizeof(sensor_shmem))) < 0)
            {
                syslog(LOG_ERR,"write");
                exit(1);
            }
        }

        if(sem_wait(ambient_sem) == 0)
        {
            memcpy((void*)shmem_tx_ptr, (void*)(&share_mem_ptr[1]), sizeof(sensor_shmem));
            syslog(LOG_INFO ,"Ambient Sensor = %d\n", shmem_tx.sensor);
            syslog(LOG_INFO ,"Ambient Sensor value = %d\n", shmem_tx.value);
            if((count = write(uart_fd1, shmem_tx_ptr, sizeof(sensor_shmem))) < 0)
            {
                syslog(LOG_ERR,"write");
                exit(1);
            }
        }
        sem_post(temperature_sem);

    }

    if(close(shm_1_fd) < 0)
    {
        syslog(LOG_ERR,"close");
        exit(1);
    }

    if(munmap(share_mem_ptr, SENSOR_SHMEM_PROD_COUNT * sizeof(sensor_shmem)) < 0)
    {
        syslog(LOG_ERR,"munmap");
        exit(1);
    }

    sem_close(temperature_sem);
    sem_close(ambient_sem);
}

/*
* Function name : rx_uart
* Parameters    : void
* Return Type   : void
* @Brief        : Producer for shared memory 2   
*/
void rx_uart(void)
{
    syslog(LOG_INFO ,"In UART Rx task\n");
    int shm_2_fd;
    sem_t *actuator_sem;
    actuator_shmem shmem_rx;
    actuator_shmem *shmem_rx_ptr = &shmem_rx;
    actuator_shmem * share_mem_ptr= NULL;
    int ret, count=1, print_act;

    if((shm_2_fd = shm_open(ACTUATOR_SHMEM_DEF, O_RDWR, 0)) < 0)
    {
        syslog(LOG_ERR,"shm_open");
        exit(1);
    }

    if((share_mem_ptr = (actuator_shmem *)mmap(NULL, sizeof(actuator_shmem), PROT_READ, MAP_SHARED, shm_2_fd, 0)) < 0)
    {
        syslog(LOG_ERR,"mmap");
        exit(1);
    }

    actuator_sem = sem_open(act_sem_name, 0, 0600, 0);

    sem_post(actuator_sem);

    while(count!=0)
    {
        count = 0;
        ret = sem_wait(actuator_sem);
        
        if (ret == 0)
        {
            fcntl(uart_fd1, F_SETFL, 0);

            syslog(LOG_INFO ,"Receive characters\n");

            if((count = read(uart_fd1, &(shmem_rx), sizeof(actuator_shmem))) < 0)
            {
                syslog(LOG_ERR,"read\n");
                exit(1);
            }

            print_act = count;
            
            while(print_act > 0)
            {

                // memcpy((void*)shmem_rx_ptr, (void*)(&share_mem_ptr[0]), sizeof(actuator_shmem));

                if(shmem_rx_ptr[count-print_act].actuator == 2)
                {
                    syslog(LOG_INFO ,"LED state\n");
                    if((ret = gpio_set_value(LED, shmem_rx_ptr[count-print_act].value)) != 0)
                    {
                        syslog(LOG_ERR,"gpio_set_value");
                        exit(1);
                    }
                }
                else if(shmem_rx_ptr[count-print_act].actuator == 1)
                {
                    int i;
                    if(shmem_rx_ptr[count-print_act].value == 1)
                    {
                        for(i =0; i<100; i++)
                        {
                            if((ret = gpio_set_value(BUZ, 1)) != 0)
                            {
                                syslog(LOG_ERR,"gpio_set_ON_value");
                                exit(1);
                            }
                            usleep(1000);
                            if((ret = gpio_set_value(BUZ, 0)) != 0)
                            {
                                syslog(LOG_ERR,"gpio_set_OFF_value");
                                exit(1);
                            }
                            usleep(1000);
                        }
                    }
                }
                print_act--;
            }
        }
        sem_post(actuator_sem);
    }

    if(munmap(share_mem_ptr, sizeof(actuator_shmem)) < 0)
    {
        syslog(LOG_ERR,"munmap");
        exit(1);
    }

    if(close(shm_2_fd) < 0)
    {
        syslog(LOG_ERR,"close");
        exit(1);
    }

    syslog(LOG_INFO ,"Testing task working\n");
    sem_close(actuator_sem);
}

/*
* Function name : sock_init
* Parameters    : void
* Return Type   : int
* @Brief        : Initialize the socket  
*/
int sock_init(void)
{
    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_flags = AI_PASSIVE; // use my IP
    hints.ai_protocol = 0;

    //signal handling
    if(signal(SIGINT,signal_handler) == SIG_ERR)
    {
        syslog(LOG_ERR,"sigaction");
        exit(1);
    }
    
    

    if(signal(SIGTERM,signal_handler) == SIG_ERR)
    {
        syslog(LOG_ERR,"sigaction");
        exit(1);    
    }

    if ((addr_status = getaddrinfo(NULL, PORT, &hints, &res)) != 0) 
    {
        syslog(LOG_DEBUG,"getaddrinfo");
        return -1;
    }
    
    // loop through all the resuls and bind to the first we can
    if ((sockfd = socket(res->ai_family, res->ai_socktype, res->ai_protocol)) == -1) 
    {
        syslog(LOG_DEBUG,"socket");
        return -1;
    }

    if((setsockfd = setsockopt(sockfd,SOL_SOCKET,SO_REUSEADDR,&option,sizeof(option))) == -1)
    {
        syslog(LOG_DEBUG,"setsockopt");
        return -1;
    }

    if ((bindfd = bind(sockfd, res->ai_addr, res->ai_addrlen)) == -1) 
    {
       syslog(LOG_DEBUG,"bind");
       return -1;
    }
    
    if((listenfd = listen(sockfd, BACKLOG)) == -1){
        syslog(LOG_DEBUG,"listen");
        return -1;   
    }
    return 0;
}

/*
* Function name : sock_task
* Parameters    : void
* Return Type   : int
* @Brief        : Task for Socket Communication   
*/
int sock_task(void)
{
    int i;
    addr_size = sizeof their_addr;
    new_fd = accept(sockfd, (struct sockaddr *)&their_addr, &addr_size); //accept an incoming connection
    if(new_fd == -1)
    {
        syslog(LOG_DEBUG,"accept");
        return -1;   
    }

    //ip_address
    syslog(LOG_INFO,"Accepted Connection from %s", inet_ntoa(their_addr.sin_addr));
    char logstring[43];
    
	//https://www.geeksforgeeks.org/time-h-header-file-in-c-with-examples/
	struct tm* ptr;
	time_t t;
	t = time(NULL);
	ptr = localtime(&t);
	char *timestamp = asctime(ptr);
    for(i = 0; i < 5; i++)
    {
        sprintf(logstring,"Temp : %u Lux : %u %s",tmp_buffer[i],lux_buffer[i],timestamp);
        send(new_fd,logstring,43,0);    
    }
    
    syslog(LOG_INFO,"Closed Connection from %s", inet_ntoa(their_addr.sin_addr));
    return 0;
}



/*
* Function name : main
* Parameters    : void
* Return Type   : int
* @Brief        : Provides entry control to the application   
*/
int main(void)
{
    int while_loop = 0;
    openlog(NULL, LOG_CONS | LOG_PERROR, LOG_USER);

    int connect_status = sock_init();
    if(connect_status != 0)
    {
        syslog(LOG_ERR,"Socket setup failed");
        return -1;
    }
    
    sem_t *main_sem;
	pid_t fork_id = 0;

    buff = (char *) malloc(1000);


	main_sem = sem_open(tmp_sem_name, O_CREAT, 0600, 0);
	sem_close(main_sem);
	main_sem = sem_open(amb_sem_name, O_CREAT, 0600, 0);
	sem_close(main_sem);
    main_sem = sem_open(act_sem_name, O_CREAT, 0600, 0);
	sem_close(main_sem);
	int shm_1_fd1 = shm_open(SENSOR_SHMEM_DEF,O_CREAT | O_RDWR, 0666);
	if(shm_1_fd1 < 0)
	{ 
		syslog(LOG_INFO ,"open\n"); 
	}

    int shm_2_fd1 = shm_open(ACTUATOR_SHMEM_DEF,O_CREAT | O_RDWR, 0666);
	if(shm_2_fd1 < 0)
	{ 
		syslog(LOG_INFO ,"open\n"); 
	}

    uart_init();
    tmp102_init();
    veml_init();
    actuator_init();

	ftruncate(shm_1_fd1, SENSOR_SHMEM_PROD_COUNT * sizeof(sensor_shmem));
    ftruncate(shm_2_fd1, sizeof(actuator_shmem));

	close(shm_1_fd1);
    close(shm_2_fd1);

    while(1)
    {
        tmp102_task();
        fork_id = fork();

        if(fork_id < 0)
        {
            exit(1);
        }

        if(fork_id > 0)
        {
            exit(0);
        }

        ambient_task();
        fork_id = fork();

        if(fork_id < 0)
        {
            exit(1);
        }

        if(fork_id > 0)
        {
            exit(0);
        }

        tx_uart();

        fork_id = fork();

        if(fork_id < 0)
        {
            exit(1);
        }

        if(fork_id > 0)
        {
            rx_uart();
            exit(0);
        }
        if(while_loop == 5)
        {
            sock_task();
            while_loop = 0;

            close(new_fd);
        }
        while_loop++;

    }
	
	sem_unlink(tmp_sem_name);
    sem_unlink(act_sem_name);
    sem_unlink(amb_sem_name);

	shm_unlink(SENSOR_SHMEM_DEF);
    shm_unlink(ACTUATOR_SHMEM_DEF);

    uart_deinit();
    
    

    return 0;

    
}