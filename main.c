/*
* References : https://www.geeksforgeeks.org/posix-shared-memory-api/
*             http://www.cse.psu.edu/~deh25/cmpsc473/notes/OSC/Processes/shm-posix-producer-orig.c
*             http://www.cse.psu.edu/~deh25/cmpsc473/notes/OSC/Processes/shm-posix-consumer.c
*             http://man7.org/training/download/posix_shm_slides.pdf
*             http://logan.tw/posts/2018/01/07/posix-shared-memory/
*             
*/

#include "env_mon.h"

#define TASKS 4

int shared_memory_check(void);

void signal_handler(int signum)
{
  assert(0 == close(tmp102_fd1));
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

void actuator_init(void)
{
    int ret = 0;

    if((ret = gpio_export(LED)) != 0)
    {
        perror("gpio_export");
        exit(1);
    }
    if((ret = gpio_export(BUZ)) != 0)
    {
        perror("gpio_export");
        exit(1);
    }

    if((ret = gpio_set_dir(LED, GPIO_DIR_OUTPUT)) != 0)
    {
        perror("gpio_set_dir");
        exit(1);
    }
    if((ret = gpio_set_dir(BUZ, GPIO_DIR_OUTPUT)) != 0)
    {
        perror("gpio_set_dir");
        exit(1);
    }
}

void actuator_deinit(void)
{
    int ret = 0;
    if((ret = gpio_unexport(LED)) != 0)
    {
        perror("gpio_unexport");
        exit(1);
    }

    if((ret = gpio_unexport(BUZ)) != 0)
    {
        perror("gpio_unexport");
        exit(1);
    }
}

/* Producer 1 for shared memory 1 */
void tmp102_task(void)
{
    printf("In TMP102 task\n");

    char buf[2] = {0};
    int temp;
    unsigned char MSB, LSB;

    float f, c;
    // Using I2C Readgit@github.com:cu-ecen-5013/final-project-Gitanjali-Suresh.git
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

    int shm_1_fd;
    sem_t *temperature_sem;
    sensor_shmem share_mem_temp = {1, c};
    sensor_shmem *share_mem_temp_ptr = &share_mem_temp;
    sensor_shmem *share_mem_ptr = NULL;

    if((shm_1_fd = shm_open(SENSOR_SHMEM_DEF,O_RDWR, 0666)) < 0)
    {
        perror("SHM open");
        exit(1);
    }

    if((share_mem_ptr = (sensor_shmem *)mmap(NULL, sizeof(sensor_shmem), PROT_READ|PROT_WRITE, MAP_SHARED, shm_1_fd, 0)) < 0)
    {
        perror("mmap");
        exit(1);
    }

    if((temperature_sem = sem_open(tmp_sem_name, 0, 0666, 0)) < 0)
    {
        perror("sem_open");
        exit(1);
    }

    memcpy((void*)(&share_mem_ptr[0]), (void*)share_mem_temp_ptr, sizeof(sensor_shmem));

    sem_post(temperature_sem);

    if(munmap(share_mem_ptr, sizeof(sensor_shmem)) < 0)
    {
        perror("munmap");
        exit(1);
    }

    sem_close(temperature_sem);

    if(close(shm_1_fd) < 0)
    {
        perror("close");
        exit(1);
    }
}

/*Consumer for shared memory 1*/
void tx_uart(void)
{
    printf("In UART Tx task\n");
    int shm_1_fd;
    sem_t *temperature_sem;
    sensor_shmem shmem_tx;
    sensor_shmem *shmem_tx_ptr = &shmem_tx;
    sensor_shmem * share_mem_ptr= NULL;
    int ret=1, count;

    if((shm_1_fd = shm_open(SENSOR_SHMEM_DEF, O_RDWR, 0666)) < 0)
    {
        perror("shm_open");
        exit(1);
    }

    if((share_mem_ptr = (sensor_shmem *)mmap(NULL, sizeof(sensor_shmem), PROT_READ, MAP_SHARED, shm_1_fd, 0)) < 0)
    {
        perror("mmap");
        exit(1);
    }

    temperature_sem = sem_open(tmp_sem_name, 0, 0600, 0);

    while(ret!=0)
    {
        ret = sem_wait(temperature_sem);
        
        if (ret == 0)
        {
            memcpy((void*)shmem_tx_ptr, (void*)(&share_mem_ptr[0]), sizeof(sensor_shmem));
            printf("Sensor = %d\n", shmem_tx.sensor);
            printf("Sensor value = %d\n", shmem_tx.value);
            // int count;
            // char tx[20] = "Hello TM4C123GXL";
            // printf("Sending: '%s'\n", tx);
            // if ((count = write(uart_fd1, &tx, 17)) < 0)
            // {
            //     perror("write\n");
            //     exit(1);
            // }
            if((count = write(uart_fd1, shmem_tx_ptr, sizeof(sensor_shmem))) < 0)
            {
                perror("write");
                exit(1);
            }
        }
        sem_post(temperature_sem);

        /* Wait for humidty and add sleep */
    }

    if(close(shm_1_fd) < 0)
    {
        perror("close");
        exit(1);
    }

    if(munmap(share_mem_ptr, sizeof(sensor_shmem)) < 0)
    {
        perror("munmap");
        exit(1);
    }

    sem_close(temperature_sem);
}

/* Producer for shared memory 2*/
void rx_uart(void)
{
    printf("In UART Rx task\n");
    int shm_2_fd;
    sem_t *actuator_sem;
    actuator_shmem shmem_rx;
    actuator_shmem *shmem_rx_ptr = &shmem_rx;
    actuator_shmem * share_mem_ptr= NULL;
    int count=1;

    if((shm_2_fd = shm_open(ACTUATOR_SHMEM_DEF, O_RDWR, 0666)) < 0)
    {
        perror("shm_open");
        exit(1);
    }

    if((share_mem_ptr = (actuator_shmem *)mmap(NULL, sizeof(actuator_shmem), PROT_READ, MAP_SHARED, shm_2_fd, 0)) < 0)
    {
        perror("mmap");
        exit(1);
    }

    actuator_sem = sem_open(act_sem_name, 0, 0600, 0);

    // while(1)
    // {
        // ret = sem_wait(actuator_sem);
        
        // if (ret == 0)
        // {
            fcntl(uart_fd1, F_SETFL, 0);

            printf("Receive characters\n");

            if((count = read(uart_fd1, (void *)shmem_rx_ptr, sizeof(actuator_shmem))) < 0)
            {
                perror("read\n");
                exit(1);
            }

            printf("Actuator = %d\n", shmem_rx.actuator);
            printf("Actuator value = %d\n", shmem_rx.value);

            memcpy((void*)shmem_rx_ptr, (void*)(&share_mem_ptr[0]), sizeof(actuator_shmem));
            printf("Test if it reaches here");
            sem_post(actuator_sem);
        // }

        /* Wait for humidty and add sleep */
    // }

    if(close(shm_2_fd) < 0)
    {
        perror("close");
        exit(1);
    }

    if(munmap(share_mem_ptr, sizeof(actuator_shmem)) < 0)
    {
        perror("munmap");
        exit(1);
    }

    printf("Testing task working");
    sem_close(actuator_sem);
}

/* Consumer for shared memory 2*/
void actuator_task(void)
{
    printf("In Actuator Task");

    int shm_2_fd, ret;
    sem_t *actuator_sem;
    actuator_shmem share_mem_act;
    actuator_shmem *share_mem_act_ptr = &share_mem_act;
    actuator_shmem *share_mem_ptr = NULL;

    if((shm_2_fd = shm_open(ACTUATOR_SHMEM_DEF,O_RDWR, 0666)) < 0)
    {
        perror("SHM open");
        exit(1);
    }

    if((share_mem_ptr = (actuator_shmem *)mmap(NULL, sizeof(actuator_shmem), PROT_READ, MAP_SHARED, shm_2_fd, 0)) < 0)
    {
        perror("mmap");
        exit(1);
    }

    if((actuator_sem = sem_open(act_sem_name, 0, 0666, 0)) < 0)
    {
        perror("sem_open");
        exit(1);
    }

    while(1)
    {
        sem_wait(actuator_sem);
        memcpy((void*)(share_mem_ptr), (void*)&(share_mem_act_ptr[0]), sizeof(actuator_shmem));
        printf("Acutator = %d\n", share_mem_act.actuator);
        printf("Value = %d\n", share_mem_act.value);
        if(share_mem_act.actuator == 0)
        {
            if((ret = gpio_set_value(LED, share_mem_act.value)) != 0)
            {
                perror("gpio_set_value");
                exit(1);
            }
        }
        else if(share_mem_act.actuator == 1)
        {
            if((ret = gpio_set_value(BUZ, share_mem_act.value)) != 0)
            {
                perror("gpio_set_value");
                exit(1);
            }
        }
        // usleep(1000000);
        
        // if((ret = gpio_set_value(LED, 0)) != 0)
        // {
        //     perror("gpio_set_OFF_value");
        //     exit(1);
        // }
        // if((ret = gpio_set_value(BUZ, 0)) != 0)
        // {
        //     perror("gpio_set_OFF_value");
        //     exit(1);
        // }

        // usleep(100000);
        sem_post(actuator_sem);
    }

    if(munmap(share_mem_ptr, sizeof(actuator_shmem)) < 0)
    {
        perror("munmap");
        exit(1);
    }

    sem_close(actuator_sem);

    if(close(shm_2_fd) < 0)
    {
        perror("close");
        exit(1);
    }
    // int i, ret = 0;
    
    // for(i=0;i<10;i++)
    // {
    //     if((ret = gpio_set_value(LED, 1)) != 0)
    //     {
    //         perror("gpio_set_ON_value");
    //         exit(1);
    //     }
    //     if((ret = gpio_set_value(BUZ, 1)) != 0)
    //     {
    //         perror("gpio_set_ON_value");
    //         exit(1);
    //     }

    //     usleep(1000000);
        
    //     if((ret = gpio_set_value(LED, 0)) != 0)
    //     {
    //         perror("gpio_set_OFF_value");
    //         exit(1);
    //     }
    //     if((ret = gpio_set_value(BUZ, 0)) != 0)
    //     {
    //         perror("gpio_set_OFF_value");
    //         exit(1);
    //     }

    //     usleep(100000);
    //}

}

void task_test(void)
{
    printf("Test task");
}

int main(void)
{
    sem_t *main_sem;
	pid_t fork_id = 0;

	main_sem = sem_open(tmp_sem_name, O_CREAT, 0600, 0);
	sem_close(main_sem);
    main_sem = sem_open(rx_sem_name, O_CREAT, 0600, 0);
	sem_close(main_sem);
    main_sem = sem_open(act_sem_name, O_CREAT, 0600, 0);
	sem_close(main_sem);
	int shm_1_fd1 = shm_open(SENSOR_SHMEM_DEF,O_CREAT | O_RDWR, 0666);
	if(shm_1_fd1 < 0)
	{ 
		printf("open\n"); 
	}

    int shm_2_fd1 = shm_open(ACTUATOR_SHMEM_DEF,O_CREAT | O_RDWR, 0666);
	if(shm_2_fd1 < 0)
	{ 
		printf("open\n"); 
	}

    uart_init();
    tmp102_init();
    actuator_init();

	ftruncate(shm_1_fd1, sizeof(sensor_shmem));

    ftruncate(shm_2_fd1, sizeof(actuator_shmem));

	close(shm_1_fd1);
    close(shm_2_fd1);

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

	tx_uart();

    fork_id = fork();

	if(fork_id < 0)
	{
		exit(1);
	}

	if(fork_id > 0)
	{
		exit(0);
	}

	rx_uart();

    printf("Outside rx_uart");
    fork_id = fork();
    printf("fork_id after rx_uart: %d", fork_id);

	if(fork_id < 0)
	{
		exit(1);
	}

	if(fork_id == 0)
	{
		exit(0);
	}

    actuator_task();

    // fork_id = fork();

	// if(fork_id < 0)
	// {
	// 	exit(1);
	// }

	// if(fork_id > 0)
	// {
	// 	exit(0);
	// }

    // task_test();
	
	sem_unlink(tmp_sem_name);
    sem_unlink(rx_sem_name);
    sem_unlink(act_sem_name);

	shm_unlink(SENSOR_SHMEM_DEF);
    shm_unlink(ACTUATOR_SHMEM_DEF);

    uart_deinit();
    actuator_deinit();

    return 0;

    // pid_t func_count[TASKS];
    // int tasks, num_tasks = TASKS;
    // sem_t *main_sem;
    // int shmem_1_fd;
    // void(*func_ptr[])(void) = { tmp102_task,
    //                             tx_uart };
    //                             // rx_uart
    //                             // actuator_task };

    // uart_init();
    // tmp102_init();
    // actuator_init();

    // if((shmem_1_fd = shm_open(SENSOR_SHMEM_DEF, O_CREAT | O_RDWR, 0600)) < 0)
    // {
    //     perror("shm_open");
    //     return -1;
    // }

    // ftruncate(shmem_1_fd, sizeof(sensor_shmem));

    // if(close(shmem_1_fd) < 0)
    // {
    //     perror("close");
    //     return -1;
    // }

    // if((main_sem = sem_open(tmp_sem_name, O_CREAT, 0600, 0)) < 0)
    // {
    //     perror("sem_open");
    //     return -1;
    // }

    // sem_close(main_sem);

    // for(tasks=0; tasks<num_tasks; tasks++) {
    //     if((func_count[tasks] = fork()) < 0)
    //     {
    //         perror("fork");
    //         exit(EXIT_FAILURE);
    //     }
    //     else if (func_count[tasks] == 0)
    //     {
    //         (*func_ptr[tasks])();
    //         exit(EXIT_SUCCESS);
    //     }
    // }

    // while(tasks > 0)
    // {
    //     tasks--;
    // }

    // uart_deinit();

    // if(sem_unlink(tmp_sem_name) < 0)
    // {
    //     perror("sem_unlink");
    //     return -1;
    // }
    
    // if(shm_unlink(SENSOR_SHMEM_DEF) < 0)
    // {
    //     perror("shm_unlink");
    //     return -1;
    // }

    // return 0;
}