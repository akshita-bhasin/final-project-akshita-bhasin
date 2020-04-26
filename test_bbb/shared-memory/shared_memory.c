/* Author: Akshita Bhasin */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <fcntl.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <errno.h>
#include <semaphore.h> 
#include <sys/wait.h> 
#include "syslog.h"

#define SENSOR_SHMEM_DEF         ("/shm_1")
#define SENSOR_SHMEM_PROD_COUNT  (1)

typedef struct {
     uint8_t sensor; 
     uint8_t value;
} sensor_shmem;

char *task_1_sem = "task_1_sem";
char *task_2_sem = "task_2_sem";
char *task_3_sem = "task_3_sem";

int task1(void) 
{
	sem_t* task1_sem;
	sensor_shmem task1 = {1,10};
	sensor_shmem *task1_ptr = &task1;
	int shm_fd;
	sensor_shmem *temp_ptr = NULL; 

	shm_fd = shm_open(SENSOR_SHMEM_DEF, O_RDWR, 0666);

	temp_ptr = (sensor_shmem *)mmap(NULL, sizeof(sensor_shmem), PROT_WRITE, MAP_SHARED, shm_fd, 0); 

	task1_sem = sem_open(task_1_sem,0,0666,0);
	memcpy((void *)(&temp_ptr[0]),(void*)task1_ptr,sizeof(sensor_shmem));
	sem_close(task1_sem);
	close(shm_fd);

	return 0; 
} 

int task2(void)
{
    sem_t* task2_sem;
	sensor_shmem task2 = {2,30};
	sensor_shmem *task2_ptr = &task2;
	int shm_fd;
	sensor_shmem *temp_ptr = NULL; 

	shm_fd = shm_open(SENSOR_SHMEM_DEF, O_RDWR, 0666);

	temp_ptr = (sensor_shmem *)mmap(NULL, sizeof(sensor_shmem), PROT_WRITE, MAP_SHARED, shm_fd, 0); 

	task2_sem = sem_open(task_2_sem,0,0666,0);
	memcpy((void *)(&temp_ptr[1]),(void*)task2_ptr,sizeof(sensor_shmem));
	sem_close(task2_sem);
	close(shm_fd);

	return 0; 
}

int task3(void)
{
    sem_t* task3_sem;
	sensor_shmem task3;
	sensor_shmem *task3_ptr = &task3;
	int shm_fd;
	sensor_shmem *temp_ptr = NULL; 

	shm_fd = shm_open(SENSOR_SHMEM_DEF, O_RDWR, 0666);

	temp_ptr = (sensor_shmem *)mmap(NULL, sizeof(sensor_shmem), PROT_WRITE, MAP_SHARED, shm_fd, 0); 

	task3_sem = sem_open(task_3_sem,0,0666,0);
    sem_post(task3_sem);
    sem_wait(task3_sem);
	memcpy((void *)(task3_ptr),(void*)(&temp_ptr[0]),sizeof(sensor_shmem));
    memcpy((void *)(task3_ptr),(void*)(&temp_ptr[1]),sizeof(sensor_shmem));
    sem_post(task3_sem);

    printf("Task 1 data: %d, %d", temp_ptr[0].sensor, temp_ptr[0].value);
    printf("Task 2 data: %d, %d", temp_ptr[1].sensor, temp_ptr[1].value);
	sem_close(task3_sem);
	close(shm_fd);

	munmap(temp_ptr,sizeof(sensor_shmem));
	return 0;
}

int main(void)
{
	sem_t *main_sem;
	pid_t fork_id = 0;
	int status = 0;

	main_sem = sem_open(task_1_sem, O_CREAT, 0600, 0);
	sem_close(main_sem);	
	main_sem = sem_open(task_2_sem, O_CREAT, 0600, 0);
	sem_close(main_sem);	
	main_sem = sem_open(task_3_sem, O_CREAT, 0600, 0);
	sem_close(main_sem);
	int shm_fd1 = shm_open(SENSOR_SHMEM_DEF,O_CREAT | O_RDWR, 0666);
	if(shm_fd1 < 0)
	{ 
		printf("open\n"); 
	}

	ftruncate(shm_fd1, 4096);
	
	close(shm_fd1);

	task1();
	fork_id = fork();
	wait(&status);	

	if(fork_id < 0)
	{
		exit(1);
	}

	if(fork_id > 0)
	{
		exit(0);
	}
	
	setsid();

	chdir("/");

	task2();
	task3();
	
	sem_unlink(task_1_sem);
	sem_unlink(task_2_sem);
	sem_unlink(task_3_sem);

	shm_unlink(SENSOR_SHMEM_DEF);

    return 0;
}