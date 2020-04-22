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

    pid_t func_count[3];
    int tasks, num_tasks = 3;
    sem_t *main_sem;
    int shmem_1_fd;
    int(*func_ptr[])(void) = { task1,
                                task2,
                                task3 };


    if((shmem_1_fd = shm_open(SENSOR_SHMEM_DEF, O_CREAT | O_RDWR, 0600)) < 0)
    {
        perror("shm_open");
        return -1;
    }

    ftruncate(shmem_1_fd, sizeof(sensor_shmem));

    if(close(shmem_1_fd) < 0)
    {
        perror("close");
        return -1;
    }

    printf("Succesfully created shm");

    if((main_sem = sem_open(task_1_sem, O_CREAT, 0600, 0)) < 0)
    {
        perror("sem_open");
        return -1;
    }

    sem_close(main_sem);

    if((main_sem = sem_open(task_2_sem, O_CREAT, 0600, 0)) < 0)
    {
        perror("sem_open");
        return -1;
    }

    sem_close(main_sem);

    if((main_sem = sem_open(task_3_sem, O_CREAT, 0600, 0)) < 0)
    {
        perror("sem_open");
        return -1;
    }

    sem_close(main_sem);

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

    if(sem_unlink(task_1_sem) < 0)
    {
        perror("sem_unlink");
        return -1;
    }
    
    if(sem_unlink(task_2_sem) < 0)
    {
        perror("sem_unlink");
        return -1;
    }

    if(sem_unlink(task_3_sem) < 0)
    {
        perror("sem_unlink");
        return -1;
    }
    if(shm_unlink(SENSOR_SHMEM_DEF) < 0)
    {
        perror("shm_unlink");
        return -1;
    }

    return 0;
}