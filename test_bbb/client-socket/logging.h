#ifndef LOGGING_H
#define LOGGING_H

//includes
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <syslog.h>


int log_setup(void);
void log_write(const char *text);
void log_complete(void);


#endif