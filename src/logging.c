#include "../inc/logging.h"

 
int fd; //file descriptor for opening file
const char *writefile = "/var/tmp/serverlog";

int log_setup(void)
{

  fd = open(writefile,O_CREAT| O_RDONLY| O_WRONLY | O_APPEND,0755);
  if (fd == -1) // checks for error
  {
    syslog(LOG_ERR, "File could not be created\n");
    return -1;
  }
  return 0; //success
}

void log_write(const char *text)
{
  ssize_t nr;
  nr = write(fd,text,strlen(text));
  if(nr == -1)  // check for error
  {
    syslog(LOG_ERR, "Write Failed\n");
  }
  else if(nr != strlen(text))  // check for partial write condition
  {
    syslog(LOG_ERR, "Partial write occurred\n");
  }
}



void log_complete(void)
{
    if(close(fd) == -1)
    {
        syslog(LOG_ERR,"Failed to close the file\n");
    }
}


