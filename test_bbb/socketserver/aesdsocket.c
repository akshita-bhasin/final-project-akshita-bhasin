/****************************************************************************************
Author : Madhukar Arora
File Name : aesdsocket.c
File Description : Opens a socket stream on port 9000, listens and waits for a 
                   connection. Can be executed in normal mode or daemon mode with a -d
                   command line argument. 
Leveraged Code : https://beej.us/guide/bgnet/html/
                 Linux System Programming, 2nd Edition - Robert Love

*****************************************************************************************/
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <stdio.h>
#include <string.h>
#include <syslog.h>
#include <stddef.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <signal.h>
#include <arpa/inet.h>
#include <netinet/in.h>


#define PORT "9000"
#define BACKLOG 10
#define BUFFER_SIZE 20
#define NR_OPEN 1024

const char *writefile = "/var/tmp/aesdsocketdata";
int new_fd, sockfd;


/**********************************************************************************
 Function Name : signal_handler
 Parameters : signo of type int
 Return Type : void
 Description : signal handler function to handle SIGINT and SIGTERM. On receiving
               a SIGINT or SIGTERM, the handler closes the new socket generated 
               during accept and removes the file to which data was being written.
 **********************************************************************************/
static void signal_handler(int signo)
{
    if((signo == SIGINT) || (signo == SIGTERM))
    {
        close(new_fd);
        remove(writefile);
        syslog(LOG_INFO,"Caught signal,exiting");
    }
    else 
    {
        fprintf(stderr,"Unexpected signal!\n");
        exit(EXIT_FAILURE);
    }
    exit(EXIT_SUCCESS);
}


/************************************************************************
Function name : main
Parameters :  takes a single optional parameter '-d' in the command line
              to run the application in daemon mode
Return type : int
Description : provides application entry control
*************************************************************************/

int main(int argc, char* argv[])
{
    openlog(NULL, LOG_CONS | LOG_PERROR, LOG_USER);

    int addr_status;
    int bindfd, listenfd, setsockfd;
    int fd, daemon_mode = 0;
    int option = 1;
    
    
    struct addrinfo hints;
    struct addrinfo *res; //point to results
    struct sockaddr_in their_addr;
    socklen_t addr_size;
    
    

    if(argc > 2) // check for more arguments
    {
        syslog(LOG_DEBUG,"Supplied more arguments \n");
    }

    

    if((argc == 2) && (strcmp(argv[1],"-d") == 0))
    {
        daemon_mode = 1; // sets flag if daemon mode '-d' is passed in command line   
    }
    

    memset(&hints, 0, sizeof hints);
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_flags = AI_PASSIVE; // use my IP
    hints.ai_protocol = 0;

    //signal handling
    if(signal(SIGINT,signal_handler) == SIG_ERR)
    {
        perror("sigaction");
        exit(1);
    }
    
    

    if(signal(SIGTERM,signal_handler) == SIG_ERR)
    {
        perror("sigaction");
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
    
    if(daemon_mode)
    {
        daemon(0, sockfd);
    }
    

    if((listenfd = listen(sockfd, BACKLOG)) == -1){
        syslog(LOG_DEBUG,"listen");
        return -1;
        
    }

    pid_t pid = getpid();
    printf("PID: %d", pid);

    while(1)
    {
        addr_size = sizeof their_addr;
        new_fd = accept(sockfd, (struct sockaddr *)&their_addr, &addr_size); //accept an incoming connection
        if(new_fd == -1)
        {
            syslog(LOG_DEBUG,"accept");
            return -1;   
        }
        //ip_address
        syslog(LOG_INFO,"Accepted Connection from %s", inet_ntoa(their_addr.sin_addr));

        int recv_check = 0;
        int read_bytes = 0; 
        char *recv_msg = (char*)malloc(BUFFER_SIZE);
        // Discussed with Atharva Nandanwar to figure out a solutiongit
        do{
            recv_check = recv(new_fd,recv_msg + read_bytes, BUFFER_SIZE, 0);
            if(recv_check == -1)
            {
                syslog(LOG_DEBUG,"Recieve message failure");
                return -1;
            }
            read_bytes += recv_check;
            if(recv_check == BUFFER_SIZE)
            {
                recv_msg = (char*)realloc(recv_msg,BUFFER_SIZE*10);
            }
        }while(*(recv_msg + read_bytes -1) != '\n');
        
        fd = open(writefile,O_CREAT| O_RDWR | O_APPEND,0755);
        if (fd == -1) // checks for error
        {
            syslog(LOG_ERR, "File could not be created\n");
            return 1;
        }
        else
        {
            write(fd,recv_msg,read_bytes);
        }
        free(recv_msg);
        off_t filesize;
        filesize = lseek(fd, 0L, SEEK_END);
        lseek(fd, 0L, SEEK_SET);
    

        char* send_msg;
        send_msg = (char*)malloc(filesize);
        
        read(fd,send_msg,filesize);
        

        send(new_fd, send_msg, filesize, 0);   // server to client    
        free(send_msg);
        syslog(LOG_INFO,"Closed Connection from %s", inet_ntoa(their_addr.sin_addr));
        memset(&recv_msg,0,strlen(recv_msg));
        close(fd);    
   }       
          
}

    
