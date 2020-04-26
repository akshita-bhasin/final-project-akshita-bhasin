# Author: Akshita Bhasin
# Filename: Makefile
# Brief: Makefile to build and compile gcc and cross compile files

# CROSS_COMPLIE - when specified, use arm-unknown-linux-gnueabi- as the cross complier
ifeq ($(CC),)
	CC := $(CROSS_COMPILE)gcc
endif

ifeq ($(CFLAGS),)
	CFLAGS := -g -Wall -Werror
endif

ifeq ($(LDFLAGS),)
	LDFLAGS := -pthread -lrt
endif

all: environmental_monitoring shtc3 server client shared_mem ambient

environmental_monitoring: main.o led.o
	@echo "$(CC) compilation"
	@$(CC) $(CFLAGS) $(INCLUDES) main.o led.o -o environmental_monitoring $(LDFLAGS)
	@echo "Successful compilation!"

main.o: main.c inc/led.h
	@echo "$(CC) compilation"
	@$(CC) $(CFLAGS) $(INCLUDES) -c main.c $(LDFLAGS)
	@echo "Successful compilation!"

led.o: src/led.c inc/led.h
	@$(CC) $(CFLAGS) $(INCLUDES) -c src/led.c

shtc3: test_bbb/SHTC3.c
	@$(CC) $(CFLAGS) $(INCLUDES) test_bbb/SHTC3.c -o shtc3

server: test_bbb/server-socket/aesdsocket.c
	@echo "$(CC) compilation"
	@$(CC) $(CFLAGS) $(INCLUDES) test_bbb/server-socket/aesdsocket.c -o server
	@echo "Successful compilation!"

client: client.o logging.o
	@echo "$(CC) compilation"
	@$(CC) $(CFLAGS) $(INCLUDES) client.o logging.o -o client
	@echo "Successful compilation!"

client.o: test_bbb/client-socket/client.c test_bbb/client-socket/logging.h
	@echo "$(CC) compilation"
	@$(CC) $(CFLAGS) $(INCLUDES) -c test_bbb/client-socket/client.c
	@echo "Successful compilation!"

logging.o: test_bbb/client-socket/logging.c test_bbb/client-socket/logging.h
	@$(CC) $(CFLAGS) $(INCLUDES) -c test_bbb/client-socket/logging.c

shared_mem: test_bbb/shared-memory/shared_memory.c
	@echo "$(CC) compilation"
	@$(CC) $(CFLAGS) $(INCLUDES) test_bbb/shared-memory/shared_memory.c -o shared_mem $(LDFLAGS)
	@echo "Successful compilation!"

ambient: test_bbb/veml6030/ambient.c
	@echo "$(CC) compilation"
	@$(CC) $(CFLAGS) $(INCLUDES) test_bbb/veml6030/ambient.c -o ambient
	@echo "Successful compilation!"
clean:
	rm -rf *.o environmental_monitoring shtc3 server client shared_mem ambient