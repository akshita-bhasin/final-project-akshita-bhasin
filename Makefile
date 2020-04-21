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

all: environmental_monitoring shtc3 server client

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

client: test_bbb/client-socket/client.c
	@echo "$(CC) compilation"
	@$(CC) $(CFLAGS) $(INCLUDES) test_bbb/client-socket/client.c -o client
	@echo "Successful compilation!"



clean:
	rm -rf *.o environmental_monitoring shtc3 server client