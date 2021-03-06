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

all: environmental_monitoring client tmp102 ambient

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

client: client.o logging.o
	@echo "$(CC) compilation"
	@$(CC) $(CFLAGS) $(INCLUDES) client.o logging.o -o client
	@echo "Successful compilation!"

client.o: src/client.c inc/logging.h
	@echo "$(CC) compilation"
	@$(CC) $(CFLAGS) $(INCLUDES) -c src/client.c
	@echo "Successful compilation!"

logging.o: src/logging.c inc/logging.h
	@$(CC) $(CFLAGS) $(INCLUDES) -c src/logging.c

tmp102: test_bbb/tmp102/tmp102.c
	@echo "$(CC) compilation"
	@$(CC) $(CFLAGS) $(INCLUDES) test_bbb/tmp102/tmp102.c -o tmp102
	@echo "Successful compilation!"

ambient: test_bbb/veml6030/ambient.c
	@echo "$(CC) compilation"
	@$(CC) $(CFLAGS) $(INCLUDES) test_bbb/veml6030/ambient.c -o ambient
	@echo "Successful compilation!"
clean:
	rm -rf *.o environmental_monitoring client tmp102 ambient