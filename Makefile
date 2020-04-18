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

# all: hello_world uart tmp102

# tmp102: test_bbb/tmp102/tmp102.c
# 	@echo "$(CC) compilation"
# 	@$(CC) $(CFLAGS) $(INCLUDES) test_bbb/tmp102/tmp102.c -o tmp102
# 	@echo "Successful compilation!"

# hello_world: test_bbb/hello_world/hello_world.c
# 	@echo "$(CC) compilation"
# 	@$(CC) $(CFLAGS) $(INCLUDES) test_bbb/hello_world/hello_world.c -o hello_world
# 	@echo "Successful compilation!"

# uart: test_bbb/uart/uart.c
# 	@echo "$(CC) compilation"
# 	@$(CC) $(CFLAGS) $(INCLUDES) test_bbb/uart/uart.c -o uart
# 	@echo "Successful compilation!"

# clean:
# 	rm -rf hello_world uart tmp102

all: environmental_monitoring shtc3 tmp102

environmental_monitoring: main.c
	@echo "$(CC) compilation"
	@$(CC) $(CFLAGS) $(INCLUDES) main.c -o environmental_monitoring
	@echo "Successful compilation!"

shtc3: test_bbb/SHTC3.c
	@echo "$(CC) compilation"
	@$(CC) $(CFLAGS) $(INCLUDES) test_bbb/SHTC3.c -o shtc3
	@echo "Successful compilation!"

tmp102: test_bbb/tmp102/tmp102.c
	@echo "$(CC) compilation"
	@$(CC) $(CFLAGS) $(INCLUDES) test_bbb/tmp102/tmp102.c -o tmp102
	@echo "Successful compilation!"

clean:
	rm -rf environmental_monitoring shtc3 tmp102