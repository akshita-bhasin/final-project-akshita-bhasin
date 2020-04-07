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

all: hello_world
	
: test_bbb/hello_world.c
	@echo "$(CC) compilation"
	@$(CC) $(CFLAGS) $(INCLUDES) test_bbb/hello_world.c -o hello_world
	@echo "Successful compilation!"

clean:
	rm -rf hello_world