CC := gcc
CFLAGS := -g -std=c11 -Wall -I.
SOURCECODE := $(wildcard *.c)
ELFFILE := main

.PNONY: build clean

build:
	$(CC) -o $(ELFFILE) $(SOURCECODE) $(CFLAGS)

clean:
	$(RM) *.o
	
