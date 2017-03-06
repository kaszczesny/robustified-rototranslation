#!/usr/bin/make -f

SRC_CC=$(shell find ./src -name *.cc)
SRC_O=$(SRC_CC:%.cc=%.o)
TESTS_CC=$(shell find ./tests -name *.cc)
TESTS_O=$(TESTS_CC:%.cc=%.o)

CC=g++
CCFLAGS=-Wall -Wextra -pedantic -Werror -std=c++11 $(shell pkg-config --cflags --libs opencv)


.PHONY: all
all: $(SRC_O)
	#"Nothing to compile - yet"


.PHONY: test
test: all $(TESTS_O)
	$(CC) -o test $(TESTS_O) $(CCFLAGS)

%.o: %.cc
	$(CC) -c $< -o $@  $(CCFLAGS)

	

.PHONY: clean
clean:
	rm -f test src/*.o tests/*.o
