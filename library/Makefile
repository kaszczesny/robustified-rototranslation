#!/usr/bin/make -f

#source and object files
SRC_CC=$(shell find ./src -name *.cc)
SRC_O=$(SRC_CC:%.cc=%.o)
TESTS_CC=$(shell find ./tests -name *.cc)
TESTS_O=$(TESTS_CC:%.cc=%.o)

#compiler variables
CC=g++-5
CCFLAGS=-Wall -Wextra -pedantic -Werror -std=c++11 $(shell pkg-config --cflags --libs opencv)

#target-specific variables
all: TESTFLAG=
test: TESTFLAG=-DTEST


.PHONY: all
all: librr.a

librr.a: $(SRC_O)
	ar rcs librr.a $^


.PHONY: test
test: all $(TESTS_O)
	$(CC) -o $@ $(TESTS_O) $(CCFLAGS) librr.a

%.o: %.cc
	$(CC) -c $< -o $@  $(CCFLAGS) $(MOREFLAGS) $(TESTFLAG)


.PHONY: doc
doc:
	./doxyfile


.PHONY: clean
clean:
	rm -rf *.a test src/*.o tests/*.o html/

