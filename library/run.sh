#!/usr/bin/env bash

make clean && make test MOREFLAGS="-D NO_DISPLAY" && ./test

