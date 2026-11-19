# Host unit tests (no ESP-IDF needed): make -C test
CC ?= gcc
CFLAGS ?= -std=c99 -Wall -Wextra -Wpedantic -Werror -O2
BUILD ?= build

all: run

$(BUILD)/test_main: test_main.c ../src/train_horn_ctl.c ../include/train_horn_ctl.h
	mkdir -p $(BUILD)
	$(CC) $(CFLAGS) -I../include test_main.c ../src/train_horn_ctl.c -lm -o $@

run: $(BUILD)/test_main
	./$(BUILD)/test_main

clean:
	rm -rf $(BUILD)

.PHONY: all run clean
