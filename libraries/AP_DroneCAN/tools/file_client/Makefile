# build the DroneCAN file client
#
#   make            build with CANFD support
#   make NOCANFD=1  build for classic CAN only

TOPDIR := ../../../..
CANARD_DIR := $(TOPDIR)/modules/DroneCAN/libcanard

CFLAGS := -O2 -Wall -I$(CANARD_DIR)
ifndef NOCANFD
CFLAGS += -DCANARD_ENABLE_CANFD=1
endif

all: dronecan_file_client

canard.o: $(CANARD_DIR)/canard.c
	$(CC) $(CFLAGS) -c $< -o $@

dronecan_file_client: dronecan_file_client.cpp canard.o
	$(CXX) $(CFLAGS) $^ -o $@

clean:
	rm -f dronecan_file_client canard.o

.PHONY: all clean
