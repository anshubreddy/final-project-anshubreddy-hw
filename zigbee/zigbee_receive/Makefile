CC ?= $(CROSS_COMPILE)gcc
CFLAGS ?= -Wall -Werror -g
LDFLAGS ?=

all: zigbee_receive

zigbee_receive: zigbee_receive.o
	$(CC) $^ $(LDFLAGS) -o $@

zigbee_receive.o: zigbee_receive.c
	$(CC) -c $^ $(CFLAGS) -o $@

clean:
	rm -f *.o zigbee_receive
