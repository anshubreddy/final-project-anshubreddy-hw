CC ?= $(CROSS_COMPILE)gcc
CFLAGS ?= -Wall -Werror -g
LDFLAGS ?=

all: zigbee_send

zigbee_send: zigbee_send.o
	$(CC) $^ $(LDFLAGS) -o $@

zigbee_send.o: zigbee_send.c
	$(CC) -c $^ $(CFLAGS) -o $@

clean:
	rm -f *.o zigbee_send
