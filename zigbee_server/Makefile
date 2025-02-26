CC ?= $(CROSS_COMPILE)gcc
CFLAGS ?= -Wall -Werror -g
LDFLAGS ?=

all: zigbee_server

zigbee_server: zigbee_server.o
	$(CC) $^ $(LDFLAGS) -o $@

zigbee_server.o: zigbee_server.c
	$(CC) -c $^ $(CFLAGS) -o $@

clean:
	rm -f *.o zigbee_server
