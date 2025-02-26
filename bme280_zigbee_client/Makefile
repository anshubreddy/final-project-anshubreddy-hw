CC ?= $(CROSS_COMPILE)gcc
CFLAGS ?= -Wall -Werror -g
LDFLAGS ?=

all: bme280_zigbee_client

bme280_zigbee_client: bme280_zigbee_client.o
	$(CC) $^ $(LDFLAGS) -o $@

bme280_zigbee_client.o: bme280_zigbee_client.c
	$(CC) -c $^ $(CFLAGS) -o $@

clean:
	rm -f *.o bme280_zigbee_client
