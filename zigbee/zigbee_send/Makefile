CC ?= $(CROSS_COMPILE)gcc
CFLAGS ?= -Wall -Werror -g
LDFLAGS ?=

all: zigbee_send

bme280: zigbee_send.o
	$(CC) $^ $(LDFLAGS) -o $@

bme280_sensor.o: zigbee_send.c
	$(CC) -c $^ $(CFLAGS) -o $@

clean:
	rm -f *.o zigbee_send
