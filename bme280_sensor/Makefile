CC ?= $(CROSS_COMPILE)gcc
CFLAGS ?= -Wall -Werror -g
LDFLAGS ?=

all: bme280

bme280: bme280_sensor.o
	$(CC) $^ $(LDFLAGS) -o $@

bme280_sensor.o: bme280_sensor.c
	$(CC) -c $^ $(CFLAGS) -o $@

clean:
	rm -f *.o bme280
