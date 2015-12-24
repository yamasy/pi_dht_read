#include <stdio.h>
#include <stdlib.h>
#include "pi_dht_read.h"

// GPIO pin number for DHT sensor
#define DHTPIN 4

int main(int argc, const char **argv) {
	float humidity, temperature;
	int count = argc < 2 ? 1 : atoi(argv[1]);
	while (count-- > 0) {
		int success = dht_read(AM2302, DHTPIN, &humidity, &temperature);
		if (success) {
			printf("temperature:%.1f Humidity:%.1f\n", temperature, humidity);
		}
		if (count > 0) {
			printf("Press ENTER key to continue\n");
			getchar();
		}
	}
	return 0;
}
