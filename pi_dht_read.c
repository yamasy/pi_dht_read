// Copyright (c) 2014 Adafruit Industries
// Author: Tony DiCola

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/file.h>
#include <time.h>
#include <unistd.h>

#include "bcm2708.h"
#include "realtime.h"
#include "pi_dht_read.h"

#define LOCKFILE "/run/lock/dht_read.lck"

// Signal transition timeout in microsecond
#define MAX_WAIT_US 400

// Number of bytes to expect from the DHT.
// They are humidity high, humidity low, temp high, temp low and checksum.
#define DHT_BYTES  5

// Number of bit pulses to expect from the DHT.  Note that this is 41 because
// the first pulse is a constant 80 microsecond pulse, with 40 pulses to represent
// the data afterwards.
#define DHT_PULSES (1 + DHT_BYTES * 8)

static const char *getLogHeader() {
	static char buff[] = "YYYY-MM-DDTHH:MM:SS dht_read: ";
	time_t timeNow = time(NULL);
	struct tm tmNow;
	localtime_r(&timeNow, &tmNow);
	snprintf(buff, sizeof(buff), "%04d-%02d-%02d %02d:%02d:%02d dht_read: ",
		tmNow.tm_year + 1900,
		tmNow.tm_mon + 1,
		tmNow.tm_mday,
		tmNow.tm_hour,
		tmNow.tm_min,
		tmNow.tm_sec);
	return buff;
}

#define DHT_READ_LOG(fmt, ...) printf("%s" fmt, getLogHeader(), ##__VA_ARGS__ )


// Return time in microsecond when the (pin) input signal is changed to (transitionHigh).
// Returns 0 if timeout. Otherwise non-zero microsecond is returned.
static uint32_t getTransitionMicros(int pin, bool transitionHigh) {
	uint32_t expectedValue = transitionHigh ? (1 << pin) : 0;
	uint32_t startedMicros = pi_timer_micros();
	while (pi_mmio_input(pin) != expectedValue) {
		uint32_t elapsedMicros = pi_timer_micros() - startedMicros;
		if (elapsedMicros >= MAX_WAIT_US) {
			return 0; // Timeout!
		}
	}
	uint32_t nowMicros = pi_timer_micros();
	// If nowMicro is 0, which happens 1uS every 2^32 us(1 hour 11 minute 35 second),
	// return maximum uint32 value, which is one microsecond earier time.
	return (nowMicros == 0) ? UINT32_MAX : nowMicros;
}

static int pi_dht_read(int type, int pin, float* pHumidity, float* pTemperature) {
	*pTemperature = 0.0f;
	*pHumidity = 0.0f;

	// Store pulse widths that each DHT bit pulse is low and high.
	// Make sure array is initialized to start at zero.
	uint32_t lowMicros[DHT_PULSES + 1] = {0};
	uint32_t highMicros[DHT_PULSES] = {0};

	// Set pin to output.
	pi_mmio_set_output(pin);

	// Bump up process priority and change scheduler to try to try to make process more 'real time'.
	set_max_priority();

	// Set pin high for ~500 milliseconds.
	pi_mmio_set_high(pin);
	sleep_milliseconds(500);

	// The next calls are timing critical and care should be taken
	// to ensure no unnecssary work is done below.

	// Set pin low for ~20 milliseconds.
	pi_mmio_set_low(pin);
	busy_wait_milliseconds(20);

	// Set pin at input.
	pi_mmio_set_input(pin);

	// Need a very short delay before reading pins or else value is sometimes still low.
	pi_timer_sleep_micros( 2 );

	// Wait for DHT to pull pin low.
	uint32_t lowStartedUs = getTransitionMicros(pin, false);
	if (lowStartedUs == 0) {
		set_default_priority();
		DHT_READ_LOG("Timeout waiting for response low\n");
		return 0;
	}

	// Record pulse widths for the expected result bits.
	int i;
	uint32_t highStartedUs;
	for (i=0; i < DHT_PULSES; i++) {

		// Count how long pin is low and store in lowMicros[i]
		highStartedUs = getTransitionMicros(pin, true);
		if (highStartedUs == 0) {
			set_default_priority();
			DHT_READ_LOG("Timeout waiting for high[%d]\n", i);
			return 0;
		}
		lowMicros[i] = highStartedUs - lowStartedUs; 

		// Count how long pin is high and store in highMicros[i]
		lowStartedUs = getTransitionMicros(pin, false);
		if (lowStartedUs == 0) {
			set_default_priority();
			DHT_READ_LOG("Timeout waiting for low[%d]\n", i);
			return 0;
		}
		highMicros[i] = lowStartedUs - highStartedUs;
	}
	// Count how log pin is low and store the final lowMicros
	highStartedUs = getTransitionMicros(pin, true);
	if (highStartedUs == 0) {
		// Timeout waiting for response.
		set_default_priority();
		DHT_READ_LOG("Timeout waiting for high[release]\n");
		return 0;
	}
	lowMicros[DHT_PULSES] = highStartedUs - lowStartedUs; 

	// Done with timing critical code, now interpret the results.

	// Drop back to normal priority.
	set_default_priority();

	uint32_t threshold = 0;
	bool needAdjust = true;
	while (needAdjust) {
		// Compute the average low pulse width to use as a 50 microsecond reference threshold.
		// Ignore the first reading because it is a constant 80 microsecond pulse.
		threshold = 0;
		for (i=1; i < DHT_PULSES; i++) {
			threshold += lowMicros[i];
		}
		threshold /= DHT_PULSES-1;
		uint32_t lowHighThreshold = threshold * 2;
		
		// Adjust high pulse widths for the interrupts
		needAdjust = false;
		for (i=1; i < DHT_PULSES; i++) {
			// If the high width is less than the threshold...
			if (highMicros[i] < threshold) {
				uint32_t lowHigh = lowMicros[i] + highMicros[i];
				// But the (low + high) width is equal to or more than the threshold...
				if (lowHigh >= lowHighThreshold) {
					// Interrupted during high detection. Add interrupt time to highMicors
					DHT_READ_LOG("Adjusting bit[%d] : %u -> %u\n", i, highMicros[i], (highMicros[i] + lowMicros[i] - threshold));
					highMicros[i] += lowMicros[i] - threshold;
					lowMicros[i] = threshold;
					needAdjust = true;
				}
			} else { // If the high width is equal or more than the threshold...
				uint32_t lowHigh = highMicros[i] + lowMicros[i+1];
				// But the (high+low) width is less than the threshold
				if (lowHigh < lowHighThreshold) {
					// Interrupted during low detection. Subtract interrupt time from highMicros.
					DHT_READ_LOG("Adjusting bit[%d] : %u -> %u\n", i, highMicros[i], (highMicros[i] + lowMicros[i+1] - threshold));
					highMicros[i] += lowMicros[i+1] - threshold;
					lowMicros[i+1] = threshold;
					needAdjust = true;
				}
			}
		} // for adjust loop
	} // while needAdjust

	// Interpret each high pulse as a 0 or 1 by comparing it to the 50us reference.
	// If the count is less than 50us it must be a ~28us 0 pulse, and if it's higher
	// then it must be a ~70us 1 pulse.
	uint8_t data[DHT_BYTES] = {0};
	for (i=1; i < DHT_PULSES; i++) {
		int index = (i-1)/8;
		data[index] <<= 1;
		if (highMicros[i] >= threshold) {
			// One bit for long pulse.
			data[index] |= 1;
		}
		// Else zero bit for short pulse.
	}

	// Useful debug info:
	//printf("Data: 0x%x 0x%x 0x%x 0x%x 0x%x\n", data[0], data[1], data[2], data[3], data[4]);

	// Verify checksum of received data.
	if (data[4] != ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) {
		DHT_READ_LOG("Checksum error\n");
		for (i=0; i <DHT_PULSES; i++) DHT_READ_LOG("%2d,%4u,%4u\n", i, lowMicros[i], highMicros[i]);
		DHT_READ_LOG("%2d,%4u\n", DHT_PULSES, lowMicros[DHT_PULSES]);
		return 0;
	}
	if (type == DHT11) {
		// Get humidity and temp for DHT11 sensor.
		*pHumidity = (float)data[0];
		*pTemperature = (float)data[2];
	} else if (type == DHT22) {
		// Calculate humidity and temp for DHT22 sensor.
		*pHumidity = (data[0] * 256 + data[1]) / 10.0f;
		*pTemperature = ((data[2] & 0x7F) * 256 + data[3]) / 10.0f;
		if (data[2] & 0x80) {
			*pTemperature *= -1.0f;
		}
	}
	return 1;
}

static int open_lockfile(const char *filename) {
	int fd = open(filename, O_CREAT | O_RDONLY, 0600);
	if (fd < 0) {
		printf("Failed to access lock file: %s\nerror: %s\n", filename, strerror(errno));
		return -1;
	}
	if (flock(fd, LOCK_EX | LOCK_NB) == -1) {
		if(errno == EWOULDBLOCK) {
			printf("Lock file is in use\n");
		}
		perror("Flock failed");
		return -1;
	}
	return fd;
}

static void close_lockfile(int fd) {
	if(flock(fd, LOCK_UN) == -1) {
		perror("Failed to unlock file");
	}
	if(close(fd) == -1) {
		perror("Closing descriptor on lock file failed");
	}
}

int dht_read(int type, int pin, float *pHumidity, float *pTemperature) {
	int success = 0;
	// Validate humidity and temperature arguments and set them to zero.
	if (pHumidity == NULL || pTemperature == NULL) {
		DHT_READ_LOG("bad argument\n");
	} else 
	// Initialize GPIO library.
	if (pi_mmio_init() < 0) {
		DHT_READ_LOG("MMIO init failed. May not be root\n");
	} else {
		int lockfd = -1;
		int count = 10;
		while (count-- > 0) {
			if (lockfd < 0) {
				lockfd = open_lockfile(LOCKFILE);
			}
			if (lockfd >= 0) {
				success = pi_dht_read(type, pin, pHumidity, pTemperature);
				if (success) {
					count = 0;
				}
			}
			if (count > 0) {
				sleep(1); // wait 1 sec to refresh
			}
		} // while count > 0
		if (lockfd >= 0) {
			close_lockfile(lockfd);
		}
	} // successfully initialized GPIO library
	return success;
}
