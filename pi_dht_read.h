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
#ifndef PI_DHT_READ_H
#define PI_DHT_READ_H

// Define sensor types.
#define DHT11 11
#define DHT22 22
#define AM2302 22

/**
 * Read humidity/temperature from Adafruit DHT sensor, with retries.
 *
 * @param type Sensor type. (ex. AM2302)
 * @param pin GPIO pin number. (ex. 4)
 * @param pHumidity Pointer to float where humidity is set on return.
 * @param pTemperature Pointer to float where temperature is set on return.
 * @return 1 if successful. 0 if failed.
 */
int dht_read(int type, int pin, float *pHumidity, float *pTemperature);

#endif
