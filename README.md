# pi_dht_read
This is Adafruit DHT11/22/AM2302 humidity/temperature sensor C/C++ library for Raspberry Pi.
This is derived from [Adafruit's driver](https://github.com/adafruit/Adafruit_Python_DHT)
with the changes to eliminate misreading data due to lack of real-timeness of Linux:
- Use BCM2708 1MHz counter instead of loop count in measuring pulse widths.
- Adjust measured pulse width by detecting the interrupts during the measurement.

