all: test_dht_read

test_dht_read: test_dht_read.c pi_dht_read.c bcm2708.c realtime.c
	gcc -o $@ -W -Wall -lrt $^

clean:
	rm -f test_dht_read

