make:
	arduino-cli compile --upload -b arduino:avr:uno . -p /dev/ttyACM0
mm:
	make make
	make monitor
set:
	sudo chmod a+rw /dev/ttyACM0
monitor:
	cat < /dev/ttyACM0
