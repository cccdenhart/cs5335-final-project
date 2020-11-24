
PORT := /dev/ttyUSB0
FQBN := arduino:avr:mega:cpu=atmega2560

all:
	arduino-cli compile --fqbn $(FQBN)
	arduino-cli upload --port $(PORT) --fqbn $(FQBN)

setup:
	arduino-cli core update-index
	arduino-cli core install arduino:avr

screen:
	stty -F /dev/ttyUSB0 9600
	screen /dev/ttyUSB0

clean:
	rm -rf build

.PHONY: all setup screen clean
