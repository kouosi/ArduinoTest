BOARD 	:= arduino:avr:nano:cpu=atmega328old # arduino:avr:uno
PORT 	?= /dev/ttyACM0
BIN 	:= build/$(notdir $(SKETCH:.cpp=.hex))
BAUD	?= 9600

all: compile

init-first-time:
	arduino-cli sketch new .

init-arduino-first-time:
	arduino-cli config init
	arduino-cli core update-index
	arduino-cli core install arduino:avr

compile:
	arduino-cli compile --fqbn $(BOARD) . --build-path build
	@# arduino-cli compile --fqbn $(BOARD) --output-dir build/release . --build-path build

uploadMonitor: upload monitor

upload: compile
	arduino-cli upload -p $(PORT) --fqbn $(BOARD) --input-dir build

monitor:
	  arduino-cli monitor -p $(PORT) -b $(BOARD) --config $(BAUD)

clean:
	rm -rf build
