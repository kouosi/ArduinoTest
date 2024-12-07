BOARD 	:= arduino:avr:uno
PORT 	:= /dev/ttyACM0
BIN 	:= build/$(notdir $(SKETCH:.cpp=.hex))

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
	  arduino-cli monitor -p /dev/ttyACM0 -b arduino:avr:uno --config 9600

clean:
	rm -rf build
