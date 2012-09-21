ARDUINO_LIBS = Wire/utility Wire
ARDUINO_DIR   = /Applications/Arduino.app/Contents/Resources/Java
ARDMK_DIR     = ~/scripts
AVR_TOOLS_DIR = /usr/local/CrossPack-AVR

MCU = atmega88
F_CPU = 8000000L

include Arduino.mk
