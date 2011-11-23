# 
# A basic makefile for compiling .c and .cpp files for Arduino
# By Alejandro Erickson alejandro.erickson@gmail.com
# and some ideas snarfed from the Internet,
# Especially
#    David Wolever 
#    http://blog.codekills.net/
# and
# eighthave, oli.keller,
# alex norman [with help from http://code.google.com/p/arduino/issues/detail?id=65#c5]
# also
# mimicking the output from Verbose Verify and Verbose Upload in the Arduino IDE 0021
# 
# Disclaimer:  I am a GNUmake and Arduino newbie.  Suggestions and improvments are
# welcome.  This makefile does not deal with list files or assembly (except
# where I used a few lines from makefiles that do and was afraid to delete
# something).  Also, note that this builds the core library files in the
# Arduino IDE directory instead of this directory.
# Sorry if it messes up another project you have going...
#
# Instructions:
#
# This makefile assumes you are using Mac OS X and Arduino IDE.  You may
# have to specify a different location for the Arduino IDE and even for
# it's internal file structure.  I've tried to indicate where that might come up.
#
# Make a project folder for your code and add to it your main.cpp along with (this)
# Makefile.  You can also include library files there, but you don't have to.
#
# I think your main.cpp file should look more less like this:
#
# #include "WProgram.h"
# //#include <other library headers>
#
# //<#defines and globals>
#
# extern "C" void __cxa_pure_virtual()
# {
#     cli();    // disable interrupts
#     for(;;);  // do nothing until hard reset
# }
#
# void setup(){
# 	//code...
# }
#
# void loop(){
# 	\\code...
# }
#
# int main(){
# 	init();
#	setup();
#	for(;;) loop();
#	return 0;
# }

#***************************** HARDWARE
#these things are specific to the board.  Try finding the parameters
#for yours on the Internet ^_^
#I've used the ones for the Arduino Duemilanove (or Nano w/ ATmega328)
#UPLOAD_SPEED = 57600
#UPLOAD_PROTOCOL = stk500
#BUILD_MCU = atmega328p
#F_CPU = 16000000


#I've used the ones for the Arduino UNO (or Nano w/ ATmega328)
#UPLOAD_SPEED = 115200
#UPLOAD_PROTOCOL = stk500v1
#BUILD_MCU = atmega328p
# why *L?  I don't know but that's what shows up in the Arduino IDE output
#F_CPU = 16000000L

#Seeeduino Mega
UPLOAD_SPEED = 57600
UPLOAD_PROTOCOL = stk500v1
BUILD_MCU = atmega88
# why *L?  I don't know but that's what shows up in the Arduino IDE output
F_CPU = 8000000L



#this is where your Arduino is plugged in.  You can check it's exact
#location by plugging the board into your computer and typing cd /dev/tty. <tab>
#it might show up as a usb modem instead...
PORT = /dev/tty.usbserial-A7004J48
#PORT = /dev/tty.usbmodem1d11
#/dev/tty.usbserial*
#*********************************** END HARDWARE

#********************* PATHS TO IMPORTANT STUFF
#your main() should be in main.cpp.  don't change this, you'll mess things up
TARGET = main.cpp


#this is where the Arduino IDE is located in your computer.
#it is correct for Version 0021, assuming you just dragged it to /Applications
#like a well behaved little mac sheep.
INSTALL_DIR = /Applications/Arduino.app/Contents/Resources/Java
VERSION = 21

#This is the location of the core library files in the Arduino IDE (version 0021)
ARDUINO = $(INSTALL_DIR)/hardware/arduino/cores/arduino

#this is the path to the avr tools that come with the Arduino IDE.
#we use these instead of the CrossPack tools you would install
#for use with Eclipse.  It's the same stuff, but less work, really...
AVR_TOOLS_PATH = $(INSTALL_DIR)/hardware/tools/avr/bin

#These are non-core library files.  Change them according to your needs
#main.cpp is in there even though it's not technically a library file.
#this works though...
# For example if you have
# #include "Wire.h"
# in main.cpp, then you have to compile the Wire library.  I just copied the
# source for the library to my project folder, but I could have linked to it
# in the Arduino IDE instead.
#.c files
LIB_SRC = \
	$(INSTALL_DIR)/libraries/Wire/utility/twi.c \
#.cpp files.  main.cpp must be here no matter what
#I didn't need to put the Wire.cpp library in the same directory as the project.  I could have linked to it in the Arduino IDE installation.
LIB_CXXSRC = \
	main.cpp \
	$(INSTALL_DIR)/libraries/Wire/Wire.cpp \

#include paths to other headers needed for compilation.  precede with -I.
#you should point to the locations you have extra library files.
OTHERINCS = \
	-Iutility/ \
	-I$(INSTALL_DIR)/libraries/ \
	-I. \
	-I$(AVRTOOLS)/../include/ \
#that last line is for when you use interrupts.h and stuff.  Brows around inside the IDE files to see where these paths are.  My rule is: if in doubt, -Include it.

#************************END PATHS TO IMPORTANT STUFF

#************************CORE LIBRARY ITEMS
#These are the things that need to be compiled for the core library.
#I got the list by looking at this verbose output from the Arduino IDE
#avr-gcc -c -g -Os -w -ffunction-sections -fdata-sections -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=21 -I/arduino/ /arduino//pins_arduino.c -o/tmp/pins_arduino.c.o 
#avr-gcc -c -g -Os -w -ffunction-sections -fdata-sections -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=21 -I/arduino/ /arduino//WInterrupts.c -o/tmp/WInterrupts.c.o 
#avr-gcc -c -g -Os -w -ffunction-sections -fdata-sections -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=21 -I/arduino/ /arduino//wiring.c -o/tmp/wiring.c.o 
#avr-gcc -c -g -Os -w -ffunction-sections -fdata-sections -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=21 -I/arduino/ /arduino//wiring_analog.c -o/tmp/wiring_analog.c.o 
#avr-gcc -c -g -Os -w -ffunction-sections -fdata-sections -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=21 -I/arduino/ /arduino//wiring_digital.c -o/tmp/wiring_digital.c.o 
#avr-gcc -c -g -Os -w -ffunction-sections -fdata-sections -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=21 -I/arduino/ /arduino//wiring_pulse.c -o/tmp/wiring_pulse.c.o 
#avr-gcc -c -g -Os -w -ffunction-sections -fdata-sections -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=21 -I/arduino/ /arduino//wiring_shift.c -o/tmp/wiring_shift.c.o 
CORE_LIB_SRC =  \
	$(ARDUINO)/pins_arduino.c \
	$(ARDUINO)/wiring.c \
	$(ARDUINO)/wiring_analog.c \
	$(ARDUINO)/wiring_digital.c \
	$(ARDUINO)/wiring_pulse.c \
	$(ARDUINO)/wiring_shift.c \
	$(ARDUINO)/WInterrupts.c
#list other library files here
#list core library files here


#These ones are for .cpp files.  Note that main.cpp is irrelevant here
#because it pertains to .pde sketches.  It's contents are
# #include<WProgram.h> int main(void){init(); setup(); for(;;) loop(); return 0;}
#I'm assuming you are writing this into your own main.cpp file.

#avr-g++ -c -g -Os -w -fno-exceptions -ffunction-sections -fdata-sections -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=21 -I/arduino/ /arduino//main.cpp -o/tmp/main.cpp.o 
#avr-g++ -c -g -Os -w -fno-exceptions -ffunction-sections -fdata-sections -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=21 -I/arduino/ /arduino//Tone.cpp -o/tmp/Tone.cpp.o 
#avr-g++ -c -g -Os -w -fno-exceptions -ffunction-sections -fdata-sections -mmcu=atmega328p -DF_CPU=16000000L -DARDUINO=21 -I/arduino/ /arduino//WString.cpp -o/tmp/WString.cpp.o
CORE_LIB_CXXSRC = \
	$(ARDUINO)/Tone.cpp \
	$(ARDUINO)/WString.cpp \
	$(ARDUINO)/HardwareSerial.cpp \
	$(ARDUINO)/WMath.cpp \
	$(ARDUINO)/Print.cpp \
#************************END CORE LIBRARY ITEMS

#*************** COMPILE COMMANDS AND FLAGS
#Toss your extra lib files in with the core files and main.cpp.
# this makes the list of .c and .cpp code we will compile
SRC = $(CORE_LIB_SRC) $(LIB_SRC)
CXXSRC = $(CORE_LIB_CXXSRC) $(LIB_CXXSRC)

#
# Define all object files.  I'm not sure what this does, but it sure helps :D
#
OBJ = $(SRC:.c=.o) $(CXXSRC:.cpp=.o)

#a compile command takes the following things in more or less the same order
# define/locate the compilers and other tools
CXX = $(AVR_TOOLS_PATH)/avr-g++
CC =  $(AVR_TOOLS_PATH)/avr-gcc
AR = $(AVR_TOOLS_PATH)/avr-ar

#<debugging flag>
DEBUG = -g

#<#define flag.  ie hardware information>
DEFS = -DF_CPU=$(F_CPU) -DARDUINO=$(VERSION)

#<mcu flag>
MCU = -mmcu=$(BUILD_MCU)

#<include flag (where to look for header files)>
INCS = -I$(ARDUINO) $(OTHERINCS)

# <optimizations flag>
OPT = -Os

#<warning level flag. what should it warn you about?>
CWARN =  -Wstrict-prototypes -Wall
CXXWARN = -Wall

# <c standard flag. which standard should it use?>
#omitted

#<other options starting with -f.>   Not all of these are necessary...
CTUNING = -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -funsigned-bitfields
CXXTUNING = -fno-exceptions -ffunction-sections -fdata-sections -fpack-struct -fshort-enums -funsigned-bitfields
# <input file name> <output file name>

#put all those flags into one variable
CCFLAGS =  $(DEBUG) $(OPT) $(CWARN) $(CTUNING) $(MCU) $(DEFS) $(INCS)
CXXFLAGS  =  $(DEBUG) $(OPT) $(CXXWARN) $(CXXTUNING) $(MCU) $(DEFS) $(INCS)

#******************** END COMPILE COMMANDS AND FLAGS
#this tells the Makefile which of the recipies to do when you type make
all:  build tmplib.a

#i'm not sure if i need this
#http://www.gnu.org/software/hello/manual/make/Suffix-Rules.html
.SUFFIXES: .elf .hex .eep

build: main.cpp.elf main.cpp.hex  tmplib.a .c.o .cpp.o


#compile all the .cpp files into .o files
.cpp.o:
	@echo Compiling cpp: $<
	$(CXX) -c $(CXXFLAGS) $< -o $@

#compile all the .c files into .o files
.c.o:
	@echo Compiling c: $<
	$(CC) -c $(CCFLAGS) $< -o $@

#put all our library files into an archive, tmplib.a, using avr-ar.  toss main.o in
#there too
tmplib.a: $(OBJ)
	@for i in $(OBJ); do echo "adding $$i to tmplib.a";  $(AR) rcs tmplib.a $$i; done

#compile the elf file from our library files and main source file
main.cpp.elf: main.cpp tmplib.a
	@for i in $(OBJ); do echo "$$i"; done
	echo "making elf file"
	$(CC) $(OPT) -Wl,--gc-sections $(MCU) -o main.cpp.elf main.o tmplib.a -L. -lm

#convert the elf file into an eep file
$(TARGET).eep: $(TARGET).elf
	avr-objcopy -O ihex -j .eeprom --set-section-flags=.eeprom=alloc,load --no-change-warnings --change-section-lma .eeprom=0 $(TARGET).elf $(TARGET).eep 
#convert the eep file to a .hex files
$(TARGET).hex: $(TARGET).eep
	avr-objcopy -O ihex -R .eeprom $(TARGET).elf $(TARGET).hex 

# Programming support using avrdude. Settings and variables.
AVRDUDE_PORT = $(PORT)
AVRDUDE_WRITE_FLASH = -Uflash:w:$(TARGET).hex:i
#note i picked very verbose output.  killing some of those -v might reduce it
AVRDUDE_FLAGS = -v -v -v -v  \
    -p$(BUILD_MCU) -P$(AVRDUDE_PORT) -c$(UPLOAD_PROTOCOL) \
    -b$(UPLOAD_SPEED) -C$(INSTALL_DIR)/hardware/tools/avr/etc/avrdude.conf -D
AVRDUDE = $(AVR_TOOLS_PATH)/avrdude


#avrdude -Cavrdude.conf -v -v -v -v -patmega328p -cstk500v1 \
#-P/dev/tty.usbmodem1d11 -b115200 -D -Uflash:w:main.cpp.hex:i 
# Program the device.
upload: $(TARGET).hex
	$(AVRDUDE) $(AVRDUDE_FLAGS) $(AVRDUDE_WRITE_FLASH)

REMOVE = rm -f

clean:
	$(REMOVE) $(TARGET).hex $(TARGET).eep $(TARGET).cof $(TARGET).elf \
	    $(TARGET).map $(TARGET).sym $(TARGET).lss tmplib.a \
		    $(OBJ)

.PHONY: all build elf hex eep lss sym program coff extcoff clean 
