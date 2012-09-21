Files in this repository:
*.patch - Patch to apply to APM repository to enable telemetry data transfer
adapter.ino - Source for sensor station emulator
apm-hss-emulator.hex - Compiled source for flashing to ATMega88
Makefile - Makefile to compile source on commandline.
hss.pde - funtions for APM data transfer. The patch already includes this
hss.sch - Schematics in Eagle format
Arduino.mk - Actual makefile by Martin Oldfield

Arduino.mk need support scripts to work. If you have trouble compiling the source, clone the makefile repository from https://github.com/mjoldfield/Arduino-Makefile


