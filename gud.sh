#!/bin/bash
# FIXME: specific setup [tom]
HERE=$(dirname $0)
PROJ=usb-device-ccid-project
PROJ=simtrace-phone-project
BOARD=simtrace-at91sam7s128-flash_dfu
ELF=$HERE/$PROJ/bin/$PROJ-$BOARD.elf
exec ~/sam7/gdb -i=mi $ELF

