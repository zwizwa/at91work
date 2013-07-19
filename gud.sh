#!/bin/bash
# FIXME: specific setup [tom]
HERE=$(dirname $0)
exec ~/sam7/gdb -i=mi $HERE/usb-device-ccid-project/bin/usb-device-ccid-project-simtrace-at91sam7s128-flash_dfu.elf
