#!/usr/bin/python

# Bare-bones MIM - Delegate phone APDU requests to pyscard.
#


# Structured as:

# - apdu_fw class: connects to abstract apdu handler and to phone through SIMtrace
# - apdu handler: supports interface:
#    - apdu()
#    - reset()




import sys
import tag  # some SIM-related constant definitions
import smartcard
import subprocess
import struct
import time

import gsmtap
import hextools
import apdufw

log = sys.stderr.write


import usb

# SIMtrace slave commands and events, see iso7816_slave.h 
CMD_SET_ATR  = 0
CMD_SET_SKIP = 1
CMD_HALT     = 2
CMD_POLL     = 3
CMD_R_APDU   = 4

EVT_RESET    = 2
EVT_C_APDU   = 4

def usb_find(idVendor, idProduct):
    busses = usb.busses()
    for bus in busses:
        devices = bus.devices
        for dev in devices:
            if ((dev.idVendor  == idVendor) and
                (dev.idProduct == idProduct)):
                return dev
    return 'wrong'


# see iso7816_slave.h
# AT91 is little endian.
def u32(val):
    return [val & 0xFF,
            (val >> 8) & 0xFF,
            (val >> 16) & 0xFF,
            (val >> 24) & 0xFF];

def pack(reply,sw1,sw2):
    p = list(reply)
    p.append(sw1)
    p.append(sw2)
    return p




# Implements the handler interface
class pyscard_smartcard:
    def __init__(self, index=0):
        self.connect(index)
        # self.force_SIM = True
        self.force_SIM = False

    def connect(self, index=0):
        r = smartcard.System.readers()
        # print r # => ['SCM Microsystems Inc. SCR 331 [CCID Interface] (21121250210402) 00 00']
        self.c = r[index].createConnection()
        self.c.connect()

    def disconnect(self):
        self.c.disconnect()
    
    def reset(self):
        self.disconnect()
        self.connect()

    def getATR(self):
        return self.c.getATR()

    # Perform APDU request on card
    def apdu(self, c_apdu):

        # Force SIM protocol for USIM cards.
        if (self.force_SIM):
            if (c_apdu[0] != 0xA0):
                return [0x6E, 0x00]

        if (c_apdu[1] == 0x10): # TERMINAL_PROFILE
            data, sw1, sw2 = self.c.transmit( list(c_apdu) )
            # Make phone poll for command
            # return pack(data,0x91,0x20)
            return pack(data,sw1,sw2)

        # Delegate to MIM.
        (data,sw1,sw2) = self.c.transmit( list(c_apdu) )
        return pack(data,sw1,sw2)



simcard = pyscard_smartcard(0)
mitm = apdufw.forwarder(handler = simcard)

mitm.run()


# log("ATR = {%s}\n" % "".join(map(lambda v: "0x%02x, " % v, c.getATR())))













