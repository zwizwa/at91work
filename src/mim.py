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

# FIXME: remove this - use wireshark gsmtap instead.
def pretty_apdu(msg):
  try:  
    ins = msg[1]
    try:
        log(" %s" % tag.iso7816[ins])
    except:
        log(" %02X" % ins)
        pass
    if (msg[1] == 0xA4): # SELECT_FILE
        file = msg[5] * 256 + msg[6]
        try:
            log(" %s" % tag.USIMConstants[file])
        except:
            try:
                log(" %s" % tag.SIMConstants[file])
            except:
                log(" %04X" % file)
                pass
            pass
    log("\n")
  except:
    pass


class apdu_fw:
    def __init__(self, handler, idVendor=0x03eb, idProduct=0x6119):
        self.handler = handler
        self.dev =  usb_find(idVendor=0x03eb, idProduct=0x6119) # CDC
        # dev = find(idVendor=0x03eb, idProduct=0x6129)  # CCID
        self.dh = self.dev.open()

    def usb_ctrl_OUT(self, req, buf):
        return self.dh.controlMsg(0x40,
                                  request=req,    # R-APDU
                                  buffer=buf,
                                  timeout=500)
    def usb_ctrl_IN(self, req):
        return self.dh.controlMsg(0xC0,
                                  request=req,
                                  buffer=512,
                                  timeout=500)
    def log(self, msg):
        sys.stderr.write(msg)

    # FIXME: make this a proper event-handler loop
    def c_apdu(self):
        while True:
            msg = []
            while (not len(msg)):
                msg = self.usb_ctrl_IN(CMD_POLL)

            evt = msg[0]
            if (evt == EVT_C_APDU):
                data = msg[4:]
                return data

            if (evt == EVT_RESET):
                log("RESET CARD\n")
                self.handler.reset()
            else:
                log("unknown event: %s\n" % hextools.bytes2hex(msg))

    def r_apdu(self, msg):
        self.usb_ctrl_OUT(CMD_R_APDU, msg)

    def command(self, tag, payload=[]):  # dummy byte
        log("CMD %d %s\n" % (tag, hextools.bytes2hex(payload)))
        self.usb_ctrl_OUT(tag, payload)



    def tick(self):
        c = self.c_apdu()
        r = self.handler.apdu(c)
        self.r_apdu(r)
        gsmtap.log(c,r)
        log("C-APDU:%s\n" % hextools.bytes2hex(c))
        pretty_apdu(c)
        log("R-APDU:%s\n" % hextools.bytes2hex(r))


    def mainloop(self):
        while 1:
            self.tick()

    def reboot(self):
        adb = "/opt/xc/android/android-sdk-linux/platform-tools/adb"
        log("adb shell reboot\n")
        subprocess.call([adb, "shell", "reboot"])

    def run(self):
        self.command(CMD_SET_ATR, self.handler.getATR())
        self.command(CMD_SET_SKIP, u32(1))
        self.command(CMD_HALT)

        # start handling incoming phone C-APDUs
        self.reboot()
        self.mainloop()




# Implements the handler interface
class apdu_smartcard:
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



simcard = apdu_smartcard()
mitm = apdu_fw(handler = simcard)

mitm.run()


# log("ATR = {%s}\n" % "".join(map(lambda v: "0x%02x, " % v, c.getATR())))













