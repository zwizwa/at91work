#!/usr/bin/python

# Bare-bones MIM - Delegate phone APDU requests to pyscard.
#
# The iso7816_slave firmware on the SIMtrace presents an ACM device.
# Connect it to this script, e.g. using:
#   socat /dev/ttyACM0 EXEC:./mim.py
# 

# force_SIM = True

force_SIM = False


import sys
import tag  # some SIM-related constant definitions
import smartcard
import subprocess
import struct
import time

import gsmtap
import hextools

log = sys.stderr.write


# APDUs go over USB vendor control requests
import usb
def usb_find(idVendor, idProduct):
    busses = usb.busses()
    for bus in busses:
        devices = bus.devices
        for dev in devices:
            if ((dev.idVendor  == idVendor) and
                (dev.idProduct == idProduct)):
                return dev
    return 'wrong'
dev = usb_find(idVendor=0x03eb, idProduct=0x6119) # CDC
# dev = find(idVendor=0x03eb, idProduct=0x6129)  # CCID
dh  = dev.open()
def usb_ctrl_OUT(req, buf):
    rv=dh.controlMsg(0x40,
                     request=req,    # R-APDU
                     buffer=buf,
                     timeout=500)
    return rv

def usb_ctrl_IN(req):
    return dh.controlMsg(0xC0,
                         request=req,
                         buffer=512,
                         timeout=500)

# SIMtrace slave commands and events, see iso7816_slave.h 
CMD_SET_ATR  = 0
CMD_SET_SKIP = 1
CMD_HALT     = 2
CMD_POLL     = 3
CMD_R_APDU   = 4

EVT_RESET    = 2
EVT_C_APDU   = 4


def c_apdu():
    while True:
        msg = []
        while (not len(msg)):
            msg = usb_ctrl_IN(CMD_POLL)
        evt = msg[0]

        if (evt == EVT_C_APDU):
            data = msg[4:]
            return data

        if (evt == EVT_RESET):
            log("RESET CARD\n")
            global c
            c.disconnect()
            c = card_connect()
        else:
            log("unknown event: %s\n" % hextools.bytes2hex(msg))

def r_apdu(msg):
    usb_ctrl_OUT(CMD_R_APDU, msg)

def command(tag, payload=[]):  # dummy byte
    log("CMD %d %s\n" % (tag, hextools.bytes2hex(payload)))
    usb_ctrl_OUT(tag, payload)




# connect to card
def card_connect():
    r = smartcard.System.readers()
    # print r # => ['SCM Microsystems Inc. SCR 331 [CCID Interface] (21121250210402) 00 00']
    c = r[0].createConnection()
    c.connect()
    return c

c = card_connect()





# log("ATR = {%s}\n" % "".join(map(lambda v: "0x%02x, " % v, c.getATR())))

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

def pack(reply,sw1,sw2):
    p = list(reply)
    p.append(sw1)
    p.append(sw2)
    return p


# Delegate APDU handling to card.
def default_delegate(msg):
    data, sw1, sw2 = c.transmit( list(msg) )
    return pack(data,sw1,sw2)


# Perform APDU request on card
def apdu(msg, delegate = default_delegate):

    # Force SIM protocol for USIM cards.
    if (force_SIM):
        if (msg[0] != 0xA0):
            return [0x6E, 0x00]

    if (msg[1] == 0x10): # TERMINAL_PROFILE
        data, sw1, sw2 = c.transmit( list(msg) )
        # Make phone poll for command
        # return pack(data,0x91,0x20)
        return pack(data,sw1,sw2)

    #if (msg[1] == 0x2C): # UNBLOCK_PIN
    #    #time.sleep(1)
    #    return [0x63, 0xCA]

    # Delegate to MIM.
    return delegate( msg )


# see iso7816_slave.h
# AT91 is little endian.
def u32(val):
    return [val & 0xFF,
            (val >> 8) & 0xFF,
            (val >> 16) & 0xFF,
            (val >> 24) & 0xFF];


def tick():
    c = c_apdu()
    r = apdu(c)
    r_apdu(r)
    gsmtap.log(c,r)
    log("C-APDU:%s\n" % hextools.bytes2hex(c))
    pretty_apdu(c)
    log("R-APDU:%s\n" % hextools.bytes2hex(r))


def mainloop():
    while 1:
        tick()

command(CMD_SET_ATR, c.getATR())
command(CMD_SET_SKIP, u32(1))
command(CMD_HALT)

# reboot android phone
adb = "/opt/xc/android/android-sdk-linux/platform-tools/adb"
def reboot():
    subprocess.call([adb, "shell", "reboot"])


# start handling incoming phone C-APDUs
reboot()
mainloop()





