#!/usr/bin/python

# Bare-bones MIM - Delegate phone APDU requests to pyscard.
#
# The iso7816_slave firmware on the SIMtrace presents an ACM device.
# Connect it to this script, e.g. using:
#   socat /dev/ttyACM0 EXEC:./mim.py
# 

force_SIM = True
# force_SIM = False


import sys
import tag  # some SIM-related constant definitions
from smartcard.System import readers
from smartcard.util import toHexString
import subprocess


# connect to card
r = readers()
# print r # => ['SCM Microsystems Inc. SCR 331 [CCID Interface] (21121250210402) 00 00']
c = r[0].createConnection()
c.connect()



def bytes2hex(bytes):
    return "".join(map(lambda v: "%02X"%v, bytes))

def hex2bytes(hex):
    return map(ord,hex.decode("hex"))

log = sys.stderr.write


# log("ATR = {%s}\n" % "".join(map(lambda v: "0x%02x, " % v, c.getATR())))

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
            log(" %s" % tag.SIMConstants[file])
        except:
            log(" %04X" % file)
            pass
    log("\n")
  except:
    pass

def pack(data,sw1,sw2):
    data.append(sw1)
    data.append(sw2)
    return data


# Delegate APDU handling to card.
def delegate(msg):
    data, sw1, sw2 = c.transmit( msg )
    return pack(data,sw1,sw2)


# Perform APDU request on card
def apdu(msg, handle = delegate):
    pretty_apdu(msg)

    # Force SIM protocol for USIM cards.
    if (force_SIM):
        if (msg[0] != 0xA0):
            return [0x6E, 0x00]

    if (msg[1] == 0x10): # TERMINAL_PROFILE
        data, sw1, sw2 = c.transmit( msg )
        # Make phone poll for command
        # return pack(data,0x91,0x20)
        return pack(data,sw1,sw2)

    else:
        return handle(msg)
    

    # Delegate to card.
    data, sw1, sw2 = handle( msg )
    data.append(sw1)
    data.append(sw2)
    return data

def apdu_hex(hex):
    return bytes2hex(apdu(hex2bytes(hex)))

def c_apdu():
    str = sys.stdin.readline().rstrip()
    log("C-APDU:%s\n" % str)
    return str


def to_phone(str):
    sys.stdout.write("%s\n" % str)
    sys.stdout.flush()

def r_apdu(str):
    log("R-APDU:%s\n\n" % str)
    to_phone(str)


# FIXME: iso7816 parser control is currently embedded in R-APDU packet
# with SW=0xFFFF

# see iso7816_slave.h
# AT91 is little endian.
def u32(val):
    return [val & 0xFF,
            (val >> 8) & 0xFF,
            (val >> 16) & 0xFF,
            (val >> 24) & 0xFF];

CMD_SET_ATR  = u32(0)
CMD_SET_SKIP = u32(1)
CMD_HALT     = u32(2)

def command(tag, payload=[0,0,0,0]):
    b = tag
    b.extend(payload)
    b.extend([0xFF, 0xFF])
    str = bytes2hex(b)
    log("COMMAND:%s\n\n" % str)
    to_phone(str)

def mainloop():
    while 1:
        r_apdu(apdu_hex(c_apdu()))

command(CMD_SET_ATR, c.getATR())
command(CMD_SET_SKIP, u32(2))
command(CMD_HALT)

# reset android phone
subprocess.call(["adb", "shell", "reboot"])

# start handling incoming phone C-APDUs
mainloop()





