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

log = sys.stderr.write


def bytes2hex(bytes):
    return "".join(map(lambda v: "%02X"%v, bytes))

def hex2bytes(hex):
    return map(ord,hex.decode("hex"))



# APDUs go over USB vendor control requests
import usb
def find(idVendor, idProduct):
    busses = usb.busses()
    for bus in busses:
        devices = bus.devices
        for dev in devices:
            if ((dev.idVendor  == idVendor) and
                (dev.idProduct == idProduct)):
                return dev
    return 'wrong'
dev = find(idVendor=0x03eb, idProduct=0x6119) # CDC
# dev = find(idVendor=0x03eb, idProduct=0x6129)  # CCID
dh  = dev.open()
def ctrl_OUT(req, buf):
    rv=dh.controlMsg(0x40,
                     request=req,    # R-APDU
                     buffer=buf,
                     timeout=500)
    return rv

def ctrl_IN(req):
    return dh.controlMsg(0xC0,
                         request=req,
                         buffer=512,
                         timeout=500)
UC_COMMAND = 1
UC_C_APDU  = 2
UC_R_APDU  = 3

def c_apdu():
    cla = 0xFF
    msg = []
    while (cla == 0xFF):
        msg = ctrl_IN(UC_C_APDU)
        cla = msg[0]
    log("C-APDU:%s\n" % bytes2hex(msg))
    return msg

def r_apdu(msg):
    log("R-APDU:%s\n" % bytes2hex(msg))
    ctrl_OUT(UC_R_APDU, msg)




# connect to card
r = readers()
# print r # => ['SCM Microsystems Inc. SCR 331 [CCID Interface] (21121250210402) 00 00']
c = r[0].createConnection()
c.connect()








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
    pretty_apdu( msg )

    # Force SIM protocol for USIM cards.
    if (force_SIM):
        if (msg[0] != 0xA0):
            return [0x6E, 0x00]

    if (msg[1] == 0x10): # TERMINAL_PROFILE
        data, sw1, sw2 = c.transmit( list(msg) )
        # Make phone poll for command
        # return pack(data,0x91,0x20)
        return pack(data,sw1,sw2)

    # Delegate to MIM.
    return delegate( msg )


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
    msg = list(tag)
    msg.extend(payload)
    log("CMD:%s\n" % bytes2hex(msg))
    ctrl_OUT(UC_COMMAND, msg)

def tick():
    c = c_apdu()
    r = apdu(c)
    r_apdu(r)

def mainloop():
    while 1:
        tick()

command(CMD_SET_ATR, c.getATR())
command(CMD_SET_SKIP, u32(2))
command(CMD_HALT)

# reset android phone
def reboot():
    subprocess.call(["adb", "shell", "reboot"])


# start handling incoming phone C-APDUs
reboot()
mainloop()





