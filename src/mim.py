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
import struct
import socket

log = sys.stderr.write


def bytes2hex(bytes):
    return "".join(map(lambda v: "%02X"%v, bytes))

def bytes2str(bytes):
    return struct.pack('B'*len(bytes), *bytes)

def hex2bytes(hex):
    return map(ord,hex.decode("hex"))



# log APDUs to gsmtap
gsmtap_hdr = [2, # GSMTAP_VERSION,
              4, # nb of u32 in header
              4, # GSMTAP_TYPE_SIM,
              0,0,0,0,0,0,0,0,0,0,0,0,0]

# gsmtap_addr = ("127.0.0.1", 4729)
gsmtap_addr = ("<broadcast>", 4729)
gsmtap_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
gsmtap_sock.bind(('127.0.0.1', 0))
# broadcast avoids ICMP port unreachable
gsmtap_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

def gsmtap(c_apdu, r_apdu):
    msg = list(gsmtap_hdr)
    msg.extend(c_apdu)
    msg.extend(r_apdu)
    gsmtap_sock.sendto(bytes2str(msg), gsmtap_addr)



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

# SIMtrace slave commands, see iso7816_slave.h 
CMD_SET_ATR  = 0
CMD_SET_SKIP = 1
CMD_HALT     = 2
CMD_POLL     = 3
CMD_R_APDU   = 4

EVT_RESET   = 2
EVT_C_APDU  = 4


def c_apdu():
    while True:
        msg = []
        while (not len(msg)):
            msg = usb_ctrl_IN(CMD_POLL)
        evt = msg[0]

        if (evt == EVT_C_APDU):
            data = msg[4:]
            log("C-APDU:%s\n" % bytes2hex(data))
            return data

        if (evt == EVT_RESET):
            log("RESET CARD\n")
            global c
            c.disconnect()
            c = card_connect()
        else:
            log("unknown event: %s\n" % bytes2hex(msg))

def r_apdu(msg):
    log("R-APDU:%s\n" % bytes2hex(msg))
    usb_ctrl_OUT(CMD_R_APDU, msg)

def command(tag, payload=[]):  # dummy byte
    log("CMD %d %s\n" % (tag, bytes2hex(payload)))
    usb_ctrl_OUT(tag, payload)




# connect to card
def card_connect():
    r = readers()
    # print r # => ['SCM Microsystems Inc. SCR 331 [CCID Interface] (21121250210402) 00 00']
    c = r[0].createConnection()
    c.connect()
    return c

c = card_connect()





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


def tick():
    c = c_apdu()
    r = apdu(c)
    r_apdu(r)
    gsmtap(c,r)

def mainloop():
    while 1:
        tick()

command(CMD_SET_ATR, c.getATR())
command(CMD_SET_SKIP, u32(2))
command(CMD_HALT)

# reboot android phone
def reboot():
    subprocess.call(["adb", "shell", "reboot"])


# start handling incoming phone C-APDUs
reboot()
mainloop()





