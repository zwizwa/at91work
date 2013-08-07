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
import tag
from smartcard.System import readers
from smartcard.util import toHexString



# connect to card
r = readers()
# print r # => ['SCM Microsystems Inc. SCR 331 [CCID Interface] (21121250210402) 00 00']
c = r[0].createConnection()
c.connect()



def bytes2hex(bytes):
    return "".join(map(lambda v: "%02X"%v, bytes))

def hex2bytes(hex):
    return map(ord,hex.decode("hex"))

sys.stderr.write("ATR = {%s}\n" % 
                 "".join(map(lambda v: "0x%02x, " % v, c.getATR())))



def pretty_apdu(msg):
    ins = msg[1]
    try:
        sys.stderr.write(" %s" % tag.iso7816[ins])
    except:
        sys.stderr.write(" %02X" % ins)
        pass
    if (msg[1] == 0xA4): # SELECT_FILE
        file = msg[5] * 256 + msg[6]
        try:
            sys.stderr.write(" %s" % tag.SIMConstants[file])
        except:
            sys.stderr.write(" %04X" % file)
            pass
    sys.stderr.write("\n")


# Perform APDU request on card
def apdu(msg):
    try:
        pretty_apdu(msg)
    except:
        pass

    # Force SIM protocol
    if (force_SIM):
        if (msg[0] != 0xA0):
            return [0x6E, 0x00]
                    
    data, sw1, sw2 = c.transmit( msg )
    data.append(sw1)
    data.append(sw2)
    return data

def apdu_hex(hex):
    return bytes2hex(apdu(hex2bytes(hex)))

def c_apdu():
    str = sys.stdin.readline().rstrip()
    sys.stderr.write("C-APDU:%s\n" % str)
    return str


def to_simtrace(str):
    sys.stdout.write("%s\n" % str)
    sys.stdout.flush()

def r_apdu(str):
    sys.stderr.write("R-APDU:%s\n\n" % str)
    to_simtrace(str)


# FIXME: iso7816 parser control is currently embedded in R-APDU packet
# with SW=0xFFFF

def command(tag, payload=[0,0,0,0]):
    b = tag
    b.extend(payload)
    b.extend([0xFF, 0xFF])
    str = bytes2hex(b)
    sys.stderr.write("COMMAND:%s\n\n" % str)
    to_simtrace(str)

CMD_SET_ATR  = [0,0,0,0]
CMD_SET_SKIP = [1,0,0,0]
CMD_HALT     = [2,0,0,0]


def mainloop():
    while 1:
        r_apdu(apdu_hex(c_apdu()))

command(CMD_SET_ATR, c.getATR())
command(CMD_SET_SKIP, [2,0,0,0])
command(CMD_HALT)

mainloop()
# execfile("mim.py")



