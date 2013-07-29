#!/usr/bin/python


# Bare-bones MIM - Delegate phone APDU requests to pyscard.
#
# The iso7816_slave firmware on the SIMtrace presents an ACM device.
# Connect it to this script, e.g. using:
#   socat /dev/ttyACM0 EXEC:./mim.py
# 


import sys
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

# Perform APDU request on card
def apdu(msg):
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

def r_apdu(str):
    sys.stderr.write("R-APDU:%s\n\n" % str)
    sys.stdout.write("%s\n" % str)
    sys.stdout.flush()


def mainloop():
    while 1:
        r_apdu(apdu_hex(c_apdu()))


mainloop()
# execfile("mim.py")



