# SIMtrace APDU service providers.  (e.g. SIM cards, MITMs, ...)
# LICENSE: GPL2
# (c) 2013 Tom Schouten <tom@getbeep.com>

# These classes implement the handler interface passed to an apdufw.forwarder object:
#
# .apdu(bytelist) -> bytelist
# .getATR() -> bytelist
# .reset()

import smartcard
import hextools
import sys
import sym
log = sys.stderr.write
try:
    from pySim.transport.serial import SerialSimLink
except:
    log("FIXME: pySim.transport.serial not found\n")



def pack(reply,sw1,sw2):
    p = list(reply)
    p.append(sw1)
    p.append(sw2)
    return p



class pyscard_smartcard:
    def __init__(self, index=0, log=sys.stderr.write, verbose=0):
        self.verbose = verbose
        self.log = log
        self.connect(index)
        # self.force_SIM = True
        self.force_SIM = False

    def connect(self, index=0):
        readers = smartcard.System.readers()
        if self.verbose>0:
            log("Connecting to reader: %s\n" %readers[index])
        # print r # => ['SCM Microsystems Inc. SCR 331 [CCID Interface] (21121250210402) 00 00']
        self.card = readers[index].createConnection()
        self.card.connect()

    def disconnect(self):
        self.card.disconnect()
    
    def reset(self):
        self.disconnect()
        self.connect()

    def getATR(self):
        return self.card.getATR()

    # Perform APDU request on card
    # Perform APDU request on card
    def apdu(self, c_apdu):
        c_apdu = hextools.bytes(c_apdu)

        # Force SIM protocol for USIM cards.
        if (self.force_SIM):
            if (c_apdu[0] != 0xA0):
                return [0x6E, 0x00]

        # Delegate
        (data,sw1,sw2) = self.card.transmit( list(c_apdu) )
        return pack(data,sw1,sw2)


# It is convenient to map tty device names to the physical USB port of
# the converter.  See the `ttyUSB_id` script in SIM/bin/

class pySim_serial:
    def __init__(self, dev, log=sys.stderr.write):
        self.log = log
        self.dev = dev
        self.reset()

    def apdu(self, msg):
        e_msg = ""
        retries = 3
        for i in range(retries):
            try:
                r, s = self.sl.send_apdu(hextools.hex(msg))
                rapdu = hextools.bytes(r) + hextools.bytes(s)
                return rapdu
            except Exception as e:
                e_msg = e.message
                print e.message
                self.log("WARNING: pySim_serial.apdu(): %s\n" % e_msg)
                self.reset()
        raise Exception("pySim_serial.apdu(): %s" % e_msg)
                

    def reset(self):
        self.sl = SerialSimLink(self.dev)
        self.sl.wait_for_card()

    def getATR(self):
        # HACK: return 3V3 ATR from other card.
        # FIXME: Modify pySim code to save ATR bytes.
        return hextools.bytes("3B9F95801FC28031E073FE2113574A330530323402A6")


