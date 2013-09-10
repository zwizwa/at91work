# SIMtrace APDU service providers.  (e.g. SIM cards, MITMs, ...)

# These classes implement the handler interface passed to an apdufw.forwarder object:
#
# .apdu(bytelist) -> bytelist
# .getATR() -> bytelist
# .reset()

import smartcard
import hextools
import sys
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
    def __init__(self, index=0):
        self.connect(index)
        # self.force_SIM = True
        self.force_SIM = False

    def connect(self, index=0):
        readers = smartcard.System.readers()
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
    def apdu(self, c_apdu):

        # Force SIM protocol for USIM cards.
        if (self.force_SIM):
            if (c_apdu[0] != 0xA0):
                return [0x6E, 0x00]

        # Delegate
        (data,sw1,sw2) = self.card.transmit( list(c_apdu) )
        return pack(data,sw1,sw2)



class pySim_serial:
    def __init__(self, dev):
        self.connect(dev)

    def connect(self, dev):
        self.sl = SerialSimLink(device=dev)
        self.sl.wait_for_card()

    def apdu(self, msg):
        r, s = self.sl.send_apdu(hextools.hex(msg))
        rapdu = hextools.bytes(r)
        rapdu.extend(hextools.bytes(s))
        return rapdu

    def disconnect(self):
        raise Exception("NI: disconnect()")

    def getATR(self):
        raise Exception("NI: getATR()")

