# SIMtrace APDU service providers.  (e.g. SIM cards, MITMs, ...)

# These classes implement the handler interface passed to an apdufw.forwarder object:
#
# .apdu(bytelist) -> bytelist
# .getATR() -> bytelist
# .reset()

import smartcard

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



