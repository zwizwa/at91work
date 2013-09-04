#!/usr/bin/python

# Example MIM: Delegate phone APDU requests to pyscard.

import apdufw   # Phone <-> APDU service provider
import apdusrv  # APDU service provider implementations

simcard = apdusrv.pyscard_smartcard(0)
mim     = apdufw.forwarder(srv = simcard)

mim.run()




