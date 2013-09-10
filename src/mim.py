#!/usr/bin/python

# Example MIM: Delegate phone APDU requests to pyscard.

import apdufw   # Phone <-> APDU service provider
import apdusrv  # APDU service provider implementations

simcard = apdusrv.pyscard_smartcard(0)
mim     = apdufw.forwarder(srv = simcard)

mim.run()


# Notes:
#
#  - SIMtrace HW currently has no way to detect phone interface
#    voltage levels.  For reliable operation, change the ATR to
#    support only voltage class B = 3V3, e.g:
#
#      ORIG http://smartcard-atr.appspot.com/parse?ATR=3B9F95801FC78031E073FE2113574A330530323402A3
#      MOD  http://smartcard-atr.appspot.com/parse?ATR=3B9F95801FC28031E073FE2113574A330530323402A6


