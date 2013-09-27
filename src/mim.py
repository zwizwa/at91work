#!/usr/bin/python
# LICENSE: GPL2
# (c) 2013 Tom Schouten <tom@getbeep.com>


# Example MIM: Delegate phone APDU requests to pyscard.

import apdufw   # Phone <-> APDU service provider
import apdusrv  # APDU service provider implementations

import hextools

# SIMtrace hardware uses 3V signaling which might give communication
# problems if the phone decides to use a different voltage.  The ATR
# byte TA(3) allows the voltage class to be limited.  See:
# http://lists.osmocom.org/pipermail/simtrace/2013-September/000554.html
atr_hack = hextools.bytes("3B9F95801FC28031E073FE2113574A330530323402A6")
#                                  ..C7.. on the original card

simcard = apdusrv.pyscard_smartcard(0)
mim     = apdufw.forwarder(srv = simcard,
                           atr = atr_hack)

mim.run()


# Notes:
#
#  - SIMtrace HW currently has no way to detect phone interface
#    voltage levels.  For reliable operation, change the ATR to
#    support only voltage class B = 3V3, e.g:
#
#      ORIG http://smartcard-atr.appspot.com/parse?ATR=3B9F95801FC78031E073FE2113574A330530323402A3
#      MOD  http://smartcard-atr.appspot.com/parse?ATR=3B9F95801FC28031E073FE2113574A330530323402A6


