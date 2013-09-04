#!/usr/bin/python

# Bare-bones MIM - Delegate phone APDU requests to pyscard.

import apdufw   # Phone <-> APDU service provider
import apdusrv  # APDU service provider implementations

simcard = apdusrv.pyscard_smartcard(0)
mitm    = apdufw.forwarder(srv = simcard)

mitm.run()













