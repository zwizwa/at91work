# SIMtrace APDU forwarder, host side.
# LICENSE: GPL2
# (c) 2013 Tom Schouten <tom@getbeep.com>

import sys
import sym
import subprocess

import gsmtap
import hextools
import usb

# SIMtrace slave commands and events, see iso7816_slave.h 
CMD_SET_ATR  = 0
CMD_SET_SKIP = 1
CMD_HALT     = 2
CMD_POLL     = 3
CMD_R_APDU   = 4

EVT_RESET    = 2
EVT_C_APDU   = 4

def usb_find(idVendor, idProduct):
    busses = usb.busses()
    for bus in busses:
        devices = bus.devices
        for dev in devices:
            if ((dev.idVendor  == idVendor) and
                (dev.idProduct == idProduct)):
                return dev
    raise Exception("SIMtrace not found at %04X:%04X" % (idVendor, idProduct))

# FIXME: remove this - use wireshark gsmtap instead.
def pretty_apdu(msg, log=sys.stderr.write):
  try:  
    ins = msg[1]
    try:
        log(" %s" % sym.iso7816[ins])
    except:
        log(" %02X" % ins)
        pass
    if (msg[1] == 0xA4): # SELECT_FILE
        file = msg[5] * 256 + msg[6]
        try:
            log(" %s" % sym.USIM_FID[file])
        except:
            try:
                log(" %s" % sym.SIM_FID[file])
            except:
                log(" %04X" % file)
                pass
            pass
    log("\n")
  except:
    pass

class default_console:
    def __init__(self):
        pass
    def poll(self):
        pass

class forwarder:
    def __init__(self, srv,
                 idVendor=0x03eb,
                 idProduct=0x6119,
                 log=sys.stderr.write,
                 verbose=2,
                 log_apdu_prefix="APDU:",
                 console=default_console(),
                 atr=None,
                 tracefile=False):
        self.verbose = verbose
        self.console = console
        self.tracefile = tracefile
        self.log_apdu_prefix = log_apdu_prefix
        self.srv = srv
        self.log = log
        self.atr = atr
        self.dev = usb_find(idVendor, idProduct)
        self.dh = self.dev.open()

    def usb_ctrl_OUT(self, req, buf):
        return self.dh.controlMsg(0x40,
                                  request=req,    # R-APDU
                                  buffer=buf,
                                  timeout=500)
    def usb_ctrl_IN(self, req):
        return self.dh.controlMsg(0xC0,
                                  request=req,
                                  buffer=512,
                                  timeout=500)
    def log(self, msg):
        sys.stderr.write(msg)

    def c_apdu(self):
        while True:
            msg = []
            # FIXME: This is the main event loop.  Move it to top level.
            while (not len(msg)):
                self.console.poll()
                msg = list(self.usb_ctrl_IN(CMD_POLL))

            evt = msg[0]
            if (evt == EVT_C_APDU):
                data = msg[4:]
                return data

            if (evt == EVT_RESET):
                if self.verbose>0:
                    self.log("RESET CARD\n")
                self.srv.reset()
            else:
                self.log("unknown event: %s\n" % hextools.bytes2hex(msg))

    def r_apdu(self, msg):
        self.usb_ctrl_OUT(CMD_R_APDU, msg)

    def command(self, tag, payload=[]):  # dummy byte
        if self.verbose>0:
            self.log("CMD %d %s\n" % (tag, hextools.bytes2hex(payload)))
        self.usb_ctrl_OUT(tag, payload)



    def tick(self):
        c = self.c_apdu()
        r = self.srv.apdu(c)
        self.r_apdu(r)
        gsmtap.log(c,r)
        h_c = hextools.bytes2hex(c)
        h_r = hextools.bytes2hex(r)
        # Human readable debugging
        if (self.verbose>1):
            self.log("C-APDU:%s\n" % h_c)
            pretty_apdu(c, self.log)
            self.log("R-APDU:%s\n" % h_r)
        # C/R trace file
        if (self.tracefile):
            f = open(self.tracefile, 'a')
            f.write("C-APDU:%s\n" % h_c)
            f.write("R-APDU:%s\n" % h_r)
            f.close()
            
        # This is useful for attaching an APDU parser on console.
        if (self.log_apdu_prefix):
            self.log("%s%s%s\n" % (self.log_apdu_prefix, h_c, h_r))


    def mainloop(self):
        while 1:
            self.tick()

    def reboot(self):
        adb = "/opt/xc/android/android-sdk-linux/platform-tools/adb"
        self.log("adb shell reboot\n")
        subprocess.call([adb, "shell", "reboot"])

    def getATR(self):
        if self.atr is not None:
            return self.atr
        else:
            return self.srv.getATR()

    def run(self):
        self.command(CMD_SET_ATR, self.getATR())
        self.command(CMD_SET_SKIP, hextools.u32(1))
        self.command(CMD_HALT)

        # start handling incoming phone C-APDUs
        self.reboot()
        self.mainloop()

