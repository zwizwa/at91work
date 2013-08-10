import usb

def find(idVendor, idProduct):
    busses = usb.busses()
    for bus in busses:
        devices = bus.devices
        for dev in devices:
            if ((dev.idVendor  == idVendor) and
                (dev.idProduct == idProduct)):
                return dev
    return 'wrong'

dev = find(idVendor=0x03eb, idProduct=0x6119)
# dev = find(idVendor=0x03eb, idProduct=0x6129)
dh  = dev.open()
# dh.detachKernelDriver(0)


def claim():
    dh.claimInterface(0)


# VENDOR requests

def ctrl_IN():
    return dh.controlMsg(0xC0,
                         request=123,
                         buffer=4,
                         value=0,
                         index=0,
                         timeout=500)
def ctrl_OUT():
    buf=[1,2,3,4]
    rv=dh.controlMsg(0x40,
                     request=123,
                     buffer=buf,
                     value=0,
                     index=0,
                     timeout=500)
    return rv
    




