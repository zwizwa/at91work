def bytes2hex(bytes):
    return "".join(map(lambda v: "%02X"%v, bytes))

def hex2bytes(hex):
    return map(ord,hex.decode("hex"))

def bytes(x):
    if type(x) == str:
        x = hex2bytes(x)
    return x

def hex(x):
    if type(x) != str:
        x = bytes2hex(x)
    return x

# see iso7816_slave.h
# AT91 is little endian.
def u32(val):
    return [val & 0xFF,
            (val >> 8) & 0xFF,
            (val >> 16) & 0xFF,
            (val >> 24) & 0xFF];
