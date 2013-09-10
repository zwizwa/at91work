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
def le_u32(val):
    return [val & 0xFF,
            (val >> 8) & 0xFF,
            (val >> 16) & 0xFF,
            (val >> 24) & 0xFF];
def u32(val):
    return le_u32(val)

def be_u16(val):
    return [(val >> 8) & 0xFF, val & 0xFF]


# big endian byte list to integer
def le_int(lst):
    shift = 0
    acc = 0
    for b in lst:
        acc += b << shift
        shift += 8
    return acc

def be_int(lst):
    r_lst = list(lst)
    r_lst.reverse()
    return le_int(r_lst)

def all_FF(msg):
    for b in msg:
        if b != 0xFF:
            return False
    return True


def test():
    print "BE %08X" % be_int([0,1,2,3])
    print "LE %08X" % le_int([0,1,2,3])

if __name__ == '__main__':
    test()
