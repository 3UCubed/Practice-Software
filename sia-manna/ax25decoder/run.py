import hexdump
import struct

def decode_addr(data, cursor):
    (a1, a2, a3, a4, a5, a6, a7) = struct.unpack("<BBBBBBB", data[cursor:cursor+7])
    hrr = a7 >> 5
    ssid = (a7 >> 1) & 0xf     
    ext = a7 & 0x1

    addr = struct.pack("<BBBBBB", a1 >> 1, a2 >> 1, a3 >> 1, a4 >> 1, a5 >> 1, a6 >> 1)
    addr = addr.replace(b'\x00', b'').decode('ascii')  # Remove null bytes and decode to ASCII
    if ssid != 0:
      call = "{}-{}".format(addr.strip(), ssid)
    else:
      call = addr.strip()
    return (call, hrr, ext)

def decode_uframe(ctrl, data, pos):
    print("U Frame")
    if ctrl == 0x3:
        # UI frame
        (pid,) = struct.unpack("<B", data[pos:pos+1])
        pos += 1
        rem = len(data[pos:-2])
        info = struct.unpack("<" + "B"*rem, data[pos:-2])
        pos += rem
        fcs = struct.unpack("<BB", data[pos:pos+2])
        print("PID: 0x{:02x}".format(pid))
        print("INFO: " + struct.pack("<" + "B"*len(info), *info).decode('ascii', errors='replace'))
        print("FCS: 0x{:02x}{:02x}".format(fcs[0], fcs[1]))

def decode_sframe(ctrl, data, pos):
    print("S Frame")

def decode_iframe(ctrl, data, pos):
    print("I Frame")

def p(frame):
    pos = 0

    # DST
    (dest_addr, dest_hrr, dest_ext) = decode_addr(frame, pos)
    pos += 7
    print("DST: " + dest_addr)
    
    # SRC
    (src_addr, src_hrr, src_ext) = decode_addr(frame, pos)  
    pos += 7
    print("SRC: " + src_addr)
    
    # REPEATERS
    ext = src_ext
    while ext == 0:
        rpt_addr, rpt_hrr, ext = decode_addr(frame, pos)
        print("RPT: " + rpt_addr)
        pos += 7

    # CTRL
    (ctrl,) = struct.unpack("<B", frame[pos:pos+1])
    pos += 1
    print("CTRL: 0x{:02x}".format(ctrl))

    if (ctrl & 0x3) == 0x3:
        decode_uframe(ctrl, frame, pos)
    elif (ctrl & 0x3) == 0x1:
        decode_sframe(ctrl, frame, pos)
    elif (ctrl & 0x1) == 0x0:
        decode_iframe(ctrl, frame, pos)

    print(hexdump.hexdump(frame))

if __name__ == "__main__":
    #f = open("test.txt", "r")
    #hex_str = f.read()
    #print(hex_str)
    hex_str = "fef16e90a0bca56afaf1fece452a2698266d24d78467d4643fc1315d6265c5c46bf624cbf46f2995f7971daf172bac7a450c271137ceb25929eb5f150b8f6864672114d492763b0df80c2991d8831165cbbbb386377539bd525ed997ae48b6fa618e7d1ce07fc87cdc"
    frame = bytes.fromhex(hex_str)
    p(frame)
