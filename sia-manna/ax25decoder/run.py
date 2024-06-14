# AX.25 Decoder Python Script
# The frame consists of the following:
# Preamble (8 bytes) - 8 * 0x7E
# Flag (1 byte) - 0x7E
# Destination Callsign (6 bytes) - XX0UHF
# Destination SSID (1 byte) - e1
# Source Callsign (6 bytes) - 0000CQ
# Source SSID (1 byte) - e0
# Control (1 byte) - 0x03
# PID (1 byte) - 0xF0
# Payload (0 - 77 bytes) - upto 616
# FCS (2 bytes) - 
# Flag (1 byte) - 0x7E
# Postamble (3 bytes) - 3 * 0x7E
# Total = 108 bytes

#Check if frame is > 128


import hexdump  # Import hexdump module for displaying binary data in a hex + ASCII format
import struct  # Import struct module for working with C-style data structures

# Function to decode AX.25 address fields (14 bytes)
# Destination Callsign (6 bytes) - XX0UHF
# Destination SSID (1 byte) - e1
# Source Callsign (6 bytes) - 0000CQ
# Source SSID (1 byte) - e0
def decode_addr(data, cursor):
    # Unpack 7 bytes starting from the cursor position
    (a1, a2, a3, a4, a5, a6, a7) = struct.unpack("<BBBBBBB", data[cursor:cursor+7])
    
    hrr = a7 >> 5  # Highest Receive Ready bits, extracted from the last 3 bits of a7
    ssid = (a7 >> 1) & 0xf  # SSID (Secondary Station Identifier), extracted from bits 1-4 of a7
    ext = a7 & 0x1  # Extension bit, the least significant bit of a7

    # Shift right 1 bit for each byte of the address, then pack and decode as ASCII, removing null bytes
    addr = struct.pack("<BBBBBB", a1 >> 1, a2 >> 1, a3 >> 1, a4 >> 1, a5 >> 1, a6 >> 1)
    addr = addr.replace(b'\x00', b'').decode('ascii')  # Remove null bytes and decode to ASCII
    if ssid != 0:
        call = "{}-{}".format(addr.strip(), ssid)  # Format callsign with SSID if SSID is not zero
    else:
        call = addr.strip()  # Otherwise, just use the callsign
    return (call, hrr, ext)  # Return the callsign, hrr, and extension bit

#def decode_addr_test(data, cursor):
    #Unpack 7 bytes starting from cursor position
    #(b1, b2, b3, b4, b5, b6, b7) = struct.unpack("<BBBBBBB", data[cursor:cursor+7])

# Decrambler function
def decrambler(frame):
    descrambled = bytearray()
    state = 0

    for byte in frame:
        for i in range(8):
            feedback = (state ^ (byte >> i)) & 1
            state = ((state >> 1) | (feedback << 6)) & 0x7F
            descrambled.append((byte >> i) ^ feedback)

    return bytes(descrambled)

# NRZI Decoder
def nrzi_decoder(data):
    decoded = bytearray()
    prev_bit = 0

    for byte in data:
        for i in range(8):
            curr_bit = (byte >> i) & 1
            decoded.append(curr_bit ^ prev_bit)
            prev_bit = curr_bit

    return bytes(decoded)

# Bit Destuffing function
def bit_destuff(data):
    destuffed = bytearray()
    count = 0

    for byte in data:
        for i in range(8):
            bit = (byte >> i) & 1
            if bit == 1:
                count += 1
            else:
                count = 0
            destuffed.append(bit)
            if count == 5:
                i += 1
                count = 0

    return bytes(destuffed)



# Function to decode U frames (Unnumbered frames) in AX.25 protocol
# This is the function we need
def decode_uframe(ctrl, data, pos):
    print("U Frame")
    if ctrl == 0x3:  # UI frame control field value
        # Unpack 1 byte for PID (Protocol Identifier) - 0xF0
        (pid,) = struct.unpack("<B", data[pos:pos+1])
        pos += 1
        rem = len(data[pos:-2])  # Remaining data length excluding FCS (Frame Check Sequence)
        info = struct.unpack("<" + "B"*rem, data[pos:-2])  # Unpack remaining data as bytes
        pos += rem
        fcs = struct.unpack("<BB", data[pos:pos+2])  # Unpack FCS as 2 bytes
        print("PID: 0x{:02x}".format(pid))  # Print PID in hex format
        print("PAYLOAD: " + struct.pack("<" + "B"*len(info), *info).decode('ascii', errors='replace'))  # Print INFO field, decode ASCII with error replacement
        print("FCS: 0x{:02x}{:02x}".format(fcs[0], fcs[1]))  # Print FCS in hex format

def to_binary_string(byte_data):
    return ' '.join(format(byte, '08b') for byte in byte_data)

# def check_frame_length(frame):
#     if len(frame) > 128:
#         print("Error: Frame length exceeds 128 bytes.")
#         return False
#     return True

# Placeholder functions for S frames and I frames (currently commented out because we don't need them)
""" def decode_sframe(ctrl, data, pos):
    print("S Frame")

def decode_iframe(ctrl, data, pos):
    print("I Frame") """

# Main function to process the AX.25 frame
def p(frame):

    pos = 0

    # Decramble the frame
    #frame = decrambler(frame)

    # Decode NRZI
    #frame = nrzi_decoder(frame)

    #if check_frame_length(frame) == False:
        #return "Invalid frame"

    # Decode preamble
    preamble = frame[pos:pos+8]
    pos += 8
    #print("Preamble: " + preamble.hex())
    if preamble != b'\x7e' * 8:
        print("Invalid preamble. Expected 8 bytes of 0x7E.")
    print("Preamble: " + preamble.hex())

    #Decode start flag
    start_flag = frame[pos:pos+1]
    pos += 1
    if start_flag != b'\x7e':
        print("Invalid start flag. Expected 0x7E.")
    print("Start Flag: " + start_flag.hex())

    # Perform bit destuffing
    #frame = bit_destuff(frame)

    # Decode destination address
    (dest_addr, dest_hrr, dest_ext) = decode_addr(frame, pos)
    pos += 7
    print("DST: " + dest_addr)
    
    # Decode source address
    (src_addr, src_hrr, src_ext) = decode_addr(frame, pos)  
    pos += 7
    print("SRC: " + src_addr)
    
    # Decode repeater addresses (if any)
    """ext = src_ext
    while ext == 0:
        rpt_addr, rpt_hrr, ext = decode_addr(frame, pos)
        print("RPT: " + rpt_addr)
        pos += 7"""

    # Decode control field
    (ctrl,) = struct.unpack("<B", frame[pos:pos+1])
    pos += 1
    if ctrl != b'\x03':
        print("Invalid control. Expected 0x03.")
    print("CTRL: 0x{:02x}".format(ctrl))

    # Determine frame type and decode accordingly
    if (ctrl & 0x3) == 0x3:
        decode_uframe(ctrl, frame, pos)  # U frame
    else:
        print("This is not a UI frame and there is an error in decoding")
    #elif (ctrl & 0x3) == 0x1:
         #decode_sframe(ctrl, frame, pos)  # S frame (currently commented out)
    #elif (ctrl & 0x1) == 0x0:
         #decode_iframe(ctrl, frame, pos)  # I frame (currently commented out)

    # Print the entire frame in hexdump format
    #print(frame)
    print(hexdump.hexdump(frame))

# Main entry point of the script
if __name__ == "__main__":
    # Commented out code for reading packet data from a file
    # f = open("test.txt", "r")
    # hex_str = f.read()
    # print(hex_str)

    # Example hex string containing an AX.25 frame

    #Frame containing payload "Hello World"
    hex_str = "7e7e7e7e7e7e7e7e7e6060606086a260b0b060aa908c6203f048656c6c6f20576f726c6421202020202020202020202020202020202020202020202020202020202020202020202020202020202020202020202020202020202020202020202020202020202012347e7e7e7e"
    
    #Frame containing payload "Hello, this is a test message for AX.25 frame with 77 bytes payload."
    #hex_str= "7e7e7e7e7e7e7e7e7ec086888a404060e0888a4040406103f048656c6c6f2c207468697320697320612074657374206d65737361676520666f722041582e3235206672616d652077697468203737206279746573207061796c6f61642e20202020202020202012347e7e7e7e"

    #The actual frame we want to decode
    #hex_str = "fef16e90a0bca56afaf1fece452a2698266d24d78467d4643fc1315d6265c5c46bf624cbf46f2995f7971daf172bac7a450c271137ceb25929eb5f150b8f6864672114d492763b0df80c2991d8831165cbbbb386377539bd525ed997ae48b6fa618e7d1ce07fc87cdc"

    frame = bytes.fromhex(hex_str)  # Convert hex string to bytes
    p(frame)  # Process the frame


    """
    Preamble: 7e7e7e7e7e7e7e7e
    Start Flag: 7e
    DST: 0000CQ
    SRC: XX0UHF-1
    Invalid control. Expected 0x03.
    CTRL: 0x03
    U Frame
    PID: 0xf0
    PAYLOAD: Hello World!                                                                 4~~
    FCS: 0x7e7e
    00000000: 7E 7E 7E 7E 7E 7E 7E 7E  7E 60 60 60 60 86 A2 60  ~~~~~~~~~````..`
    00000010: B0 B0 60 AA 90 8C 62 03  F0 48 65 6C 6C 6F 20 57  ..`...b..Hello W
    00000020: 6F 72 6C 64 21 20 20 20  20 20 20 20 20 20 20 20  orld!           
    00000030: 20 20 20 20 20 20 20 20  20 20 20 20 20 20 20 20                  
    00000040: 20 20 20 20 20 20 20 20  20 20 20 20 20 20 20 20                  
    00000050: 20 20 20 20 20 20 20 20  20 20 20 20 20 20 20 20                  
    00000060: 20 20 20 20 20 20 12 34  7E 7E 7E 7E                    .4~~~~
    None
    """

