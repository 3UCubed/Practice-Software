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
import conversions #Import my personal library to facilitate datatype conversions


# Function to decode AX.25 address fields (14 bytes)
# Destination Callsign (6 bytes) - XX0UHF
# Destination SSID (1 byte) - e1
# Source Callsign (6 bytes) - 0000CQ
# Source SSID (1 byte) - e0
# The address is 6 bytes + 1 SSID byte, each character is shifted left by 1
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
# Expects hex
def descrambling(frame):
    descrambled = bytearray()
    state = 0

    for byte in frame:
        for i in range(8):
            feedback = (state ^ (byte >> i)) & 1
            state = ((state >> 1) | (feedback << 6)) & 0x7F
            descrambled.append((byte >> i) ^ feedback)

    return bytes(descrambled)

# NRZI Decoder
def nrzi_decoding(data):
    decoded = bytearray()
    prev_bit = 0

    for byte in data:
        for i in range(8):
            curr_bit = (byte >> i) & 1
            decoded.append(curr_bit ^ prev_bit)
            prev_bit = curr_bit

    #return bytes(decoded)
    return decoded

# Bit Destuffing function
def bit_destuffing(data):
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

def process_frame(frame):
    # Ensure the frame is in bytes
    if isinstance(frame, bytes):
        # Convert bytes to hexadecimal string
        hex_string = frame.hex()
        try:
            # Convert hexadecimal string to integer
            integer_value = int(hex_string, 16)
            # Format the integer to a binary string with at least 128 bits
            binary_string = format(integer_value, "0128b")
            return binary_string
        except ValueError as e:
            print(f"Error converting frame to integer: {e}")
            return None
    else:
        print("Frame is not in bytes format.")
        return None


# Main function to process the AX.25 frame
def p(frame):

    position = 0

    print("FRAME: ", frame)

    # Decramble the frame
    #frame = descrambler(frame)
    descrambled = descrambling(frame)

    print ("Descrambled frame: ", descrambled)

    # Decode NRZI
    nrzi_decoded = nrzi_decoding(descrambled)
    print ("NRZI decoded frame: ", nrzi_decoded)

    #bit_stuff_string = bit_destuffed.decode('utf-8')

    bit_destuffed = bit_destuffing(nrzi_decoded)
    print("Bit-destuffed (without extracting mid frame): ", bit_destuffed)

    #frame = bit_destuffed
    #binframe = conversions.binary_to_hexadecimal(frame)
    print("Top Hex string: ", conversions.binary_to_hexadecimal(frame))


    #print("Type for bitstudded: ", type(bit_destuffed))

    """ bit_destuff_ready = extractmidframe(nrzi_decoded)
    print("Frame ready for bitstuffing: ", bit_destuff_ready)

    bit_stuffed = bit_destuff(bit_destuff_ready)
    print("Bit-stuffed frame: ", bit_stuffed)
 """

    #if check_frame_length(frame) == False:
        #return "Invalid frame"

# Decode preamble
    preamble = frame[position:position+8]
    position += 8
    #print("Preamble: " + preamble.hex())
    if preamble != b'\x7e' * 8:
        print("Invalid preamble. Expected 8 bytes of 0x7E.")
    print("Preamble: " + preamble.hex())

    #Decode start flag
    start_flag = frame[position:position+1]
    position += 1
    if start_flag != b'\x7e':
        print("Invalid start flag. Expected 0x7E.")
    print("Start Flag: " + start_flag.hex()) 

    # Perform bit destuffing
    """ if (position >= 9):
        frame = bit_destuff(frame) """

    # Decode destination address
    (dest_addr, dest_hrr, dest_ext) = decode_addr(frame, position)
    position += 7
    print("DST: " + dest_addr)
    
    # Decode source address
    (src_addr, src_hrr, src_ext) = decode_addr(frame, position)  
    position += 7
    print("SRC: " + src_addr)
    
    # Decode repeater addresses (if any)
    """ext = src_ext
    while ext == 0:
        rpt_addr, rpt_hrr, ext = decode_addr(frame, pos)
        print("RPT: " + rpt_addr)
        pos += 7"""

    # Decode control field
    (ctrl,) = struct.unpack("<B", frame[position:position+1])
    position += 1
    if ctrl != b'\x03':
        print("Invalid control. Expected 0x03.")
    print("CTRL: 0x{:02x}".format(ctrl))

    # Determine frame type and decode accordingly
    if (ctrl & 0x3) == 0x3:
        decode_uframe(ctrl, frame, position)  # U frame
    else:
        print("This is not a UI frame and there is an error in decoding")
    #elif (ctrl & 0x3) == 0x1:
         #decode_sframe(ctrl, frame, pos)  # S frame (currently commented out)
    #elif (ctrl & 0x1) == 0x0:
         #decode_iframe(ctrl, frame, pos)  # I frame (currently commented out)

    # Print the entire frame in hexdump format
    
    #print(hex(binframe))
    #print(frame)
    #print(hexdump.hexdump(binframe))
    print(hexdump.hexdump(frame))
    print(hexdump.hexdump(bit_destuffed))

# Main entry point of the script
if __name__ == "__main__":
    # Commented out code for reading packet data from a file
    # f = open("test.txt", "r")
    # hex_str = f.read()
    # print(hex_str)

    # Example hex string containing an AX.25 frame

    #

    #This is the raw frame and not AX.25 encoded (i.e. it is NOT bit stuffed, NRZI encoded or scrambled per G3RUH/K9NG and UoSAT-14/22/23)
    #Frame containing payload "Hello World"
    #correct_output = "7e7e7e7e7e7e7e7e7e6060606086a260b0b060aa908c6203f048656c6c6f20576f726c6421202020202020202020202020202020202020202020202020202020202020202020202020202020202020202020202020202020202020202020202020202020202012347e7e7e7e"

    #Although this frame is AX.25 encoded, we do not think it contains the AX.25 pramble (8* 0x7E) and post amble (3* 0x7E)
    #Black Magic
    #hex_str = "6060606086A2E0B0B060AA908C6103F054686520717569636B2062726F776E20666F78206A756D7073206F76657220746865206C617A7920646F67"
    
    #This is the raw frame and not AX.25 encoded
    #Frame containing payload "Hello, this is a test message for AX.25 frame with 77 bytes payload."
    #hex_str= "7e7e7e7e7e7e7e7e7ec086888a404060e0888a4040406103f048656c6c6f2c207468697320697320612074657374206d65737361676520666f722041582e3235206672616d652077697468203737206279746573207061796c6f61642e20202020202020202012347e7e7e7e"

    #This is the actual frame we are recieving on the GS emulator radio in our lab. This emulator radio uses UHF type II packet decoder from my past mission (i.e. it should only spit out the Payload part– upto 128 bytes); but we are not totally sure if that is the case, and it is under investigation.
    #The actual frame we want to decode
    hex_str = "fef16e90a0bca56afaf1fece452a2698266d24d78467d4643fc1315d6265c5c46bf624cbf46f2995f7971daf172bac7a450c271137ceb25929eb5f150b8f6864672114d492763b0df80c2991d8831165cbbbb386377539bd525ed997ae48b6fa618e7d1ce07fc87cdc"

    #Converted to utf-8
    #hex_str = "66 65 66 31 36 65 39 30 61 30 62 63 61 35 36 61 66 61 66 31 66 65 63 65 34 35 32 61 32 36 39 38 32 36 36 64 32 34 64 37 38 34 36 37 64 34 36 34 33 66 63 31 33 31 35 64 36 32 36 35 63 35 63 34 36 62 66 36 32 34 63 62 66 34 36 66 32 39 39 35 66 37 39 37 31 64 61 66 31 37 32 62 61 63 37 61 34 35 30 63 32 37 31 31 33 37 63 65 62 32 35 39 32 39 65 62 35 66 31 35 30 62 38 66 36 38 36 34 36 37 32 31 31 34 64 34 39 32 37 36 33 62 30 64 66 38 30 63 32 39 39 31 64 38 38 33 31 31 36 35 63 62 62 62 62 33 38 36 33 37 37 35 33 39 62 64 35 32 35 65 64 39 39 37 61 65 34 38 62 36 66 61 36 31 38 65 37 64 31 63 65 30 37 66 63 38 37 63 64 63"


    #Frame generated by my encoder
    #hex_str = "7e7e7e7e7e7e7e7e7e6060606086a260b0b060aa908c6203f048656c6c6f20576f726c6421202020202020202020202020202020202020202020202020202020202020202020202020202020202020202020202020202020202020202020202020202020202012347e7e7e7e"

    #HDLC Frame
    #hex_str = "7E060606066145070D0D0655093186C00F12A63636F604EAF64E36268439307E"

    #encoder.py made frame
    #hex_str = "55AA55AA55AA55AA55E619E619344FCC590CB3BB8C68319FD9C61DE9E9EA4C8A15F1E9E33333333333333333333333333333333333333333333333333333333990601FE01FE"
    #Modified encoder.py made frame to evade ValueError: non-hexadecimal number found in fromhex() arg at position
    #hex_str = "55AA55AA55AA55AA55E619E619344FCC590CB3BB8C68319FD9C61DE9E9EA4C8A15F1E9E33333333333333333333333333333333333333333333333333333333990601FE01F0E"


    frame = bytes.fromhex(hex_str)  # Convert hex string to bytes
    print("FRAME: " , frame)
    p(frame)  # Process the frame

    # When I comment out the AX.25 decoding operations (bit-destuffing, NRZI decoding, and descrambling)
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

