import struct

def encode_callsign(callsign, ssid):
    # Callsign should be 6 characters, pad with spaces if shorter
    callsign = callsign.ljust(6)
    # Encode each character by shifting left 1 bit
    encoded_callsign = [(ord(char) << 1) for char in callsign]
    # Encode the SSID
    encoded_ssid = (ssid & 0x0F) << 1 | 0x60
    # Combine the encoded callsign and SSID
    return struct.pack('<6B', *encoded_callsign) + struct.pack('B', encoded_ssid)

def calculate_fcs(frame):
    # Placeholder FCS calculation function (real implementation needed for actual use)
    # For simplicity, let's return a fixed FCS here
    return b'\x12\x34'

def construct_ax25_frame():
    # Components of the frame
    preamble = b'\x7E' * 8
    start_flag = b'\x7E'
     # Encode the addresses
    dest_addr = encode_callsign("0000CQ", 0xe0)
    src_addr = encode_callsign("XX0UHF", 0xe1)
    #dest_addr = struct.pack('<BBBBBBB', 0xC0, 0x86, 0x88, 0x8A, 0x40, 0x40, 0x60)
    #src_addr = struct.pack('<BBBBBBB', 0xE0, 0x88, 0x8A, 0x40, 0x40, 0x40, 0x61)
    control = b'\x03'
    pid = b'\xF0'
    
    # Example payload (11 bytes, "Hello World!", and then padded to 77 bytes)
    payload = b'Hello World!'
    payload += b' ' * (77 - len(payload))  # Pad the payload to ensure it's 77 bytes
    
    # Construct the initial part of the frame (excluding FCS)
    frame = preamble + start_flag + dest_addr + src_addr + control + pid + payload
    
    # Calculate the FCS for the frame (excluding the preamble and flags)
    #fcs = calculate_fcs(frame[8:8+1+len(dest_addr)+len(src_addr)+1+1+len(payload)])
    fcs = calculate_fcs(frame[8:-2])
    
    # Complete the frame with FCS, end flag, and postamble
    end_flag = b'\x7E'
    postamble = b'\x7E' * 3
    full_frame = frame + fcs + end_flag + postamble
    
    return full_frame

def main():
    frame = construct_ax25_frame()
    print(frame.hex())

if __name__ == "__main__":
    main()
