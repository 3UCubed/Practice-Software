import struct

def calculate_fcs(frame):
    # Placeholder FCS calculation function (real implementation needed for actual use)
    # For simplicity, let's return a fixed FCS here
    return b'\x12\x34'

def construct_ax25_frame():
    # Components of the frame
    preamble = b'\x7E' * 8
    start_flag = b'\x7E'
    dest_addr = struct.pack('<BBBBBBB', 0xC0, 0x86, 0x88, 0x8A, 0x40, 0x40, 0x60)
    src_addr = struct.pack('<BBBBBBB', 0xE0, 0x88, 0x8A, 0x40, 0x40, 0x40, 0x61)
    control = b'\x03'
    pid = b'\xF0'
    
    # Example payload (77 bytes)
    payload = b'Hello, this is a test message for AX.25 frame with 77 bytes payload.'
    payload += b' ' * (77 - len(payload))  # Pad the payload to ensure it's 77 bytes
    
    # Construct the initial part of the frame (excluding FCS)
    frame = preamble + start_flag + dest_addr + src_addr + control + pid + payload
    
    # Calculate the FCS for the frame (excluding the preamble and flags)
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
