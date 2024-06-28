import struct

def reverse_bits(byte):
    reversed_byte = 0
    for i in range(8):
        reversed_byte <<= 1
        reversed_byte |= (byte & 1)
        byte >>= 1
    return reversed_byte

def convert_msb_to_lsb(buffer):
    return bytes(reverse_bits(byte) for byte in buffer)

def crc_calc(buffer, size_frame):
    shiftRegister = 0xFFFF  # Initialization of the Shift Register to 0xFFFF

    for i in range(size_frame):  # Process all bytes in the frame
        byte = buffer[i]

        for j in range(8):  # Process each bit from LSB to MSB
            outBit = shiftRegister & 0x0001  # Get the LSB of the shift register
            shiftRegister >>= 1  # Shift the register to the right

            if outBit != (byte & 0x01):  # Compare LSB of shift register and current bit of byte
                shiftRegister ^= 0x8408  # XOR with the mirrored polynomial if the bits are different
            
            byte >>= 1  # Shift the byte to the right to process the next bit

    return shiftRegister ^ 0xFFFF  # Final XOR.

def bit_stuffing(data):
    stuffed_data = []
    count = 0
    
    for bit in data:
        stuffed_data.append(bit)
        if bit == '1':
            count += 1
            if count == 5:
                stuffed_data.append('0')
                count = 0
        else:
            count = 0
    
    return ''.join(stuffed_data)

def construct_ax25_frame():
    # Preamble and Flag
    preamble = '7E' * 8
    start_flag = '7E'
    preframe = preamble + start_flag

    # Encode the addresses
    dest_addr = '303030304351'  # 0000CQ
    dest_ssid = 'E0'
    src_addr = '585830554846'  # XX0UHF
    src_ssid = 'E1'

    # Control and PID fields
    control = '03'
    pid = 'F0'

    # Example payload (11 bytes, "Hello World!", and then padded to 77 bytes)
    payload = '48656c6c6f20576f726c64'.ljust(154, '0')

    # Construct the initial part of the frame (excluding FCS)
    frame_hex = dest_addr + dest_ssid + src_addr + src_ssid + control + pid + payload
    frame_bytes = bytes.fromhex(frame_hex)

    # Calculate the FCS for the frame (excluding preamble and flags)
    crc_value = crc_calc(frame_bytes, len(frame_bytes))

    # Split FCS into two bytes
    fcs = struct.pack('<H', crc_value)

    # Combine frame with FCS
    full_frame = frame_bytes + fcs

    # Convert full frame to binary
    initial_frame_bin = ''.join(format(byte, '08b') for byte in full_frame)

    # Perform bit stuffing
    bit_stuffed_frame = bit_stuffing(initial_frame_bin)

    return bit_stuffed_frame

def main():
    full_frame = construct_ax25_frame()
    print("Full frame (hex):", hex(int(full_frame, 2)))

if __name__ == "__main__":
    main()
