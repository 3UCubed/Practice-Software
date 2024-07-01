def reverse_bits(byte):
    reversed_byte = 0
    for i in range(8):
        reversed_byte <<= 1
        reversed_byte |= (byte & 1)
        byte >>= 1
    return reversed_byte

def convert_msb_to_lsb(buffer):
    return bytes(reverse_bits(byte) for byte in buffer)

def crc_calc(frame, size_frame):
    #size_frame -= 3  # The last flag and the 2 bytes for FCS are removed.
    shiftRegister = 0xFFFF  # Initialization of the Shift Register to 0xFFFF

    for i in range(size_frame):  # Process all bytes in the frame
        byte = frame[i]

        for j in range(8):  # Process each bit from LSB to MSB
            outBit = shiftRegister & 0x0001  # Get the LSB of the shift register
            shiftRegister >>= 1  # Shift the register to the right

            if outBit != (byte & 0x01):  # Compare LSB of shift register and current bit of byte
                shiftRegister ^= 0x8408  # XOR with the mirrored polynomial if the bits are different
            
            byte >>= 1  # Shift the byte to the right to process the next bit

    return shiftRegister ^ 0xFFFF  # Final XOR.

if __name__ == "__main__":
    crc_test_data = '303030304351E0585830554846E103F048656c6c6f20576f726c64000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000'
    crc_test_data_bytes = bytes.fromhex(crc_test_data)
    #crc_test_data = b'\x6C\x6E\x70\x72\x60\x40\xE0\x62\x64\x66\x68\x6A\x40\x61\x03\xF0\x48\x65\x6C\x6C\x6F\x20\x57\x6F\x72\x6C\x64\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00'
    
    # Exclude the initial and last 0x7E flags
    #data_to_crc = crc_test_data[1:-1]
    data_to_crc = crc_test_data_bytes

    # Convert data to LSB first
    lsb_first_data = convert_msb_to_lsb(data_to_crc)
    
    # Calculate the size of the frame for FCS calculation
    size_frame = len(lsb_first_data)  # +3 to include the size reduction in crc_calc

    # Calculate CRC
    fcs_value = crc_calc(lsb_first_data, size_frame)
    print(f"CRC Output: {fcs_value:04X}")
