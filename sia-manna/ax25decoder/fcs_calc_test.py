def reverse_bits(byte):
    reversed_byte = 0
    for i in range(8):
        reversed_byte <<= 1
        print("reversed_byte <<= 1: ", reversed_byte)
        reversed_byte |= (byte & 1)
        print("reversed_byte |= (byte & 1): ", reversed_byte)
        byte >>= 1
        print("byte >>= 1: ", byte)
    return reversed_byte

def convert_msb_to_lsb(buffer):
    return bytes(reverse_bits(byte) for byte in buffer)

def crc_calc(buffer, size_frame):

    size_frame -= 3  # The last flag and the 2 bytes for FCS are removed.

    # Initialization of the Shift Register to 0xFFFF
    shiftRegister = 0xFFFF

    #Loop through each byte
    for i in range(1, size_frame):          # The first flag is not calculated so i=1.
        byte = buffer[i]
        
        #Loop through each bit
        for j in range(8):
            outBit = shiftRegister & 0x0001
            shiftRegister >>= 1             # Shift the register to the right and processes LSB first

            
            if outBit != (byte & 0x01):     # Compares LSB to shift register and byte
                shiftRegister ^= 0x8408     # Mirrored polynom.
            byte >>= 1                      # Shift byte to right
    
    return shiftRegister ^ 0xFFFF           # Final XOR.

if __name__ == "__main__":
    crc_test_data = b'\x30\x30\x30\x30\x43\x51\xE0\x58\x58\x30\x55\x48\x46\xE1\x03\xF0\x48\x65\x6c\x6c\x6f\x20\x57\x6f\x72\x6c\x64\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00'
    lsb_first_data = convert_msb_to_lsb(crc_test_data)
    print("lsb_first_data: ", lsb_first_data)
    #size_frame = len(lsb_first_data) + 3 
    #crc_output = crc_calc(lsb_first_data, len(lsb_first_data))
    #print(f"CRC Output:  {crc_output:04X}")
