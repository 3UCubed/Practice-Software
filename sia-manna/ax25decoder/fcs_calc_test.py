import conversions

def reverse_bits(data):

    if isinstance(data, str):
        if all(c in '01' for c in data):
            bin_data = data.zfill(8)
        else:
            bin_data = '{0:08b}'. format(int(data))
    else:
        bin_data = '{0:08b}'.format(data)
    rev_bin_data = bin_data[::-1]
    rev_data = int(rev_bin_data, 2)
    binary_array = [int(bit) for bit in rev_bin_data]


    return binary_array
    
    #rev = reversed(bin_data)
    # print(rev)
    # return rev
    #return ("{:08b}".format(data)[::-1],2)


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
    crc_test_data = crc_test_data.hex()
    print("In hex: ", crc_test_data)
    crc_test_data_bin = conversions.hexadecimal_to_binary(crc_test_data)
    print("crc_test_data_bin: ", crc_test_data_bin)
    print("reversed_bits: ", reverse_bits(crc_test_data_bin))
    #crc_test_data_hex = conversions.binary_to_hexadecimal(crc_test_data_bin)
    crc_test_data_hex = b'\x26\x36\x4E\xF6\xEA\x04\xF6\x36\x36\xA6\x12\x0F\xC0\x87\x62\x12\xAA\x0C\x1A\x1A\x07\x8A\xC2\x0C\x0C\x0C\x0C'
    print("CRC test data in hexadecimal: ", crc_test_data_hex)
    crc_output = crc_calc(crc_test_data_bin, len(crc_test_data_bin))
    print(f"CRC Output:  {crc_output:04X}")
















    """ #crc_test_data = conversions.hexadecimal_to_binary(crc_test_data)
    print("In bin: ", crc_test_data)
    crc_test_data = reverse_bits(crc_test_data)
    print("reversed_bits: ", crc_test_data)
    #lsb_first_data = convert_msb_to_lsb(crc_test_data)
    #print("lsb_first_data: ", lsb_first_data)
    #size_frame = len(lsb_first_data) + 3 
    #crc_output = crc_calc(lsb_first_data, len(lsb_first_data))
    #print(f"CRC Output:  {crc_output:04X}") """
