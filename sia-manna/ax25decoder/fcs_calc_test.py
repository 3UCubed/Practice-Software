def crc_calc(buffer, size_frame):

    print("buffer: ", buffer)
    print("size_frame: ", size_frame)
    size_frame -= 3  # The last flag and the 2 bytes for FCS are removed.
    print("size_frame -=3: ", size_frame)

    # Initialization of the Shift Register to 0xFFFF
    shiftRegister = 0xFFFF
    print("shiftRegister: ", shiftRegister)

    for i in range(1, size_frame):  # The first flag is not calculated so i=1.
        byte = buffer[i]
        print("byte: ", byte)

        for j in range(8):
            print("j: ", j)
            outBit = shiftRegister & 0x0001
            print("outBit = shiftRegister & 0x0001: ", outBit)
            shiftRegister >>= 1  # Shift the register to the right.

            if outBit != (byte & 0x01):
                shiftRegister ^= 0x8408  # Mirrored polynom.
            byte >>= 1
    
    return shiftRegister ^ 0xFFFF  # Final XOR.

if __name__ == "__main__":
    crc_test_data = b'\x7E\x30\x30\x30\x30\x43\x51\xE0\x58\x58\x30\x55\x48\x46\xE1\x03\xF0\x48\x65\x6c\x6c\x6f\x20\x57\x6f\x72\x6c\x64\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00'
    print(type(crc_test_data))
    #print(type(0x002B))
    crc_output = crc_calc(crc_test_data, len(crc_test_data))
    print("CRC Output: ", crc_output)
    print(type(crc_output))