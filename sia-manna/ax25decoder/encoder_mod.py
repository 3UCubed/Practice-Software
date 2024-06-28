import struct
import conversions

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
    size_frame -= 3  # The last flag and the 2 bytes for FCS are removed.
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
    dest_addr = "303030304351"
    dest_ssid = "E0"
    src_addr = "585830554846"
    src_ssid = "E1"
    control = "03"
    pid = "F0"
    payload = "48656c6c6f20576f726c64"
    payload = payload.ljust(154, '0')
    
    frame = dest_addr + src_addr + dest_ssid + src_ssid + control + pid + payload
    frame_bytes = bytes.fromhex(frame)
    
    crc_value = crc_calc(frame_bytes, len(frame_bytes))
    fcs_bytes = crc_value.to_bytes(2, 'big')
    fcs_hex = fcs_bytes.hex().upper()

    initial_frame = frame + fcs_hex
    initial_frame_bin = ''.join(f'{int(initial_frame[i:i+2], 16):08b}' for i in range(0, len(initial_frame), 2))
    
    bit_stuffed_frame = bit_stuffing(initial_frame_bin)
    return bit_stuffed_frame

def main():
    initial_frame = construct_ax25_frame()
    print("Full frame:", initial_frame)

if __name__ == "__main__":
    main()
