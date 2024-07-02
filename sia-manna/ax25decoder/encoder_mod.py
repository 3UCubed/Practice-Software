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

def crc_calc(frame, size_frame):
    shift_register = 0xFFFF  # Initialization of the Shift Register to 0xFFFF

    for i in range(size_frame):  # Process all bytes in the frame
        byte = frame[i]
        for j in range(8):  # Process each bit from LSB to MSB
            out_bit = shift_register & 0x0001  # Get the LSB of the shift register
            shift_register >>= 1  # Shift the register to the right
            if out_bit != (byte & 0x01):  # Compare LSB of shift register and current bit of byte
                shift_register ^= 0x8408  # XOR with the mirrored polynomial if the bits are different
            byte >>= 1  # Shift the byte to the right to process the next bit

    return shift_register ^ 0xFFFF  # Final XOR

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
    end_flag = b'7E'
    start_flag = b'7E'

    scale = 16


    """DESTINATION ADDRESS"""

    #dest_addr = '303030304351'  # 0000CQ encoded into hex
    # Encode the addresses
    dest_addr_ascii = "0000CQ"

    #Conversion into hex
    dest_addr_hex = ""
    for i in dest_addr_ascii:
        #print("hex bytes for ascii: ", hex(ord(i)))
        dest_addr_hex = dest_addr_hex + format(ord(i), "x")
    print("dest address hex: ", dest_addr_hex)


    #Conversion into binary
    dest_addr_bin = bin(int(dest_addr_hex, scale)).zfill(8)
    print("dest address bin: ", dest_addr_bin)
    

    
    dest_ssid = 'e0'
    dest_ssid_bin = bin(int(dest_ssid, scale)).zfill(8)
    print("dest ssid bin: ", dest_ssid_bin)


    """SOURCE ADDRESS"""

    #src_addr = '585830554846'  # XX0UHF encoded into hex

    # Encode the addresses
    src_addr_ascii = "XX0UHF"

    #Conversion into hex
    src_addr_hex = ""
    for i in src_addr_ascii:
        #print("hex bytes for ascii: ", hex(ord(i)))
        src_addr_hex = src_addr_hex + format(ord(i), "x")
    print("src address hex: ", src_addr_hex)


    src_ssid = 'e1'

    # Control and PID fields
    control = '03'
    pid = 'f0'

    # Example payload (11 bytes, "Hello World!", and then padded to 77 bytes)
    #payload = '48656c6c6f20576f726c64'.ljust(154, '0')
    #payload = '48656c6c6f20576f726c64'
    payload_hex = ""
    payload_ascii = "Hello World"
    for i in payload_ascii:
        payload_hex = payload_hex + format(ord(i), "x")
    print("payload_hex: ", payload_hex)
    

    # Example payload "ilovechickennuggets"
    #payload = '696C6F7665636869636B656E6E7567676574730A'

    # Construct the initial part of the frame (excluding FCS)
    frame_hex = dest_addr_hex + dest_ssid + src_addr_hex + src_ssid + control + pid + payload_hex
    print("Pre-bit-stuffed frame in hex: ", frame_hex)

    frame_bin = conversions.hexadecimal_to_binary(frame_hex)
    print("frame_bin: ", frame_bin)

    frame_bin_bit_stuffed = bit_stuffing(frame_bin)
    print("Frame after bit stuffed: ", frame_bin_bit_stuffed)

    print("bit stuffed frame_bin_bit_stuffed: ", hex(int(frame_bin_bit_stuffed, 2)))



    frame_bytes = bytes.fromhex(frame_hex)
    #frame_bytes = b'0000CQàXX0UHFá\x03\xf0Hello World'
    print("Pre-bitstuffed frame in bytes: ", frame_bytes)

    # Calculate the FCS for the frame (excluding preamble and flags)
    crc_value = crc_calc(frame_bytes, len(frame_bytes))


    # Split FCS into two bytes
    fcs = struct.pack('<H', crc_value)
    print("FCS:" , fcs)

    #Swap FCS bytes
    fcs_swap = fcs[1:2] + fcs[0:1]
    print("Swapped: ", fcs_swap)


    # Combine frame with FCS
    fframe = frame_bytes + fcs_swap
    print("Pre-bit-stuffed: ", fframe.hex())
    #full_frame = frame_bytes

    
    fframe_bin = conversions.hexadecimal_to_binary(fframe.hex())
    fframe_bin_bytes = bytes.fromhex(fframe_bin)
    print("fframe_bin_bytes: ", fframe_bin_bytes)
    #fframe_bin_bytes_lsb = convert_msb_to_lsb(test_frame_bin_bytes)
    #print("fframe_bin_bytes_lsb: ", fframe_bin_bytes_lsb)
    #fframe_bit_stuffed = bit_stuffing(fframe_bin_bytes_lsb)


    test_frame_hex = '303030304351e0585830554846e103f048656c6c6f20576f726c647e80'
    test_frame_bin = conversions.hexadecimal_to_binary('303030304351e0585830554846e103f048656c6c6f20576f726c647e80')
    test_frame_bin_bytes = bytes.fromhex(test_frame_bin)
    print("test_frame_bin_bytes: ", test_frame_bin_bytes)
    test_frame_bin_bytes_lsb = convert_msb_to_lsb(test_frame_bin_bytes)
    print("test_frame_bin_bytes_lsb ", test_frame_bin_bytes_lsb)
    test_frame_bit_stuffed = bit_stuffing(test_frame_bin)
    print("Bit stuffed test frame: ", test_frame_bit_stuffed)

    print ("final frame: ", hex(int(test_frame_bit_stuffed, 2)))

    # Convert full frame to binary
    initial_frame_bin = ''.join(format(byte, '08b') for byte in fframe)

    # Perform bit stuffing
    bit_stuffed_frame = bit_stuffing(initial_frame_bin)

    return bit_stuffed_frame

def main():
    full_frame = construct_ax25_frame()
    full_frame_print = hex(int(full_frame, 2))
    print("Full frame (hex):", full_frame_print)
    format_full_frame = conversions.format_hex_string(full_frame_print)
    print("Formatted full frame: ")
    print(format_full_frame)

if __name__ == "__main__":
    main()
