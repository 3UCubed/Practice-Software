import struct
import conversions

def format_hex_string(hex_string):
    # Remove the "0x" prefix and split the string into pairs of characters
    bytes_list = [hex_string[i:i+2] for i in range(2, len(hex_string), 2)]
    
    # Format each byte as "0xXX"
    formatted_bytes = [f"0x{byte}" for byte in bytes_list]
    
    # Create formatted lines
    lines = []
    line = []
    for i, byte in enumerate(formatted_bytes):
        line.append(byte)
        if (i + 1) % 8 == 0:  # After every 8 bytes, add the line to lines
            lines.append(' '.join(line[:8]) + '  ' + ' '.join(line[8:]))
            line = []
    
    if line:  # Add any remaining bytes
        lines.append(' '.join(line[:8]) + '  ' + ' '.join(line[8:]))
    
    return '\n'.join(lines)

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


def construct_ax25_frame(payload_of_choice, destination_address, source_address):
    # Preamble and Flag
    end_flag = b'7E'
    start_flag = b'7E'

    scale = 16


    """DESTINATION ADDRESS"""

    #dest_addr = '303030304351'  # 0000CQ encoded into hex
    # Encode the addresses
    #dest_addr_ascii = "3U3UHF"
    dest_addr_ascii = destination_address

    #Conversion into hex
    dest_addr_hex = ""
    for i in dest_addr_ascii:
        #print("hex bytes for ascii: ", hex(ord(i)))
        dest_addr_hex = dest_addr_hex + format(ord(i), "x")
    print("Destination Address in hexadecimal: ", dest_addr_hex)


    #Conversion into binary
    dest_addr_bin = bin(int(dest_addr_hex, scale)).zfill(8)
    print("Destination Address in Binary: ", dest_addr_bin)
    

    
    dest_ssid = 'e0'
    dest_ssid_bin = bin(int(dest_ssid, scale)).zfill(8)
    print("Destination SSID in binary: ", dest_ssid_bin)


    """SOURCE ADDRESS"""

    #src_addr = '585830554846'  # XX0UHF encoded into hex

    # Encode the addresses
    #src_addr_ascii = "3U3UNH"
    src_addr_ascii = source_address

    #Conversion into hex
    src_addr_hex = ""
    for i in src_addr_ascii:
        #print("hex bytes for ascii: ", hex(ord(i)))
        src_addr_hex = src_addr_hex + format(ord(i), "x")
    print("Source Address in hexadecimal: ", src_addr_hex)

    #Conversion into binary
    src_addr_bin = bin(int(src_addr_hex, scale)).zfill(8)
    print("Source Address in Binary: ", src_addr_bin)


    src_ssid = 'e1'

    src_ssid_bin = bin(int(src_ssid, scale)).zfill(8)
    print("Source SSID in binary: ", src_ssid_bin)

    # Control and PID fields
    control = '03'
    pid = 'f0'

    # Example payload (11 bytes, "Hello World!", and then padded to 77 bytes)
    #payload = '48656c6c6f20576f726c64'.ljust(154, '0')
    #payload = '48656c6c6f20576f726c64'
    payload_hex = ""
    #payload_ascii = "Hello World"
    payload_ascii = payload_of_choice
    for i in payload_ascii:
        payload_hex = payload_hex + format(ord(i), "x")
    print("Payload in hexadecimal: ", payload_hex)
    

    # Example payload "ilovechickennuggets"
    #payload = '696C6F7665636869636B656E6E7567676574730A'

    # Construct the initial part of the frame (excluding FCS)
    frame_hex = dest_addr_hex + dest_ssid + src_addr_hex + src_ssid + control + pid + payload_hex
    print("Pre-bit-stuffed frame without FCS from dest address-payload in hexadecimal: ", frame_hex)









    frame_bin = conversions.hexadecimal_to_binary(frame_hex)
    print("Pre-bit-stuffed frame without FCS from dest address-payload in binary: ", frame_bin)

    frame_bin_bit_stuffed = bit_stuffing(frame_bin)
    print("Frame without FCS after bit stuffing (works with NotBlackMagic): ", frame_bin_bit_stuffed)

    final_frame_without_fcs_in_hex = hex(int(frame_bin_bit_stuffed, 2))

    print("Bit-stuffed frame without FCS in hexadecimal: ", final_frame_without_fcs_in_hex)






    """FCS CALCULATION"""

    frame_bytes = bytes.fromhex(frame_hex)
    crc_value = crc_calc(frame_bytes, len(frame_bytes))
    # Split FCS into two bytes
    fcs = struct.pack('<H', crc_value)
    print("FCS:" , fcs)
    #Swap FCS bytes
    fcs_swap = fcs[1:2] + fcs[0:1]
    print("Swapped: ", fcs_swap)

    frame_with_fcs = frame_hex + fcs_swap.hex()
    print("Frame with FCS without bit-stuffing: ", frame_with_fcs)
    frame_with_fcs_bin = conversions.hexadecimal_to_binary(frame_with_fcs)
    frame_with_fcs_bit_stuffed = bit_stuffing(frame_with_fcs_bin)
    final_frame_with_fcs_bit_stuffed = hex(int(frame_with_fcs_bit_stuffed, 2))
    print("Bit-stuffed frame with FCS in hexadecimal: ", final_frame_with_fcs_bit_stuffed)




    """

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

    
    #fframe_bin = conversions.hexadecimal_to_binary(fframe.hex())
    #fframe_bin_bytes = bytes.fromhex(fframe_bin)
    #print("fframe_bin_bytes: ", fframe_bin_bytes)
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
    """

    return final_frame_without_fcs_in_hex

def main():
    payload_of_choice = input("What is your value for payload? ")
    dstaddr_of_choice = input("What is your destination address? ")
    srcaddr_of_choice = input("What is the source address? ")
    full_frame = construct_ax25_frame(payload_of_choice, dstaddr_of_choice, srcaddr_of_choice)
    #full_frame_print = hex(int(full_frame, 2))
    #print("Full frame (hex):", full_frame_print)
    print("Full frame without FCS that works with not black magic: ", full_frame)
    format_full_frame = format_hex_string(full_frame)
    print("Formatted full frame to put in black magic decoder: ")
    print(format_full_frame)

if __name__ == "__main__":
    main()
