# !/sia_manna/ax25decoder
"""! @brief encoder_mod.py created to emulate the AX.25 encoding process"""

##
# @mainpage AX.25 Encoder
#
# @section description_main Description
# A Python Script that performs AX.25 encoding by taking user input for destination
# address, source address and payload and outputs an AX.25, performs HDLC encoding and 
# outputs an HDLC encoded frame as well. As of the 3 AX.25 encoding process steps, this 
# encoder constructs the frame in hex, bit-stuffs it, performs FCS calculation, and performs
# HDLC encoding
#
# @section notes_main Notes
# - AX.25 encoding process consists of three processes:
# - Bit-stuffing
# - NRZI encoding
# - Scrambling
# - 
# Copyright (c) 2020 Woolsey Workshop.  All rights reserved.

# Imports
import struct
import conversions
import math

# To do
# 1. Create doxygen about the AX.25 decoder
# 2. Email Erika
# 3. Write descriptions for proper documentation
# 

# Formats frames for easy copy and paste into NotBlackMagic decoder
# Ensure that all strings are formatted regardless of whether they are string or array
def format_hex_string(hex_val):

    """! Formats frames for easy copy and paste into NotBlackMagic decoder
    @param hex_val hexadecimal value of ax25 frame or hdlc frame
    @return formatted hexadecimal string
    """


    final_answer = ""

    if isinstance(hex_val, str):
        # Remove the "0x" prefix and split the string into pairs of characters
        bytes_list = [hex_val[i:i+2] for i in range(2, len(hex_val), 2)]
        
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
        
        final_answer = '\n'.join(lines)
    else:
        line_length = 8
        formatted_hex = [f"0x{int(x, 16):02X}" for x in hex_val]
    
        # Group the formatted hex strings into lines of specified length
        lines = []
        for i in range(0, len(formatted_hex), line_length):
            line = ' '.join(formatted_hex[i:i + line_length])
            lines.append(line)
        
        # Join the lines into a single string with new lines
        final_answer = '\n'.join(lines)

    
    return final_answer

# correct fcs calculation
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

# correct bit stuffing
# needs to be debugged to figure out why it converts the pid and control
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


# hdlc encoding should take in fcs included bit stuffed frame and
# that inverts the order from msb to lsb and then convert it to hexadecimal
def hdlc_frame(init_frame):

    """! Takes bit-stuffed AX.25 frame and performs bit reversal,
    adds FCS and HDLC encodes it
    @param hex_val hexadecimal value of ax25 frame
    @return hexadecimal HDLC frame
    """


    print ("AX.25 frame: ", init_frame)
    hex_encoded_frame = init_frame.replace("0x", "").replace(" ", "")

    #Convert to binary
    init_frame_bin = conversions.hexadecimal_to_binary(hex_encoded_frame)
    print("AX25 frame to binary: ", init_frame_bin)


    # Step 1: Separate into an array of 4-bit segments
    segments = [init_frame_bin[i:i+8] for i in range(0, len(init_frame_bin), 8)]
    print ("MSB first segment: ", segments)
    
    # Step 2: Flip the bits in each segment
    flipped_segments = []
    for segment in segments:
        flipped_segment = segment[::-1]
        flipped_segments.append(flipped_segment)
    print("Bit inversion that converts the frame from MSB to LSB: ", flipped_segments)
    
    # Step 3: Convert each flipped segment to hexadecimal
    hex_values = [hex(int(segment, 2))[2:].upper() for segment in flipped_segments]
    print("Hexadecimal value array of the HDLC encoded frame: ", hex_values)
    
    return hex_values


#write clearer print statements
def ax25_frame(payload_of_choice, destination_address, source_address):

    """! Construct the AX25 frame from scratch with destination
    address, destination SSID, source address, source SSID, 
    control, PID, payload
    @param hex_val hexadecimal value of ax25 frame
    @return hexadecimal HDLC frame
    """

    scale = 16

    """DESTINATION ADDRESS"""

    dest_addr_ascii = destination_address

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

    src_addr_ascii = source_address

    src_addr_hex = ""
    for i in src_addr_ascii:
        #print("hex bytes for ascii: ", hex(ord(i)))
        src_addr_hex = src_addr_hex + format(ord(i), "x")
    print("Source Address in hexadecimal: ", src_addr_hex)

    src_addr_bin = bin(int(src_addr_hex, scale)).zfill(8)
    print("Source Address in Binary: ", src_addr_bin)

    src_ssid = 'e1'

    src_ssid_bin = bin(int(src_ssid, scale)).zfill(8)
    print("Source SSID in binary: ", src_ssid_bin)

    control = '03'
    pid = 'f0'

    #payload = '48656c6c6f20576f726c64'.ljust(154, '0')
    payload_hex = ""
    payload_ascii = payload_of_choice
    for i in payload_ascii:
        payload_hex = payload_hex + format(ord(i), "x")
    print("Payload in hexadecimal: ", payload_hex)
    

    # Construct the initial part of the frame (excluding FCS)
    frame_hex = dest_addr_hex + dest_ssid + src_addr_hex + src_ssid + control + pid + payload_hex
    #print("Pre-bit-stuffed frame without FCS from dest address-payload in hexadecimal: ", frame_hex)









    frame_bin = conversions.hexadecimal_to_binary(frame_hex)
    print("FRAME (NO bit-stuffing, NO FCS, Destination Address-Payload, Binary): ", frame_bin)

    frame_bin_bit_stuffed = bit_stuffing(frame_bin)
    print("FRAME (Bit-stuffed, NO FCS, Destination Address- Payload, Binary): ", frame_bin_bit_stuffed)
    print("THIS FRAME WORKS WITH NOTBLACKMAGIC DECODER")

    final_frame_without_fcs_in_hex = hex(int(frame_bin_bit_stuffed, 2))
    print("FRAME (Bit-stuffed, NO FCS, Destination Address- Payload, Hexadecimal): ", final_frame_without_fcs_in_hex)






    """FCS CALCULATION"""

    # Converts the hex string to hex bytes for the FCS function
    frame_bytes = bytes.fromhex(frame_hex)

    # FCS calculation
    crc_value = crc_calc(frame_bytes, len(frame_bytes))

    # Split FCS into two bytes
    fcs = struct.pack('<H', crc_value)
    print("FCS:" , fcs)

    #Swap FCS bytes
    fcs_swap = fcs[1:2] + fcs[0:1]
    print("Swapped FCS bytes: ", fcs_swap)

    # Swapped FCS bytes in hexadecimal
    print("Swapped FCS in hexadecimal: ", fcs_swap.hex())

    # Pre-bit-stuffed frame with FCS
    frame_with_fcs = frame_hex + fcs_swap.hex()
    print("FRAME (NO bit-stuffing, FCS, Destination Address-FCS, Hexadecimal): ", frame_with_fcs)

    # Conversion to binary for bit-stuffing
    frame_with_fcs_bin = conversions.hexadecimal_to_binary(frame_with_fcs)

    # Bit-stuffing
    frame_with_fcs_bit_stuffed = bit_stuffing(frame_with_fcs_bin)
    final_frame_with_fcs_bit_stuffed = hex(int(frame_with_fcs_bit_stuffed, 2))
    print("FRAME (Bit-stuffed, FCS, Destination Address-FCS, Hexadecimal): ", final_frame_with_fcs_bit_stuffed)

    #absolute_last_frame = final_frame_without_fcs_in_hex + fcs_swap.hex()


    return final_frame_without_fcs_in_hex
    #return absolute_last_frame


def hdlc_encoding(init_frame):

    """! Construct the AX25 frame from scratch with destination
    address, destination SSID, source address, source SSID, 
    control, PID, payload
    @param hex_val hexadecimal value of ax25 frame
    @return hexadecimal HDLC frame
    """


    print ("AX.25 Frame: ", init_frame)
    hex_encoded_frame = init_frame.replace("0x", "").replace(" ", "")

    #Convert to binary
    init_frame_bin = conversions.hexadecimal_to_binary(hex_encoded_frame)
    print("AX.25 Frame to binary: ", init_frame_bin)

    
    # Step 1: Separate into an array of 4-bit segments
    segments = [init_frame_bin[i:i+8] for i in range(0, len(init_frame_bin), 8)]
    print ("AX.25 Frame separated into 8 bit-segments: ", segments)
    
    # Step 2: Flip the bits in each segment
    flipped_segments = []
    for segment in segments:
        flipped_segment = segment[::-1]
        #flipped_segment = hex(int(flipped_segment, 2))
        #flipped_segment = ''.join('1' if bit == '0' else '0' for bit in segment)
        #print("flipped segment: ", flipped_segment)
        flipped_segments.append(flipped_segment)
    print("HDLC frame separated into reversed AX.25 frame segments: ", flipped_segments)
    
    # Step 3: Convert each flipped segment to hexadecimal
    hex_values = [hex(int(segment, 2))[2:].upper() for segment in flipped_segments]
    print("Hexadecimal values of HDLC frame segments: ", hex_values)

    # Insert flag delimiter that acts as end flag of current frame and start flag of next
    hex_values.insert(0, '7E')

    # Formatting for HDLC frame to test with NotBlackMagic decoder
    final_hdlc = format_hex_string(hex_values)
    
    #return hex_values
    return final_hdlc


def main():
    payload_of_choice = input("What is your value for payload? ")
    dstaddr_of_choice = input("What is your destination address? ")
    srcaddr_of_choice = input("What is the source address? ")
    full_frame = ax25_frame(payload_of_choice, dstaddr_of_choice, srcaddr_of_choice)
    print("Full frame without FCS that works with not black magic: ", full_frame)
    reversed_frame = hdlc_frame(full_frame)
    print("HDLC Encoded: ") 
    print(reversed_frame)
    format_full_frame = format_hex_string(full_frame)

    print("Formatted full frame to put in black magic decoder: ")
    print(format_full_frame)

if __name__ == "__main__":
    main()
