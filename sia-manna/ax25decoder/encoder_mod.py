"""
Frame:  <   Preamble |   Flag   |  Destination Callsign |  SSID   | Source Callsign |   SSID   | Control |    PID    |    Payload    |    FCS   |   Flag   |   Postamble >
        _____________|__________|_______________________|_________|_________________|__________|_________|___________|_______________|__________|__________|______________
            8 bytes  |  1 byte  |        6 bytes        | 1 byte  |   6 bytes       |  1 byte  | 1 byte  |  1 byte   |    77 bytes   |  2 bytes |  1 byte  |    3 bytes    

                                |-----------------------------------------------Bit stuffed-----------------------------------------------------|
        |-----------------------------------------------------------------------NRZI - Encoded----------------------------------------------------------------------------|
        |-----------------------------------------------------------------------Scrambled---------------------------------------------------------------------------------|

Sequence of encoding operations: 

1. Bit stuff the portion of the frame between the start and end flags
2. Add preframe (preamble and start flag) and postframe(end flag and postamble) and NRZI Encode the whole frame
3. Scramble
4. Encode in hex
"""

import struct
import conversions
from crc import Calculator, Crc16

""" def crc_calc(buffer, size_frame):
    size_frame -= 3  # The last flag and the 2 bytes for FCS are removed.

    # Initialization of the Shift Register to 0xFFFF
    shiftRegister = 0xFFFF

    for i in range(1, size_frame):  # The first flag is not calculated so i=1.
        byte = buffer[i]

        for j in range(8):
            outBit = shiftRegister & 0x0001
            shiftRegister >>= 1  # Shift the register to the right.

            if outBit != (byte & 0x01):
                shiftRegister ^= 0x8408  # Mirrored polynom.
                byte >>= 1
                continue
            byte >>= 1

    return shiftRegister ^ 0xFFFF  # Final XOR. """

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



"""Bit-stuffing adds a '0' after every five consecutive '1's
Function works and passes tests"""
def bit_stuffing(data):
    """
    Perform bit stuffing on the given data.
    """
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

"""
NRZI Encoding is an operation in the AX.25 encoding operation sequence.
The bits are encoded in the presence or absence of a transition at a clock edge.
The '1' bit is encoded as a transition and the '0' bit is encoded as no transition.
Encoding bits in the presence or absence of transitions ensures that the receiver
doesn't have to know what level or state it encodes.
1 = presence of a transition
0 = absence of a transition
Function does not work for certain test cases

def nrzi_encoding(data):
    encoded = []
    current_level = '0'
    
    for bit in data:
        if bit == '0':
            # Change state
            current_level = '1' if current_level == '0' else '0'
        # Append current state
        encoded.append(current_level)
    
    return ''.join(encoded)
"""
"""
Perform scrambling on the given data using a specified polynomial.
    Default polynomial for AX.25 is 1 + x^12 + x^17.
def scrambling(data, polynomial="10000000000010001"):
  
    polynomial = [int(x) for x in polynomial]  # Convert the polynomial string into a list of integers.
    state = [0] * len(polynomial)  # Initialize the state register with zeros.

    scrambled = []  # Initialize an empty list to store the scrambled bits.

    for bit in data:  # Iterate over each bit in the input data.
        feedback = int(bit) ^ state[-1]  # Calculate the feedback XOR by taking the XOR of the current bit and the last bit of the state.
        scrambled.append(str(feedback))  # Append the feedback bit (as a string) to the scrambled list.
        state = [feedback ^ s for s in state[:-1]] + [feedback]  # Update the state register by shifting and XORing with the feedback bit.

    return ''.join(scrambled)  # Return the scrambled data as a concatenated string.
"""


""" def encode_callsign(callsign, ssid):
    # Callsign should be 6 characters, pad with spaces if shorter
    callsign = callsign.ljust(6)
    # Encode each character by shifting left 1 bit
    encoded_callsign = [(ord(char) << 1) for char in callsign]
    # Encode the SSID
    encoded_ssid = (ssid & 0x0F) << 1 | 0x60
    # Combine the encoded callsign and SSID
    return struct.pack('<6B', *encoded_callsign) + struct.pack('B', encoded_ssid) """

""" def placeholder_fcs(frame):
    # Placeholder FCS calculation function (real implementation needed for actual use)
    # For simplicity, let's return a fixed FCS here
    return b'\x12\x34' """


def construct_ax25_frame():

    """COMPONENTS OF THE FRAME"""

    """ #preamble = b'\x7E' * 8
    preamble = "7E7E7E7E7E7E7E7E"
    preamble_bin = format(int(preamble, 16), "064b")
    #start_flag = b'\x7E'
    start_flag = "7E"
    start_flag_bin = format(int(start_flag, 16), "008b")
   
    
    preframe = preamble + start_flag
    print("Preframe: ", preframe)
    preframebin = preamble_bin + start_flag_bin
    print("Preframe to binary: ", preframebin)

    # Complete the frame with FCS, end flag, and postamble
    #end_flag = b'\x7E'
    end_flag= "7E"
    end_flag_bin = format(int(end_flag, 16), "008b")
    #postamble = b'\x7E' * 3
    postamble = "7E7E7E"
    postamble_bin = format(int(postamble, 16), "024b")
    #full_frame = frame + fcs + end_flag + postamble

    postframe = end_flag + postamble
    print("Postframe: ", postframe)    
    postframebin = postamble_bin + end_flag_bin
    print("Postframe to binary: ", postframebin) """

    # Encode the addresses
    #dest_addr = encode_callsign("0000CQ", 0xe0)
    #src_addr = encode_callsign("XX0UHF", 0xe1)
    #dest_addr = struct.pack('<BBBBBBB', 0x00, 0x00, 0x00, 0x00, 0x40, 0x40, 0xE0)
    #src_addr = struct.pack('<BBBBBBB', 0xE0, 0x88, 0x8A, 0x40, 0x40, 0x40, 0xE1)

    """Expected Destination Callsign
    Encoded in hex
    ASCII value = 0000CQ
    """
    dest_addr = "303030304351"
    print("Destination Callsign in hexadecimal: ", dest_addr)

    """Destination Callsign SSID = E0"""
    dest_ssid = "E0"

    """Expected Source Callsign
    Encoded in hex
    ASCII value = XX0UHF"""
    src_addr = "585830554846"

    """Source Callsign SSID = E1"""
    src_ssid = "E1"

    """Control Field
    Encoded in hex
    """
    control = "03"

    """Control Field
    Encoded in bytes"""
    #control = b'\x03'

    """PID Field
    Encoded in hex"""
    pid = "F0"

    """PID Field
    Encoded in bytes"""
    #pid = b'\xF0'
    
    # Example payload (11 bytes, "Hello World!", and then padded to 77 bytes)
    #payload = "6060606086A2E0B0B060AA908C6103F054686520717569636B2062726F776E20666F78206A756D7073206F76657220746865206C617A7920646F67"
    #payload = b'6060606086A2E0B0B060AA908C6103F054686520717569636B2062726F776E20666F78206A756D7073206F76657220746865206C617A7920646F67'
    #payload = b'Hello World!'
    #payload += b' ' * (77 - len(payload))  # Pad the payload to ensure it's 77 bytes
    payload = "48656c6c6f20576f726c64"
    print("Payload in ASCII: " , bytearray.fromhex(payload).decode())
    payload = payload.ljust(77, '0')
    print("payload: ", payload)
    
    # Construct the initial part of the frame (excluding FCS)
    frame = dest_addr + src_addr + control + pid + payload

    print("FRAME: ", frame)

    #Decoding bytes object to produce string
    #frame = frame.decode("utf-8")

    
    # Calculate the FCS for the frame (excluding the preamble and flags)
    #fcs = calculate_fcs(frame[8:8+1+len(dest_addr)+len(src_addr)+1+1+len(payload)])
    """ fcs_preamble = b'\x7E' * 8
    fcs_start_flag = b'\x7E'
    fcs_calc_frame = preamble + start_flag + frame """
    #fcs = b'\x12\x34'
    #fcs = calculate_fcs(fcs_calc_frame[8:-2])
    #fcs = crc_calc(frame, 104)
    #fcs = "80B8"
    crc_test_data = b'\x30\x30\x30\x30\x43\x51\xE0\x58\x58\x30\x55\x48\x46\xE1\x03\xF0\x48\x65\x6c\x6c\x6f\x20\x57\x6f\x72\x6c\x64\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00'
    crc_value = crc_calc(crc_test_data, len(crc_test_data))
    print("CRC value: ", crc_value)
    print("CRC value in bytes: ", crc_value.to_bytes(2, 'big'))
    fcs_byte_one = crc_value & 0xff
    print("FCS byte one: ", fcs_byte_one)
    fcs_byte_two = ((crc_value >> 8) & 0xff)
    print("FCS byte two: ", fcs_byte_two)
    fcs = fcs_byte_one + fcs_byte_two
    print("FCS: ", fcs)


    initialframe = frame + str(fcs)
    #initialframe = initialframe.decode("utf-8")

    #initialframe = "6060606086a260b0b060aa908c6203f048656c6c6f20576f726c642120202020202020202020202020202020202020202020202020202020202020202020202020202020202020202020202020202020202020202020202020202020201234"

    print("Initial frame in hex: "  , initialframe)
    
    #conversion to binary
    initialframebin = format(int(initialframe, 16), "104b")
    #initialframebin = bin(initialframe)
    print ("Raw frame converted to binary ", initialframebin)


    bit_stuffed = bit_stuffing(initialframebin)
    print("Bit stuffed frame: ", bit_stuffed)


    # Complete the entire frame with the end flags and postamble
    #nrzi_encode_ready = preframebin + bit_stuffed + postframebin

    #nrzi_encode = nrzi_encoding(bit_stuffed)

    #print("NRZI encoded: ", nrzi_encode)

    #scramble = scrambling(nrzi_encode)

    #print ("Scrambled: " , scramble)

    #return full_frame
    return bit_stuffed

def main():
    #frame = b'nothing'
    initialframe = construct_ax25_frame()
    #printframe = hex(initialframe)
    print("Full frame: ", initialframe)

if __name__ == "__main__":
    main()