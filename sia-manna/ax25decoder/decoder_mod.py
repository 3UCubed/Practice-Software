import struct

def hexToASCII(hexx):
 
    # initialize the ASCII code string as empty.
    ascii = ""
 
    for i in range(0, len(hexx), 2):
 
        # extract two characters from hex string
        part = hexx[i : i + 2]
 
        # change it into base 16 and
        # typecast as the character 
        ch = chr(int(part, 16))
 
        # add this char to final ASCII string
        ascii += ch
     
    return ascii

def binary_to_hexadecimal(binary_str):
    # Ensure the binary string length is a multiple of 4
    while len(binary_str) % 4 != 0:
        binary_str = '0' + binary_str
    
    # Convert binary string to hexadecimal
    hex_str = hex(int(binary_str, 2))[2:].upper()
    
    return hex_str

def bit_destuffing(data):
    destuffed_data = []
    count = 0
    i = 0

    while i < len(data):
        bit = data[i]
        destuffed_data.append(bit)
        
        if bit == '1':
            count += 1
            if count == 5:
                # Skip the next bit which should be a '0' added during stuffing
                i += 1
                count = 0
        else:
            count = 0
        
        i += 1

    return ''.join(destuffed_data)

def destruct_ax25_frame(encoded_frame):
    
    bit_destuffed_frame = bit_destuffing(encoded_frame)
    print("Bit-destuffed frame: ", bit_destuffed_frame)
    bit_destuffed_frame_hex = binary_to_hexadecimal(bit_destuffed_frame)
    print("Bit_decoded_frame_hex: ", bit_destuffed_frame_hex)

    position = 0
    dest_ssid = ""
    src_ssid = ""
    control = ""
    pid = ""
    dest_addr_array = []
    src_addr_array = []
    payload_array = []

    for i in range(0, len(bit_destuffed_frame_hex), 2):
        position += 1
        hex_pair = bit_destuffed_frame_hex[i:i+2]
        hex_pair_in_ascii = hexToASCII(hex_pair)
        if position <= 6:
            dest_addr_array += hex_pair_in_ascii
        elif position == 7:
            dest_ssid = hex_pair
        elif position >=8 and position <= 13:
            src_addr_array += hex_pair_in_ascii
        elif position == 14:
            src_ssid = hex_pair
        elif position == 15:
            control = hex_pair
        elif position == 16:
            pid = hex_pair
        else:
        #if position >= 17:
            payload_array += hex_pair_in_ascii
        
    dest_addr = ""
    for i in dest_addr_array:
        dest_addr += i
    print("Destination Address: ", dest_addr) 

    print("Destination SSID: ", dest_ssid)

    src_addr = ""
    for i in src_addr_array:
        src_addr += i
    print("Source Address: ", src_addr)       

    print("Source SSID: ", src_ssid)

    print("Control: ", control)
    print("PID: ", pid)

    payload = ""
    for i in payload_array:
        payload += i
    print("Payload: ", payload)

    return bit_destuffed_frame_hex
    
def main():
    #frame_of_choice = input("What is your value for the encoded frame? ")
    encoded_frame = "0x60 0x60 0x60 0x60 0x86 0xa3 0xc0 0xb0 0xb0 0x60 0xaa 0x90 0x8d 0xc2 0x07 0xd0 0x68 0x65 0x6c 0x6c 0x6f 0x20 0x77 0x6f 0x72 0x6c 0x64"
    hex_encoded_frame = encoded_frame.replace("0x", "").replace(" ", "")
    print("hex_encoded_frame: ", hex_encoded_frame)
    bytes_encoded_frame = ''.join(f'{int(hex_encoded_frame[i:i+2], 16):08b}' for i in range(0, len(hex_encoded_frame), 2))
    bit_destuffed_frame = destruct_ax25_frame(bytes_encoded_frame)

if __name__ == "__main__":
    main()
