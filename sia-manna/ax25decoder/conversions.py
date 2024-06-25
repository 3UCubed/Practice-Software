"""
Conversions lib from various datatypes
"""

""" def hextobin(data):
    n = int(data, 16) 
    bStr = '' 
    while n > 0: 
        bStr = str(n % 2) + bStr 
        n = n >> 1   
    res = bStr 

    return res """

def ascii_to_hexadecimal(data):
    return hex(ord(data))

def hexadecimal_to_binary(data):
    hex_num = '1f'
    hex_dict = {'0': '0000', '1': '0001', '2': '0010', '3': '0011', '4': '0100', '5': '0101', '6': '0110', '7': '0111', '8': '1000', '9': '1001', 'a': '1010', 'b': '1011', 'c': '1100', 'd': '1101', 'e': '1110', 'f': '1111'}
    binary = ''
    for digit in hex_num:
        binary += hex_dict[digit]


def binary_to_hexadecimal(n):
   
    binary_string = ''.join(format(byte, '08b') for byte in n)

    # Convert binary string to hexadecimal string
    hex_num = hex(int(binary_string, 2))

    # Return the hexadecimal string, removing the '0x' prefix
    return hex_num[2:]

def binary_to_hexadecimal_2(data):
    bnum = int(data)
    temp = 0
    mul = 1
     
    # counter to check group of 4
    count = 1
     
    # char array to store hexadecimal number
    hexaDeciNum = ['0'] * 100
     
    # counter for hexadecimal number array
    i = 0
    while bnum != 0:
        rem = bnum % 10
        temp = temp + (rem*mul)
         
        # check if group of 4 completed
        if count % 4 == 0:
           
            # check if temp < 10
            if temp < 10:
                hexaDeciNum[i] = chr(temp+48)
            else:
                hexaDeciNum[i] = chr(temp+55)
            mul = 1
            temp = 0
            count = 1
            i = i+1
             
        # group of 4 is not completed
        else:
            mul = mul*2
            count = count+1
        bnum = int(bnum/10)
         
    # check if at end the group of 4 is not
    # completed
    if count != 1:
        hexaDeciNum[i] = chr(temp+48)
         
    # check at end the group of 4 is completed
    if count == 1:
        i = i-1
         
    # printing hexadecimal number
    # array in reverse order
    print("\n Hexadecimal equivalent of {}:  ".format(data), end="")
    while i >= 0:
        print(end=hexaDeciNum[i])
        i = i-1

    def conversion(var_type, data):
        print(var_type)
        print("In hexadecimal: ", data)
        print("In ASCII: ", data)
        print("In binary: ", data)
        print("This is the final variable: \n")
        return data

