import decoder
import encoder

# Test for preamble alone
# Since preamble is NRZI encoded and then scrambled,this is a test
# to see if they match

def crc_calc_test(data):
    length = len(data)
    test_crc_value = encoder.crc_calc(data, length)
    """For more secure testing purposes- test against this: https://crccalc.com/"""
    return test_crc_value

def ax25_bit_destuffing_test(data):
    bit_destuffed_data = decoder.bit_destuffing(data)
    return bit_destuffed_data

def ax25_bitstuffing_test(data):
    bit_stuffed_data = encoder.bit_stuffing(data)
    return bit_stuffed_data

def ax25_nrzi_decoding_test(data):
    nrzi_decoded_data = decoder.nrzi_decoding(data)
    return nrzi_decoded_data

def ax25_nrzi_encoding_test(data):
    nrzi_encoded_data = encoder.nrzi_encoding(data)
    return nrzi_encoded_data

def ax25_descrambling_test(data):
    descrambled_data = decoder.descrambling(data)
    return descrambled_data


def ax25_nrziencoding(data):
    """
    Perform NRZI encoding on the given data.
    """
    encoded = []
    current_level = '0'
    
    for bit in data:
        if bit == '0':
            # Change state
            current_level = '1' if current_level == '0' else '0'
        # Append current state
        encoded.append(current_level)
    
    return ''.join(encoded)

def ax25_scrambler(data, polynomial="10000000000010001"):
    """
    Perform scrambling on the given data using a specified polynomial.
    Default polynomial for AX.25 is 1 + x^12 + x^17.
    """
    polynomial = [int(x) for x in polynomial]
    state = [0] * len(polynomial)
    
    scrambled = []
    
    for bit in data:
        feedback = int(bit) ^ state[-1]
        scrambled.append(str(feedback))
        state = [feedback ^ s for s in state[:-1]] + [feedback]
    
    return ''.join(scrambled)

def encode_test(test_str):
    """
    Preamble should be equal to 8 * 7E
    """

    test_str = ax25_nrziencoding(test_str)
    print("Test string after nrzi encoding: " + test_str)
    test_str = ax25_scrambler(test_str)
    print("test string after scrambling: " + test_str)

    return test_str

if __name__ == "__main__":
    test_string = b'\x7e' * 8
    print("Correct preamble: 7E7E7E7E7E7E7E7E")
    outstr = encode_test(test_string)
    print ("\n Encoded preamble: " + outstr)
