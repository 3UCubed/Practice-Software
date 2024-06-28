import decoder
import encoder

# Tests for both encoder and decoder functions

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


if __name__ == "__main__":

    """Bit-stuffing test 1"""
    bit_stuff_test_data_1 = '11111'
    bit_stuff_output_1 = ax25_bitstuffing_test(bit_stuff_test_data_1)
    print("Bit-stuffed test out put 1: ", bit_stuff_output_1)

    """Bit-stuffing test 2"""
    bit_stuff_test_data_2 = '11110111110111011111'
    bit_stuff_output_2 = ax25_bitstuffing_test(bit_stuff_test_data_2)
    print("Bit-stuffed test out put 2: ", bit_stuff_output_2)

    """Bit destuffing test"""
    input_data = bytearray([0b01111110, 0b11111010])
    result = ax25_bit_destuffing_test(input_data)
    print("Bit destuffing test: ", result)


    """NRZI Encoding Test"""
    test_cases = {
        "00000": "10101",
        "11111": "00000",
        "010101": "100110",
        "101010": "011001",
        "": "",  # empty string
        "1": "0",  # single bit
        "0": "1",  # single bit
    }

    for input_data, expected_output in test_cases.items():
        result = encoder.nrzi_encoding(input_data)
        assert result == expected_output, f"Test failed for input: {input_data}. Expected: {expected_output}, got: {result}"
        print(f"Test passed for input: {input_data}. Output: {result}")

    """Scrambling Test"""
    scrambling_input = "101010101010"
    scrambling_output = encoder.scrambling(scrambling_input)
    print("Original Data:", scrambling_input)
    print("Scrambled Data:", scrambling_output)

    crc_test_data = b'\x7E\x7E\x7E\x7E\x7E\x7E\x7E\x7E\x7E\x30\x30\x30\x30\x43\x51\xE0\x58\x58\x30\x55\x48\x46\xE1\x03\xF0\x48\x65\x6c\x6c\x6f\x20\x57\x6f\x72\x6c\x64\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00'
    crc_output = crc_calc_test(crc_test_data)
    print("CRC Output: ", crc_output)
