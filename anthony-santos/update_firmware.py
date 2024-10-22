import serial
import time
import re
import os
import sys
import argparse

# Constants for bootloader protocol
ENTER_BOOTLOADER_COMMAND = b'\x00\x2A'
SYNC_COMMAND = b'\x7F'
ERASE_COMMAND = b'\x44\xBB'
WRITE_COMMAND = b'\x31\xCE'
GO_COMMAND = b'\x21\xDE'
ACK = b'\x79'
NACK = b'\x1F'

# Replace with the actual address where your firmware should be written
BOOTLOADER_START_ADDRESS = 0x08000000
CHUNK_SIZE = 256  # Typically 256 bytes at a time for STM32 flash writes

# Function to send a command and wait for ACK
def send_command(command):
    ser.write(command)
    response = ser.read(1)
    if response == ACK:
        print("Command acknowledged.")
    elif response == NACK:
        print("Command not acknowledged (NACK received).")
    else:
        print(f"Unexpected response: {response}")
    return response

# Function to configure the serial port
def configure_serial(port, baudrate, parity, stopbits):
    return serial.Serial(
        port=port,
        baudrate=baudrate,
        bytesize=serial.EIGHTBITS,  # 8 data bits
        parity=parity,              # Set parity
        stopbits=stopbits,          # Set stop bits
        timeout=2                   # Timeout for reading from the bootloader
    )

# Function to send the 0x2A command to the MCU to enter bootloader mode
def send_bootloader_command():
    print("Sending 0x2A command to enter bootloader mode...")
    ser.write(ENTER_BOOTLOADER_COMMAND)
    response = ser.read(100)
    print(f"Received response: {response}")
    return response == b'Entering Firmware Update Mode\n'

# Function to sync with the bootloader
def sync_with_bootloader():
    print("Sending sync command...")
    response = send_command(SYNC_COMMAND)
    return response == ACK

# Function to erase flash memory
def erase_flash():
    print("Sending erase command (0x44 and 0xBB)...")
    response = send_command(ERASE_COMMAND)
    if response != ACK:
        send_command(b'\x44')
        response = send_command(ERASE_COMMAND)
        if response != ACK:
            return False
    
    # Now send the actual erase data for Bank 1 (0xFFFE) with checksum
    erase_data = b'\xFF\xFE\x01'  # Bank 1 erase (0xFFFE) with additional 0x01
    checksum = bytes([0x44 ^ 0xFF ^ 0xFE ^ 0x01])  # XOR checksum for 0x44, 0xFF, 0xFE, and 0x01

    print("Sending PageNumber (FF FE 01) and checksum...")
    ser.write(erase_data)  # Send FF FE 01
    ser.write(checksum)  # Send checksum

    # Wait for final ACK
    ser.timeout = 15
    response = ser.read(1)
    if response == ACK:
        print("Erase command for Bank 1 acknowledged.")
        return True
    else:
        print(f"Erase command failed. Received: {response}")
        return False


# Function to send a memory write command and a chunk of data
def send_firmware_chunk(address, data_chunk):
    # print(f"Writing {len(data_chunk)} bytes to address {hex(address)}...")
    # Step 1: Send the Write Memory command (0x31) and its complement (0xCE)
    ser.write(WRITE_COMMAND)

    # Step 2: Wait for ACK after the Write Memory command
    response = ser.read(1)
    if response != ACK:
        print(f"Error: ACK not received after Write command (response: {response})")
        return False

    # Step 3: Send the 4-byte start address and checksum (XOR of address bytes)
    address_bytes = address.to_bytes(4, byteorder='big')  # 4-byte address in big-endian format
    checksum = address_bytes[0] ^ address_bytes[1] ^ address_bytes[2] ^ address_bytes[3]  # XOR of all address bytes
    # print(f"Sending address {address_bytes.hex()} and checksum {hex(checksum)}...")
    ser.write(address_bytes)
    ser.write(bytes([checksum]))

    # Step 4: Wait for ACK after sending the address
    response = ser.read(1)
    if response != ACK:
        print(f"Error: ACK not received after address (response: {response})")
        return False
    
    # Step 5: Send the number of bytes being transmitted
    num_of_bytes = len(data_chunk) - 1  # We send N-1 as per STM32 protocol
    # print(f"Sending number of bytes: {num_of_bytes}...")
    ser.write(bytes([num_of_bytes]))
    
    # Step 6: Calculate and send the final checksum
    checksum = num_of_bytes  # Start with the number of bytes
    for byte in data_chunk:
        checksum ^= byte  # XOR each byte into the checksum
    
    checksum &= 0xFF  # Ensure checksum is a byte (0-255)
    # Step 7: Send the actual data bytes and checksum
    # print(f"Sending data {data_chunk.hex()}...")
    ser.write(data_chunk)
    ser.write(bytes([checksum]))
    
    # Step 8: Wait for final ACK after sending the data and checksum
    response = ser.read(1)
    if response == ACK:
        # print("Write operation successful.")
        return True
    else:
        print(f"Write operation failed at {address} (NACK received). Response: {response}")
        return False


# Function to jump to the new firmware after upload
def jump_to_firmware():
    print("Sending GO command...")
    address_bytes = BOOTLOADER_START_ADDRESS.to_bytes(4, byteorder='big')
    checksum = address_bytes[0] ^ address_bytes[1] ^ address_bytes[2] ^ address_bytes[3]  # XOR of all address bytes

    ser.write(GO_COMMAND)
    ser.write(address_bytes)
    ser.write(bytes([checksum]))

    # Expect ACK after GO command
    response = ser.read(1)
    return response == ACK


# Function to validate the file path
def validate_file_path(file_path):
    # Convert relative paths to absolute paths for better clarity
    abs_path = os.path.abspath(file_path)
    
    if os.path.exists(abs_path):
        print(f"File path is valid: {abs_path}")
        return True
    else:
        print(f"Error: The file path {abs_path} does not exist or cannot be accessed.")
        return False

# Function to validate the COM port format
def validate_com_port(com_port):
    if re.match(r"COM\d+$", com_port):
        print("COM port format is valid.")
        return True
    else:
        print("Error: Invalid COM port format. Expected format: COMx (e.g., COM5).")
        return False

# Main function to handle the whole process
def upload_firmware(firmware_path, com_port):
    global ser

    # Step 1: Open UART at 460800 baud with no parity and send 0x2A command
    ser = configure_serial(port=com_port, baudrate=460800, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE)
    print("Configured UART at 460800 baud, no parity.")
    if not send_bootloader_command():
        print("Returning to firmware, try again")
        return

    ser.close()  # Close the serial port before reconfiguring

    # Step 3: Reconfigure UART for bootloader (115200 baud, even parity)
    ser = configure_serial(port=com_port, baudrate=115200, parity=serial.PARITY_EVEN, stopbits=serial.STOPBITS_ONE)
    print("Reconfigured UART at 115200 baud, even parity.")

    # Step 4: Sync with bootloader
    if not sync_with_bootloader():
        print("Failed to sync with the bootloader.")
        print("Jumping back to firmware, try again")
        jump_to_firmware()
        return

    # Step 5: Erase flash memory
    if not erase_flash():
        print("Failed to erase flash memory.")
        return
    
    
    # Command / bytes are offset after previous command, reset here using sync
    response = send_command(SYNC_COMMAND)
    
    
    # Step 6: Read the firmware file
    print("Writing new firmware...")
    with open(firmware_path, 'rb') as f:
        firmware = f.read()
    
    # Step 7: Send firmware in chunks
    address = BOOTLOADER_START_ADDRESS
    for i in range(0, len(firmware), CHUNK_SIZE):
        chunk = firmware[i:i+CHUNK_SIZE]

        # Pad chunk if it's not full
        if len(chunk) < CHUNK_SIZE:
            chunk += b'\xFF' * (CHUNK_SIZE - len(chunk))

        # Send the chunk
        if not send_firmware_chunk(address, chunk):
            print(f"Failed to send chunk at address {hex(address)}.")
            return

        address += CHUNK_SIZE

    print("Firmware upload complete!")

    # Step 8: Jump to the new firmware
    if not jump_to_firmware():
        print("Failed to jump to the new firmware.")
    else:
        print("Successfully jumped to the new firmware!")


# Function to parse and validate command-line arguments
def parse_arguments():
    parser = argparse.ArgumentParser(description="Upload firmware to STM32 over UART.")
    parser.add_argument("firmware_path", help="Path to the firmware .bin file")
    parser.add_argument("com_port", help="COM port (e.g., COM5)")

    args = parser.parse_args()

    # Validate the file path
    if not validate_file_path(args.firmware_path):
        sys.exit("Exiting: Invalid file path.")

    # Validate the COM port
    if not validate_com_port(args.com_port):
        sys.exit("Exiting: Invalid COM port.")

    return args

if __name__ == "__main__":
    # Parse command-line arguments
    args = parse_arguments()

    # Call the upload function with the provided firmware file path and COM port
    upload_firmware(args.firmware_path, args.com_port)

    # Close the serial port if open
    if 'ser' in globals() and ser.is_open:
        ser.close()