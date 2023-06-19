// Problem: CubeSat Telemetry Packet Generator
// You are tasked with developing a telemetry packet generator for a CubeSat mission.
// The CubeSat is equipped with various sensors to collect data, and the generated telemetry
// packets will be transmitted back to the ground station for analysis. Each packet will contain
// a header, data payload, and a checksum.
// Your goal is to implement a function called generate_telemetry_packet that takes in the sensor
// data and returns a formatted telemetry packet.


//Instructions:
//The telemetry packet structure consists of:
//Header (2 bytes): Start of packet marker
//Data Payload (variable length): Sensor data
//Checksum (1 byte): XOR of all bytes in the packet (excluding the start marker)
//The header is a fixed value of 0xAA55.
//The data payload can contain various types of sensor data, such as temperature, pressure, humidity, or any other relevant measurements. You can represent the data in any format you prefer (e.g., integer, floating-point).
//The checksum is calculated by performing an XOR operation on all bytes in the packet (excluding the start marker). The result should be stored in a single byte.
//Ensure that your function accepts the sensor data as input and generates a properly formatted telemetry packet as output.
//Test your implementation by generating telemetry packets with different sensor data and verifying the output.

#include <iostream>
#include <vector>

std::vector<uint8_t> generate_telemetry_packet(const std::vector<float>& sensor_data) { //function to generate telemetry packet based on sensor data
    std::vector<uint8_t> packet; //creating an empty vector to hold the telemetry packet
    packet.push_back(0xAA); //start of the marker byte 1
    packet.push_back(0x55); //start of the marker byte 2

    for (const auto& data : sensor_data) { //iterate over each sensor data value in the input vector
        //&data retrieves memory address of the data variable, returning pointer to where float is stored
        //reinterpret case is a type-casting operator that instructs the compiler to reinterpret the memory
        // address provided as a pointer to a const uint8_t type.
        //const uint8_t* byte_ptr declares a new variable called byte_ptr as a pointer to a const uint8_t type.
        const uint8_t* byte_ptr = reinterpret_cast<const uint8_t*>(&data);

        // Insert the bytes pointed to by 'byte_ptr' into the 'packet' vector
        // The size of the data is determined by sizeof(data)
        //little-endian format
        packet.insert(packet.end(), byte_ptr, byte_ptr + sizeof(data));
    }

    uint8_t checksum = 0; //Initialize the checksum to 0
    for (const auto& byte : packet) { //Iterating over each byte in the packet
        checksum ^= byte; //perform the xor operator. Excluding the marker because this starts before
    }
    packet.push_back(checksum); //Add the checksum byte to the packet

    return packet; //Returning the generated telemetry packet
}

int main() {
    std::vector<float> sensor_data = { 25.0, 1023.0, 35.6 }; //Example pressure data (temperature, pressure, humidity)
    std::vector<uint8_t> telemetry_packet = generate_telemetry_packet(sensor_data); //Generate the telemetry packet

    for (const auto& byte : telemetry_packet) { //Iterate over each byte in the telemetry packet
        std::cout << std::hex << static_cast<int>(byte) << " "; //Printing the byte in hexadecimal format
    }
    std::cout << std::endl; //printing a new line

    return 0;
}

