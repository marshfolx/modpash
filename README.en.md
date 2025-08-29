# Modpash

Modpash is a C++ library for Modbus protocol communication, suitable for embedded systems and the Arduino platform. It provides both client and server functionalities, supports non-blocking serial communication, and can be easily integrated into various projects.

## Features

- Supports the Modbus RTU protocol
- Non-blocking serial communication
- CRC16 checksum
- Cross-platform support for Arduino and other embedded systems

## Examples

- `example/basic_client.cpp` - Basic Modbus client example
- `example/demo_server.cpp` - Modbus server example
- `example/non_block_serial_echo.cpp` - Non-blocking serial echo example

## Dependencies

- Arduino development environment
- `scheduler_basic` library
- `oled_basic` library (required only for OLED examples)

## Installation

1. Download and install the Arduino IDE.
2. Install the required libraries (e.g., `scheduler_basic` and `oled_basic`).
3. Copy the Modpash library folder into the libraries folder of Arduino.
4. Restart the Arduino IDE.

## Usage

Please refer to the example code to learn how to use Modpash for Modbus communication.

## License

This project uses the MIT License. For details, please see the LICENSE file.