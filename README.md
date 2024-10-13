# qfuse RP2040 Firmware
## Overview

The `qfuse` RP2040 Firmware is designed to interface with the ICM-42688 IMU sensors connected to the RP2040 microcontroller. It performs sensor data collection, configuration, and serialization into JSON format for transmission over UART. The firmware is modular, utilizing both functional and object-oriented programming paradigms to ensure maintainability and scalability.

## Features

- **Sensor Fusion**: Collects data from multiple IMU sensors (accelerometer, gyroscope, temperature).
- **SPI Communication**: Interfaces with ICM-42688 IMU sensors using SPI protocol.
- **JSON Serialization**: Converts sensor data and settings into JSON format for easy transmission and integration.
- **UART Communication**: Sends serialized JSON data to the ESP32C6 module via UART.
- **Configurable Settings**: Allows customization of sensor parameters such as ODR (Output Data Rate), FSR (Full Scale Range), and power modes.
- **Device Identification**: Generates a unique device ID based on the RP2040's unique board ID.
- **Modular Design**: Organized into classes and structures for easy maintenance and future enhancements.

## Prerequisites

- **Hardware:**
    - `qfuse` Dev board 
        - RP2040-based board (e.g., Raspberry Pi Pico)
        - ICM-42688 IMU sensor(s)
        - ESP32C6 module for UART communication
        - Connecting wires
        - Breadboard or custom PCB for connections

- **Software:**
  - C++ Compiler (supported by the Raspberry Pi Pico SDK)
  - [Pico SDK](https://github.com/raspberrypi/pico-sdk) version 1.5.1
  - [CMake](https://cmake.org/) version 3.13 or higher
  - [nlohmann/json](https://github.com/nlohmann/json) library for JSON handling

## Installation

1. **Clone the Repository**

   ```bash
   git clone https://github.com/yourusername/qfuse-rp2040-firmware.git
   cd qfuse-rp2040-firmware
   ```

2. **Set Up the Pico SDK**

   Follow the [official Pico SDK setup guide](https://github.com/raspberrypi/pico-sdk) to install and initialize the SDK.

   ```bash
   git clone https://github.com/raspberrypi/pico-sdk.git
   export PICO_SDK_PATH=/path/to/pico-sdk
   ```

3. **Install Dependencies**

   Ensure that the `nlohmann/json` library is available. You can include it as a submodule or install it via your package manager.

   ```bash
   # Example using vcpkg
   git clone https://github.com/microsoft/vcpkg.git
   cd vcpkg
   ./bootstrap-vcpkg.sh
   ./vcpkg integrate install
   ./vcpkg install nlohmann-json
   ```

   Alternatively, download the single-header file from [here](https://github.com/nlohmann/json/releases) and place it in the `include/` directory.

## Setup

### Hardware Connections

1. **SPI Connections between RP2040 and ICM-42688:**

   | RP2040 Pin | ICM-42688 Pin | Function       |
   |------------|---------------|----------------|
   | GPIO 18    | SCLK          | SPI Clock      |
   | GPIO 19    | MOSI          | SPI Master Out |
   | GPIO 20    | MISO          | SPI Master In  |
   | GPIO 21    | CS            | Chip Select    |
   | GND        | GND           | Ground         |
   | 3.3V       | VCC           | Power          

2. **UART Connections between RP2040 and ESP32C6:**

   | RP2040 Pin | ESP32C6 Pin | Function  |
   |------------|-------------|-----------|
   | GPIO 8     | TXD         | UART TX   |
   | GPIO 9     | RXD         | UART RX   |
   | GND        | GND         | Ground    |

   **Note:** Ensure voltage levels are compatible or use level shifters if necessary.

### Firmware Configuration

1. **Configure Sensor Settings:**

   Modify the `main.cpp` file to adjust sensor settings such as ODR, FSR, and power modes as per your application requirements.

2. **Device ID:**

   The firmware automatically generates a unique device ID based on the RP2040's unique board ID. No manual configuration is required.

## Building and Uploading

1. **Create a Build Directory**

   ```bash
   mkdir build
   cd build
   ```

2. **Generate Build Files with CMake**

   ```bash
   cmake .. -DCMAKE_BUILD_TYPE=Release
   ```

3. **Build the Firmware**

   ```bash
   make
   ```

4. **Upload the Firmware to RP2040**

   - **Manual Method:**
     - Hold down the BOOTSEL button on the RP2040 board.
     - Connect it to your computer via USB. It should mount as a mass storage device.
     - Drag and drop the generated `.uf2` file from the `build` directory to the RP2040's storage.

   - **Using `picotool`:**
     ```bash
     picotool load qfuse-rp2040-firmware.uf2
     ```

## Usage

### Running the Firmware

After uploading the firmware, the RP2040 will initialize the IMU sensors and begin collecting data. It will serialize the data into JSON format and transmit it over UART to the ESP32C6 module.

### UART Communication

- **Sending Data:**

  The firmware sends JSON-formatted sensor data and settings over UART. Ensure that the ESP32C6 is configured to receive and process this data.

- **Receiving Data:**

  Currently, the firmware includes stubs for sending and receiving data over UART. Implement the `send_json` and UART receive functionalities as needed to integrate with your backend systems.

## Project Structure

```
qfuse-rp2040-firmware/
├── include/
│   ├── ICM42688.h          # IMU register and settings definitions
│   └── json.hpp            # nlohmann/json single-header library
├── src/
│   └── main.cpp            # Main firmware source code
├── CMakeLists.txt          # CMake build configuration
├── README.md               # Firmware documentation
└── build/                  # Build directory (generated after CMake)
```

- **include/**: Contains header files and external libraries.
- **src/**: Contains the main firmware source code.
- **CMakeLists.txt**: Configuration file for CMake to build the project.
- **build/**: Directory where build artifacts are generated.

## Code Description

### ICM42688.h

**Location:** `include/ICM42688.h`

**Description:**

Defines register addresses and settings constants for the ICM-42688 IMU sensor. Organized into namespaces for registers (`ICM42688REG`) and settings (`ICM42688SET`), facilitating easy reference and maintenance.

**Key Components:**

- **Register Definitions:** Constants representing various register addresses in different user banks.
- **Settings Definitions:** Constants representing various configuration settings like ODR, FSR for accelerometer and gyroscope.

### main.cpp

**Location:** `src/main.cpp`

**Description:**

The core firmware logic for the RP2040 microcontroller. It initializes SPI communication with the IMU sensors, configures sensor settings, reads sensor data, serializes it into JSON, and transmits it over UART.

#### Classes

##### IMU Driver

**Description:**

Represents an individual ICM-42688 IMU sensor. Handles sensor initialization, configuration, data reading, and JSON serialization.

**Key Methods:**

- `IMU(spi_inst_t* spi_port, uint cs_pin, uint8_t subdevice_id)`: Constructor that initializes SPI communication and configures the sensor.
- `void initialize()`: Initializes SPI and configures the sensor. Verifies sensor presence using the `WHO_AM_I` register.
- `SensorData read_sensor_data()`: Reads accelerometer, gyroscope, temperature data, and IMU timestamp from the sensor.
- `json jsonify_data(const SensorData& data_in, const std::string& device_id)`: Converts sensor data into a JSON object.
- `json jsonify_settings(const std::string& device_id)`: Converts sensor settings into a JSON object.
- `uint32_t read_imu_timestamp()`: Reads the 24-bit IMU timestamp from the sensor.

**Private Methods:**

- `void cs_select() / cs_deselect()`: Manages Chip Select line for SPI communication.
- `void write_register(uint8_t reg_addr, uint8_t data)`: Writes a byte to a specified register.
- `uint8_t read_register(uint8_t reg_addr)`: Reads a byte from a specified register.
- `void read_registers(uint8_t reg_addr, uint8_t* data, size_t length)`: Reads multiple bytes starting from a specified register.
- `void select_register_bank(uint8_t bank)`: Switches between different user banks in the sensor.
- `void print_register(uint8_t reg_addr)`: Prints the value of a register for debugging.
- `void setup_spi()`: Configures the SPI pins.
- `void reset_device_configuration()`: Resets the device configuration to default.
- `void configure_sensor()`: Configures sensor settings like ODR, FSR, power modes.
- `AccelerometerData read_accel()`: Reads accelerometer data.
- `GyroscopeData read_gyro()`: Reads gyroscope data.
- `TemperatureData read_temp()`: Reads temperature data.
- `void set_accel_odr(const uint odr)`: Sets accelerometer ODR.
- `void set_accel_fsr(const uint fsr)`: Sets accelerometer FSR.
- `void set_gyro_odr(const uint odr)`: Sets gyroscope ODR.
- `void set_gyro_fsr(const uint fsr)`: Sets gyroscope FSR.
- `void calculate_sensitivity()`: Calculates sensitivity based on FSR settings.
- `void set_power_modes(const SensorSetting& accel_mode, const SensorSetting& gyro_mode, bool temp_disabled)`: Sets power modes for accelerometer and gyroscope.
- `void set_clock()`: Configures the sensor clock settings.

##### RP2040Controller

**Description:**

Manages the overall operation of the RP2040, including initializing IMU sensors, reading data, and handling UART communication.

**Key Methods:**

- `RP2040Controller()`: Constructor that initializes device ID, SPI, and IMU sensors. Sends initial settings JSON.
- `void run()`: Main loop that continuously reads sensor data, serializes it, and sends it over UART.
- `void read_device_id()`: Retrieves a unique device ID from the RP2040's unique board ID.

#### Functions

##### UART Functions

- `void uart_initialize()`: Configures UART pins and initializes UART with specified settings.
- `void uart_send(const char* data)`: Sends a null-terminated string over UART.
- `void uart_receive(char* data)`: Receives data over UART and stores it in the provided buffer.

**Note:** These functions are currently standalone but are marked with TODO comments to integrate them into the `RP2040Controller` class for better encapsulation.

#### Data Structures

- **SensorSetting, BoolSetting, AAFSetting:** Structures to represent various sensor configurations.
- **AccelerometerData, GyroscopeData, TemperatureData, SensorData:** Structures to hold sensor readings.

#### JSON Handling

Utilises the `nlohmann::json` library to serialise sensor data and settings into JSON format, facilitating easy transmission and integration with backend systems.

## JSON Data Format

For testing purposes we will use the following JSON packets to simulate the data that will be transmitted over MQTT. 


**Data** `data.json`
```json
{
	"device": "E46338809B472231",
    "time": "1728792656",
	"data": [{
		"subdevice": "1",
		"timestamp": "0000C8",
		"accel": {
			"x": 0.0,
			"y": 0.0,
			"z": 0.0
		},
		"gyro": {
			"x": 0.0,
			"y": 0.0,
			"z": 0.0
		},
		"temperature": 25.0
	},
	{
		"subdevice": "1",
		"timestamp": "0000C9",
		"accel": {
			"x": 0.0,
			"y": 0.0,
			"z": 0.0
		},
		"gyro": {
			"x": 0.0,
			"y": 0.0,
			"z": 0.0
		},
		"temperature": 25.1
	}]
}
```

**Settings** `settings.json` - without filtering - currently being used in development
```json
{
    "device": "E46338809B472231",
    "time": "1728792656",
    "settings": {
	    "subdevice": "1",
        "accel_odr": "100Hz",
        "accel_fsr": "±4g",
        "accel_sensitivity": 8192.0,
        "gyro_odr": "100Hz",
        "gyro_fsr": "±500dps",
        "gyro_sensitivity": 65.5,
        "power": {
            "accel_mode": "Low Noise",
            "gyro_mode": "Low Noise"
        }
    }
}

```

**Logs** `logs.json`
```json
{
	"device": "E46338809B472231",
    "time": "1728792656",
	"log": "some log line"
}
```

- **Timestamp** will be in the form of 'ticks' from the IMU. Left in the form of a 24bit hexadecimal as output from the IMU's. These are dependant on the IMU ODR and the external system clock. These are only used for *actual 'data'* from the IMU's and will not be included in logs or settings JSONs. 
- **Time** will be in the form of UNIX epoch time. This will be used in IMU data, logs and settings JSONs. It will be handled solely on the ESP32C6 side. The RP2040 will pass IMU data, settings, and logs to the ESP32C6, which will append the current time to the JSON data before sending it to the server.


## License

This project is licensed under the [MIT License](LICENSE).
