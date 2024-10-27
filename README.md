# RP2040 Firmware for `qfuse` IMU Data Acquisition System

## Overview

This repository contains the firmware for the RP2040 microcontroller used in the `qfuse` development board. The firmware is responsible for interfacing with multiple ICM-42688-P IMU sensors via SPI, collecting high-precision accelerometer and gyroscope data, and transmitting the serialized data to the ESP32C6 module over UART. The collected data is intended for sensor fusion applications in motion tracking and vibration analysis, forming the foundation for product development.

## Features

- **SPI Communication with Multiple IMUs**: Supports reading data from multiple ICM-42688-P sensors using SPI0 and SPI1 interfaces.
- **Data Serialization**: Sensor data is serialized into JSON format for consistent data structure and ease of parsing downstream.
- **UART Communication**: Transmits serialized data to the ESP32C6 module via UART at a baud rate of 921600.
- **Multicore Processing**: Utilizes both cores of the RP2040; Core 0 handles data collection, and Core 1 handles UART transmission.
- **High Data Rate**: Configurable output data rate (ODR) up to 100Hz for both accelerometer and gyroscope data.
- **Time Synchronization**: Each data packet includes a timestamp from the IMU and the RP2040's unique device ID.
- **Modular Design**: Code is organized into classes and modules for scalability and maintainability.

## Hardware Setup

### Components

- **RP2040 Microcontroller**: Acts as the main controller for data acquisition and processing.
- **ICM-42688-P IMU Sensors**: High-precision accelerometer and gyroscope sensors.
- **ESP32C6 Module**: Handles communication with the backend server over Wi-Fi.
- **`qfuse` Development Board**: Custom PCB integrating the above components.
- **UART Connection**: Connects the RP2040's UART1 (TX on GPIO 5, RX on GPIO 4) to the ESP32C6 module.
- **SPI Connections**:
  - **SPI0**: Uses GPIO 18 (SCK), GPIO 19 (MOSI), GPIO 20 (MISO) with CS pins on GPIO 17 and GPIO 21.
  - **SPI1**: Uses GPIO 10 (SCK), GPIO 11 (MOSI), GPIO 12 (MISO) with CS pins on GPIO 9 and GPIO 13.

### Pin Assignments

| Interface | Signal | GPIO Pin | Device  |
| --------- | ------ | -------- | ------- |
| UART1     | TXD    | GPIO 5   | ESP32C6 |
| UART1     | RXD    | GPIO 4   | ESP32C6 |
| SPI0      | SCK    | GPIO 18  | IMU0&1  |
| SPI0      | MOSI   | GPIO 19  | IMU0&1  |
| SPI0      | MISO   | GPIO 20  | IMU0&1  |
| SPI0      | CS     | GPIO 17  | IMU0&1  |
| SPI0      | CS     | GPIO 21  | IMU0&1  |
| SPI1      | SCK    | GPIO 10  | IMU2&3  |
| SPI1      | MOSI   | GPIO 11  | IMU2&3  |
| SPI1      | MISO   | GPIO 12  | IMU2&3  |
| SPI1      | CS     | GPIO 9   | IMU2&3  |
| SPI1      | CS     | GPIO 13  | IMU2&3  |

### IMU Configuration

- **IMU 0 and IMU 1** (Connected to SPI0):
  - Gyroscope FSR: ±2000 dps
  - Accelerometer FSR: ±16 g
- **IMU 2 and IMU 3** (Connected to SPI1):
  - Gyroscope FSR: ±500 dps
  - Accelerometer FSR: ±4 g
- **Output Data Rate**:
  - Gyroscope ODR: 100 Hz
  - Accelerometer ODR: 100 Hz

## Software Dependencies

- **Pico SDK**: Required for building firmware for the RP2040 microcontroller.
- **CMake**: Version 3.13 or higher.
- **GNU Arm Embedded Toolchain**: For cross-compiling to ARM Cortex-M0+.
- **nlohmann/json**: JSON library for C++ (header-only).
- **Standard C++ Libraries**: Requires C++17 support.

## Directory Structure

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


- `src/`: Contains the source code files.
  - `main.cpp`: Entry point of the firmware.
  - `ICM42688.hpp` and `ICM42688.cpp`: Implementation of the IMU class for interfacing with the ICM-42688-P sensor.
  - `json.hpp`: Header file for the JSON library.
- `CMakeLists.txt`: CMake build configuration file.

## Build Instructions

### Prerequisites

1. **Install the Pico SDK**:
   - Follow the instructions at [Getting Started with Raspberry Pi Pico](https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf) to set up the Pico SDK on your system.
   - Ensure that the `PICO_SDK_PATH` environment variable is set to the location of the Pico SDK.
2. **Install CMake**:
   - Version 3.13 or higher is required.
3. **Install the GNU Arm Embedded Toolchain**:
   - Ensure that the `arm-none-eabi-gcc` compiler is installed and accessible in your PATH.
4. **Install nlohmann/json**:
   - The `json.hpp` file is included in the `src/` directory.

### Building the Firmware

1. **Clone the Repository**:
   ```bash
   git clone https://github.com/yourusername/qfuse-rp2040-firmware.git
   cd qfuse-rp2040-firmware
   ```
2. **Create a Build Directory**:
   ```bash
   mkdir build
   cd build
   ```
3. **Configure the Build**:
   ```bash
   cmake ..
   ```
   - Ensure that the `PICO_SDK_PATH` is correctly set.
   - If the Pico SDK is located in a non-standard location, you can specify it directly:
     ```bash
     cmake -DPICO_SDK_PATH=/path/to/pico-sdk ..
     ```
4. **Build the Firmware**:
   ```bash
   make
   ```
   - This will generate a `.uf2` firmware file (e.g., `labs.uf2`) in the `build` directory.

### Flashing the Firmware

1. **Connect the RP2040 Board**:
   - Hold down the BOOTSEL button on the RP2040 board.
   - Connect the board to your computer via USB.
   - Release the BOOTSEL button after connecting.
2. **Copy the Firmware**:
   - The RP2040 will appear as a mass storage device named `RPI-RP2`.
   - Copy the generated `.uf2` file to the `RPI-RP2` drive.
3. **Reset the Board**:
   - The board will automatically reboot and start running the new firmware.

**Alternatively Using `picotool`:**
```bash
picotool load qfuse-rp2040-firmware.uf2
```

## Usage Instructions

After uploading the firmware, the RP2040 will initialize the IMU sensors and begin collecting data. It will serialize the data into JSON format and transmit it over UART to the ESP32C6 module.

1. **Power On the System**:
   - Ensure that the `qfuse` board is powered and connected to the ESP32C6 module.
2. **Start the ESP32C6 Module**:
   - The ESP32C6 should be running its corresponding firmware to receive data over UART and transmit it over Wi-Fi.
3. **Data Transmission**:
   - The RP2040 will collect data from the IMUs at the configured ODR and transmit serialized JSON data over UART to the ESP32C6.
   - The data packets include accelerometer and gyroscope readings, timestamps, and the device ID.
4. **Data Reception**:
   - The ESP32C6 module will forward the data to the backend server via MQTT.

## Code Organization

### `main.cpp`

**Location:** `src/main.cpp`
The core firmware logic for the RP2040 microcontroller. It initializes SPI communication with the IMU sensors, configures sensor settings, reads sensor data, serializes it into JSON, and transmits it over UART.
- **Class `RP2040Controller`**:
  - Initializes UART and SPI interfaces.
  - Manages a vector of `IMU` objects.
  - Handles data collection and transmission loops.
  - Uses multicore processing to offload UART transmission to Core 1.
- **Functions**:
  - `uart_task_entry()`: Entry point for Core 1 to handle UART transmission.
  - `main()`: Initializes the controller and starts the data collection process.

### `ICM42688.hpp` and `ICM42688.cpp`

**Location:** `include/ICM42688.h`

Defines register addresses and settings constants for the ICM-42688 IMU sensor. Organized into namespaces for registers (`ICM42688REG`) and settings (`ICM42688SET`), facilitating easy reference and maintenance.

**Key Components:**

- **Register Definitions:** Constants representing various register addresses in different user banks.
- **Settings Definitions:** Constants representing various configuration settings like ODR, FSR for accelerometer and gyroscope.
- **Class `IMU`**:
  - Encapsulates the functionality for interfacing with an ICM-42688-P sensor.
  - Handles SPI communication, sensor configuration, and data reading.
  - Provides methods to serialize sensor data into JSON strings.

### `CMakeLists.txt`

- Configures the build process.
- Includes settings for cross-compilation to the RP2040.
- Links necessary libraries and sets up the build targets.

## Configuration

- **Adjusting IMU Settings**:
  - The IMU settings such as ODR and FSR can be adjusted in the `IMU` class methods `set_accel_odr()`, `set_accel_fsr()`, `set_gyro_odr()`, and `set_gyro_fsr()`.
  - In `main.cpp`, the IMU instances are initialized with specific settings; these can be modified to suit your requirements.
- **UART Baud Rate**:
  - The UART baud rate is set to `921600` in `main.cpp`. Ensure that the ESP32C6 module is configured to use the same baud rate.
- **Data Packet Size**:
  - The maximum number of measurements per packet is defined by `PACKET_STACK_SIZE`. Adjust this value based on memory constraints and data throughput requirements.
### Components
##### IMU Driver

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

## JSON Data Format

For testing purposes the following JSON packet structure is used that will be transmitted over MQTT. 

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

## Acknowledgments

- **[Pico SDK](https://github.com/raspberrypi/pico-sdk):** For providing the development environment for the RP2040.
- **[nlohmann/json](https://github.com/nlohmann/json)**: For the JSON serialization library.
- **InvenSense**: For the ICM-42688-P sensor datasheets and technical resources.
