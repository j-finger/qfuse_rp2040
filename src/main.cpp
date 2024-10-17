// main.cpp
#include <stdio.h>
#include <string.h>
#include <vector>
#include <string>
#include <map>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"
#include "hardware/uart.h"
#include "pico/unique_id.h"
#include "pico/multicore.h"
#include "pico/sync.h"
#include "pico/util/queue.h"

#include "ICM42688.hpp" // Include the ICM42688 abstraction
#include "json.hpp"     // JSON library

using json = nlohmann::json;

// Define queue parameters
#define JSON_QUEUE_SIZE 100          // Define the maximum number of JSON messages in the queue
#define JSON_STRING_MAX_LENGTH 8000  // Increased max length to accommodate more data

// Create a queue for JSON strings
queue_t json_queue;

// UART
#define UART_ID uart1
#define BAUD_RATE 115200
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY UART_PARITY_NONE
#define UART_TX_PIN 5
#define UART_RX_PIN 4

// UART and SPI Macros
#define IMU_COUNT 4  // Number of IMUs connected

// SPI0 (First two IMUs)
#define SPI0_SCLK_PIN 10
#define SPI0_MOSI_PIN 11
#define SPI0_MISO_PIN 12

#define SPI0_PORT spi1
#define SPI0_BAUD_RATE (115200)  // 1 MHz

const uint SPI0_CS_PINS[] = {9, 13}; // CS pins for IMUs on SPI0

// SPI1 (Next two IMUs)
#define SPI1_SCLK_PIN 18
#define SPI1_MOSI_PIN 19
#define SPI1_MISO_PIN 20

#define SPI1_PORT spi0
#define SPI1_BAUD_RATE (115200)  // 1 MHz

const uint SPI1_CS_PINS[] = {17, 21}; // CS pins for IMUs on SPI1

// Global Function for Core 1
void uart_task_entry() {
    char json_str[JSON_STRING_MAX_LENGTH];
    while (true) {
        if (queue_try_remove(&json_queue, json_str)) {
            // Send the JSON string over UART
            uart_write_blocking(UART_ID, (const uint8_t*)json_str, strlen(json_str));
            uart_write_blocking(UART_ID, (const uint8_t*)"\n", 1);  // Optional: Add newline for readability
            printf("Sent JSON over UART:\n%s\n\n", json_str);
        }
    }
}

// RP2040 Controller Class
class RP2040Controller {
public:
    RP2040Controller() {
        // Initialize UART
        uart_initialize();

        // Initialize the queue
        queue_init(&json_queue, sizeof(char) * JSON_STRING_MAX_LENGTH, JSON_QUEUE_SIZE);

        // Get RP2040 device ID
        read_device_id();

        // Initialize SPI0
        spi_init(SPI0_PORT, SPI0_BAUD_RATE);
        gpio_set_function(SPI0_SCLK_PIN, GPIO_FUNC_SPI);
        gpio_set_function(SPI0_MOSI_PIN, GPIO_FUNC_SPI);
        gpio_set_function(SPI0_MISO_PIN, GPIO_FUNC_SPI);

        // Initialize SPI1
        spi_init(SPI1_PORT, SPI1_BAUD_RATE);
        gpio_set_function(SPI1_SCLK_PIN, GPIO_FUNC_SPI);
        gpio_set_function(SPI1_MOSI_PIN, GPIO_FUNC_SPI);
        gpio_set_function(SPI1_MISO_PIN, GPIO_FUNC_SPI);

        max_measurements = 2;  // Collect 5 samples per IMU

        // Initialize IMUs
        // IMUs on SPI0
        for (int i = 0; i < 2; i++) {
            IMU imu(SPI0_PORT, SPI0_CS_PINS[i], i);
            imus_.push_back(imu);
        }
        // IMUs on SPI1
        for (int i = 0; i < 2; i++) {
            IMU imu(SPI1_PORT, SPI1_CS_PINS[i], i + 2); // Indexing from 2 for the next IMUs
            imus_.push_back(imu);
        }

        // Initialize measurement data storage and counts
        imu_data_samples_.resize(imus_.size());
        measurement_counts_.resize(imus_.size(), 0); // Initialize counts to 0

        // // Print IMU settings
        // for (auto& imu : imus_) {
        //     json settings_json = imu.jsonify_settings();
        //     settings_json["device"] = device_id_;
        //     printf("Settings JSON:\n%s\n\n", settings_json.dump(4).c_str());
        // }

        // // Send settings JSON for each IMU over UART
        // for (auto& imu : imus_) {
        //     json settings_json = imu.jsonify_settings();
        //     enqueue_json(settings_json);  // Enqueue JSON for UART transmission
        //     printf("Enqueued Settings JSON:\n%s\n\n", settings_json.dump(4).c_str());
        // }

        // Launch Core 1 to handle UART transmissions
        multicore_launch_core1(uart_task_entry);
    }

    void run() {
        printf("Running RP2040 Controller\n");
        absolute_time_t last_read_time = get_absolute_time();

        while (true) {
            // Sleep to prevent tight loop
            sleep_ms(10);

            absolute_time_t current_time = get_absolute_time();

            // Check if it's time to read sensor data (e.g., every 1 second)
            if (absolute_time_diff_us(last_read_time, current_time) >= 1000000) {  // 1 second
                last_read_time = current_time;

                // Read sensor data from each IMU
                for (size_t imu_index = 0; imu_index < imus_.size(); ++imu_index) {
                    IMU& imu = imus_[imu_index];
                    SensorData data = imu.read_sensor_data();
                    json data_json = imu.jsonify_data(data);

                    // Add subdevice number
                    data_json["subdevice"] = std::to_string(imu_index);

                    // Add the measurement to the imu's data samples
                    imu_data_samples_[imu_index].push_back(data_json);

                    measurement_counts_[imu_index]++;
                }

                // Check if all IMUs have collected enough samples
                bool all_collected = true;
                for (size_t i = 0; i < measurement_counts_.size(); ++i) {
                    if (measurement_counts_[i] < max_measurements) {
                        all_collected = false;
                        break;
                    }
                }

                if (all_collected) {
                    // Build the UART packet
                    json uart_packet;
                    uart_packet["device"] = device_id_;
                    // Get the current time, convert to string
                    uart_packet["time"] = std::to_string(to_ms_since_boot(get_absolute_time()));

                    json data_array = json::array();

                    // Loop over measurements
                    for (uint8_t sample_index = 0; sample_index < max_measurements; ++sample_index) {
                        // Loop over IMUs
                        for (size_t imu_index = 0; imu_index < imus_.size(); ++imu_index) {
                            // Get the sample data for this IMU and sample index
                            json sample_data = imu_data_samples_[imu_index][sample_index];

                            data_array.push_back(sample_data);
                        }
                    }

                    uart_packet["data"] = data_array;

                    // Enqueue the UART packet
                    enqueue_json(uart_packet);
                    printf("Enqueued Data JSON:\n%s\n\n", uart_packet.dump(4).c_str());

                    // Reset measurement counts and clear data samples
                    for (size_t i = 0; i < measurement_counts_.size(); ++i) {
                        measurement_counts_[i] = 0;
                        imu_data_samples_[i].clear();
                    }
                }

                // Perform other non-blocking tasks here if needed
                tight_loop_contents();  // Yield to other processes
            }
        }
    }

private:
    std::vector<IMU> imus_;
    std::string device_id_;
    uint8_t max_measurements;
    std::vector<std::vector<json>> imu_data_samples_; // Stores samples per IMU
    std::vector<uint8_t> measurement_counts_; // Counts per IMU

    // Function to enqueue JSON strings for UART transmission
    void enqueue_json(const json& json_obj) {
        std::string json_str = json_obj.dump();
        // Ensure the JSON string does not exceed the maximum length
        if (json_str.length() >= JSON_STRING_MAX_LENGTH) {
            printf("JSON string too long to enqueue.\n");
            return;
        }

        // Enqueue the JSON string
        if (!queue_try_add(&json_queue, json_str.c_str())) {
            printf("Failed to enqueue JSON string.\n");
        }
    }

    // UART Initialization
    void uart_initialize() {
        // Set the GPIO pin mux to the UART function
        gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
        gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

        // Initialize the UART with the specified baud rate
        uart_init(UART_ID, BAUD_RATE);

        // Set UART data format
        uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);

        // Enable FIFO
        uart_set_fifo_enabled(UART_ID, true);

        // Disable flow control CTS/RTS
        uart_set_hw_flow(UART_ID, false, false);
    }

    // Function to send data over UART
    void uart_send(const char* data) {
        size_t len = strlen(data);
        uart_write_blocking(UART_ID, (const uint8_t*)data, len);
    }

    // Function to receive data over UART (to be further developed)
    void uart_receive(char* data) {
        int idx = 0;
        while (uart_is_readable(UART_ID)) {
            char ch = uart_getc(UART_ID);
            data[idx++] = ch;
            if (ch == '}') {  // Assuming JSON ends with '}'
                break;
            }
            if (idx >= 255) {  // Prevent buffer overflow
                break;
            }
        }
        data[idx] = '\0';  // Null-terminate the string
    }

    // Read device ID from unique board ID
    void read_device_id() {
        pico_unique_board_id_t id_out;
        pico_get_unique_board_id(&id_out);
        char id_str[2 * PICO_UNIQUE_BOARD_ID_SIZE_BYTES + 1];
        for (int i = 0; i < PICO_UNIQUE_BOARD_ID_SIZE_BYTES; i++) {
            sprintf(id_str + i * 2, "%02X", id_out.id[i]);
        }
        device_id_ = std::string(id_str);
        printf("Device ID: %s\n", device_id_.c_str());
    }
};

int main() {
    // Initialize stdio
    stdio_init_all();
    sleep_ms(5000);  // Wait for serial connection
    printf("Starting RP2040 Controller\n");

    // Create RP2040Controller instance
    RP2040Controller controller;

    printf("RP2040 Controller initialized\n");
    controller.run();

    return 0;
}
