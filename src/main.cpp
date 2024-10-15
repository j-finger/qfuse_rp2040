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
#include "json.hpp" // JSON library

using json = nlohmann::json;

// Define queue parameters
#define JSON_QUEUE_SIZE 100 // Define the maximum number of JSON messages in the queue
#define JSON_STRING_MAX_LENGTH 512 // Define the maximum length of each JSON string

// Create a queue for JSON strings
queue_t json_queue;

// UART and SPI Macros
#define IMU_COUNT 1 // Number of IMUs connected per channel
#define DEVICE_ID_LENGTH 16

#define UART_ID uart1
#define BAUD_RATE 115200
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY    UART_PARITY_NONE
#define UART_TX_PIN 8
#define UART_RX_PIN 9

// SPI2
#define SPI0_SCLK_PIN 10
#define SPI0_MOSI_PIN 11
#define SPI0_MISO_PIN 12

#define SPI0_PORT spi1
#define SPI0_BAUD_RATE (1000 * 1000) // 1 MHz

const uint SPI0_CS_PINS[IMU_COUNT] = {9};
// const uint SPI0CS_PINS[IMU_COUNT] = {9, 13};

// SPI1
#define SPI1_SCLK_PIN 18
#define SPI1_MOSI_PIN 19
#define SPI1_MISO_PIN 20

#define SPI1_PORT spi0
#define SPI1_BAUD_RATE (1000 * 1000) // 1 MHz

const uint SPI1_CS_PINS[IMU_COUNT] = {17};
// const uint SPI1CS_PINS[IMU_COUNT] = {17, 21};

// RP2040 Controller Class
class RP2040Controller {
public:
    RP2040Controller() {
        // Initialize UART
        // uart_initialize();

        // Initialize the queue
        // queue_init(&json_queue, sizeof(char) * JSON_STRING_MAX_LENGTH, JSON_QUEUE_SIZE);

        // Get RP2040 device ID
        read_device_id();

        // Initialize SPI
        spi_init(SPI0_PORT, SPI0_BAUD_RATE);
        gpio_set_function(SPI0_SCLK_PIN, GPIO_FUNC_SPI);
        gpio_set_function(SPI0_MOSI_PIN, GPIO_FUNC_SPI);
        gpio_set_function(SPI0_MISO_PIN, GPIO_FUNC_SPI);

        // Initialize IMUs
        for (int i = 0; i < IMU_COUNT; i++) {
            IMU imu(SPI0_PORT, SPI0_CS_PINS[i], i);
            imus_.push_back(imu);
        }


        // Print IMU settings
        for (auto& imu : imus_) {
            json settings_json = imu.jsonify_settings(device_id_);
            printf("Settings JSON:\n%s\n\n", settings_json.dump(4).c_str());
        }


        // // Send settings JSON for each IMU over UART
        // for (auto& imu : imus_) {
        //     json settings_json = imu.jsonify_settings(device_id_);
        //     enqueue_json(settings_json); // Enqueue JSON for UART transmission
        //     printf("Enqueued Settings JSON:\n%s\n\n", settings_json.dump(4).c_str());
        // }

        // Launch Core 1 to handle UART transmissions
        // multicore_launch_core1(uart_task_entry);
    }

    void run() {
        printf("Running RP2040 Controller\n");
        // absolute_time_t last_read_time = get_absolute_time();

        while (true) {

            sleep_ms(1000); // Sleep for 1 second
            for (auto& imu : imus_) {
                SensorData data = imu.read_sensor_data();
                json data_json = imu.jsonify_data(data, device_id_);
                printf("Data JSON:\n%s\n\n", data_json.dump(4).c_str());
            }


            // absolute_time_t current_time = get_absolute_time();

            // // Check if it's time to read sensor data (e.g., every 1 second)
            // if (absolute_time_diff_us(last_read_time, current_time) >= 1000000) { // 1 second
            //     last_read_time = current_time;

            //     // Read sensor data from each IMU
            //     for (auto& imu : imus_) {
            //         SensorData data = imu.read_sensor_data();
            //         json data_json = imu.jsonify_data(data, device_id_);
            //         enqueue_json(data_json); // Enqueue JSON for UART transmission
            //         printf("Enqueued Data JSON:\n%s\n\n", data_json.dump(4).c_str());
            //     }
            // }

            // // Perform other non-blocking tasks here if needed
            // tight_loop_contents(); // Yield to other processes
        }
    }

private:
    std::vector<IMU> imus_;
    std::string device_id_;

    // UART and Queue Definitions
    // queue_t json_queue;

    // // Function to send JSON over UART
    // void send_json(const json& json_obj) {
    //     std::string json_str = json_obj.dump();
    //     // Ensure the JSON string does not exceed the maximum length
    //     if (json_str.length() >= JSON_STRING_MAX_LENGTH) {
    //         printf("JSON string too long to enqueue.\n");
    //         return;
    //     }

    //     // Enqueue the JSON string
    //     if (queue_add_blocking(&json_queue, json_str.c_str()) != PICO_OK) {
    //         printf("Failed to enqueue JSON string.\n");
    //     }
    // }

    // Function to enqueue JSON strings for UART transmission
    // void enqueue_json(const json& json_obj) {
    //     std::string json_str = json_obj.dump();
    //     // Ensure the JSON string does not exceed the maximum length
    //     if (json_str.length() >= JSON_STRING_MAX_LENGTH) {
    //         printf("JSON string too long to enqueue.\n");
    //         return;
    //     }

    //     // Enqueue the JSON string
    //     if (queue_add_blocking(&json_queue, json_str.c_str()) != PICO_OK) { // Changed condition
    //         printf("Failed to enqueue JSON string.\n");
    //     }
    // }

    // // UART Initialization
    // void uart_initialize() {
    //     // Set the GPIO pin mux to the UART function
    //     gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    //     gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

    //     // Initialize the UART with the specified baud rate
    //     uart_init(UART_ID, BAUD_RATE);

    //     // Set UART data format
    //     uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);

    //     // Enable FIFO
    //     uart_set_fifo_enabled(UART_ID, true);

    //     // Disable flow control CTS/RTS
    //     uart_set_hw_flow(UART_ID, false, false);
    // }

    // // Function to send data over UART
    // void uart_send(const char* data) {
    //     size_t len = strlen(data);
    //     uart_write_blocking(UART_ID, (const uint8_t*)data, len);
    // }

    // // Function to receive data over UART (to be further developed)
    // void uart_receive(char* data) {
    //     int idx = 0;
    //     while (uart_is_readable(UART_ID)) {
    //         char ch = uart_getc(UART_ID);
    //         data[idx++] = ch;
    //         if (ch == '}') { // Assuming JSON ends with '}'
    //             break;
    //         }
    //         if (idx >= 255) { // Prevent buffer overflow
    //             break;
    //         }
    //     }
    //     data[idx] = '\0'; // Null-terminate the string
    // }

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

    // // UART Task Entry (Core 1)
    // static void uart_task_entry() {
    //     char json_str[JSON_STRING_MAX_LENGTH];
    //     while (true) {
    //         // Dequeue a JSON string (blocking call)
    //         if (queue_remove_blocking(&json_queue, json_str)) {
    //             // Send the JSON string over UART
    //             uart_write_blocking(UART_ID, (const uint8_t*)json_str, strlen(json_str));
    //             uart_write_blocking(UART_ID, (const uint8_t*)"\n", 1); // Optional: Add newline for readability
    //             printf("Sent JSON over UART:\n%s\n\n", json_str);
    //         }
    //     }
    // }
};

int main() {
    // Initialize stdio
    stdio_init_all();
    sleep_ms(5000); // Wait for serial connection
    printf("Starting RP2040 Controller\n");

    // Create RP2040Controller instance
    RP2040Controller controller;

    printf("RP2040 Controller initialized\n");
    controller.run();

    return 0;
}
