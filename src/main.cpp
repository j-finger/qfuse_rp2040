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
#define JSON_QUEUE_SIZE 3         // Define the maximum number of JSON messages in the queue
#define JSON_STRING_MAX_LENGTH 25000  // Increased max length to accommodate more data
const uint ODR_SEL = 6;
const uint PACKET_STACK_SIZE = 50;

// Create a queue for JSON strings
queue_t json_queue;



// 921600
// UART
#define UART_ID uart1
#define BAUD_RATE 921600
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY UART_PARITY_NONE
#define UART_TX_PIN 5
#define UART_RX_PIN 4

// UART and SPI Macros
// #define IMU_COUNT 2  // Number of IMUs connected
#define IMU_COUNT 2  // Number of IMUs connected

// SPI0 (First two IMUs)
#define SPI1_SCLK_PIN 10
#define SPI1_MOSI_PIN 11
#define SPI1_MISO_PIN 12

#define SPI1_PORT spi1
#define SPI1_BAUD_RATE (921600)  // 1 MHz

// const uint SPI0_CS_PINS[] = {9}; // CS pins for IMUs on SPI0
const uint SPI1_CS_PINS[] = {9, 13}; // CS pins for IMUs on SPI0

// SPI1 (Next two IMUs)
#define SPI0_SCLK_PIN 18
#define SPI0_MOSI_PIN 19
#define SPI0_MISO_PIN 20

#define SPI0_PORT spi0
#define SPI0_BAUD_RATE (921600)  // 1 MHz

// const uint SPI1_CS_PINS[] = {17}; // CS pins for IMUs on SPI1
const uint SPI0_CS_PINS[] = {17, 21}; // CS pins for IMUs on SPI1



// Global Function for Core 1
void uart_task_entry() {
    char* json_str;
    while (true) {
        if (queue_try_remove(&json_queue, &json_str)) {
            // Send the JSON string over UART
            uart_write_blocking(UART_ID, (const uint8_t*)json_str, strlen(json_str));
            // Optionally, print it to the console for debugging
            // printf("Sent JSON over UART:\n%s\n\n", json_str);
            printf("Sent JSON over UART:\n");

            // Free the allocated memory
            free(json_str);
        }
    }
}



// RP2040 Controller Class
class RP2040Controller {
public:
    RP2040Controller() {
        // Initialize UART
        uart_initialize();

        // TODO: Wait for the 'Connected' message over UART
        wait_for_connected_message();  

        // Initialize the queue to hold pointers to char arrays
        queue_init(&json_queue, sizeof(char*), JSON_QUEUE_SIZE);

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

        initialize_cs_pins();

        /* Initialize IMUs */
        // Initialize IMUs on SPI0 (High Range IMU0)s
        for (int i = 0; i < IMU_COUNT/2; i++) {
            IMU imu(SPI0_PORT, SPI0_CS_PINS[i], i);
            imu.set_gyro_fsr(ICM42688SET::GYRO_FS_SEL_2000); // 0b000, 2000 dps
            imu.set_accel_fsr(ICM42688SET::ACCEL_FS_SEL_16); // 0b000, 16g
            imu.set_accel_odr(7); // 0b1000, 100Hz
            imu.set_gyro_odr(7);  // 0b1000, 100Hz
            imus_.push_back(imu);
        }
        // Initialize IMUs on SPI1 (Low Range IMU1)
        for (int i = 0; i < IMU_COUNT/2; i++) {
            IMU imu(SPI1_PORT, SPI1_CS_PINS[i], i + IMU_COUNT/2); // Indexing from 2 for the next IMUs
            imu.set_gyro_fsr(ICM42688SET::GYRO_FS_SEL_500);  // 0b010, 500 dps
            imu.set_accel_fsr(ICM42688SET::ACCEL_FS_SEL_4);  // 0b010, 4g
            imu.set_accel_odr(7); // 0b1000, 100Hz
            imu.set_gyro_odr(7);  // 0b1000, 100Hz
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
            sleep_ms(1);

            absolute_time_t current_time = get_absolute_time();

            // Check if it's time to read sensor data (e.g., every 1 ms)
            if (absolute_time_diff_us(last_read_time, current_time) >= 10000) {  // 10 ms
                last_read_time = current_time;

                // Read sensor data from each IMU
                for (size_t imu_index = 0; imu_index < imus_.size(); ++imu_index) {
                    IMU& imu = imus_[imu_index];
                    SensorData data = imu.read_sensor_data();
                    std::string data_str = imu.jsonify_data(data);

                    // Store the JSON string in the imu's data samples
                    imu_data_samples_[imu_index].push_back(data_str);

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
                    // Build the UART packet manually
                    char json_buffer[JSON_STRING_MAX_LENGTH];
                    int total_written = 0;

                    // Start building the JSON string
                    int written = snprintf(json_buffer + total_written, sizeof(json_buffer) - total_written,
                                        "{\"device\":\"%s\",\"data\":[",
                                        device_id_.c_str());

                    if (written < 0 || written >= (int)(sizeof(json_buffer) - total_written)) {
                        printf("Error building JSON string (device).\n");
                        // Reset counts and continue
                        reset_measurement_data();
                        continue;  // Skip this iteration
                    }
                    total_written += written;

                    bool first_entry = true;
                    bool error_occurred = false;

                    // Loop over measurements
                    for (uint8_t sample_index = 0; sample_index < max_measurements && !error_occurred; ++sample_index) {
                        // Loop over IMUs
                        for (size_t imu_index = 0; imu_index < imus_.size(); ++imu_index) {
                            // Get the sample data for this IMU and sample index
                            const std::string& sample_data_str = imu_data_samples_[imu_index][sample_index];

                            // Append comma if not the first entry
                            if (!first_entry) {
                                if (total_written >= sizeof(json_buffer) - 1) {
                                    error_occurred = true;
                                    printf("Buffer overflow when adding comma.\n");
                                    break;
                                }
                                json_buffer[total_written++] = ',';
                            } else {
                                first_entry = false;
                            }

                            // Append the sample data string
                            int remaining_buffer = sizeof(json_buffer) - total_written;
                            if (remaining_buffer <= 0) {
                                error_occurred = true;
                                printf("Buffer overflow before adding sample data.\n");
                                break;
                            }
                            written = snprintf(json_buffer + total_written, remaining_buffer,
                                            "%s", sample_data_str.c_str());
                            if (written < 0 || written >= remaining_buffer) {
                                printf("Error building JSON string (sample data).\n");
                                error_occurred = true;
                                break;  // Exit the loop
                            }
                            total_written += written;
                        }

                        if (error_occurred) {
                            break;  // Exit outer loop if an error occurred
                        }
                    }

                    if (!error_occurred) {
                        // Close the JSON string
                        int remaining_buffer = sizeof(json_buffer) - total_written;
                        if (remaining_buffer <= 0) {
                            printf("Buffer overflow when closing JSON.\n");
                            error_occurred = true;
                        } else {
                            written = snprintf(json_buffer + total_written, remaining_buffer, "]}");
                            if (written < 0 || written >= remaining_buffer) {
                                printf("Error closing JSON string.\n");
                                error_occurred = true;
                            } else {
                                total_written += written;
                            }
                        }
                    }

                    if (!error_occurred) {
                        // Enqueue the JSON string
                        enqueue_json(std::string(json_buffer, total_written));
                    } else {
                        printf("Error occurred during JSON construction, not enqueuing partial data.\n");
                    }

                    // Reset measurement counts and clear data samples
                    reset_measurement_data();
                }

                // Perform other non-blocking tasks here if needed
                tight_loop_contents();  // Yield to other processes
            }
        }
    }


    // Helper function to reset measurement counts and data samples
    void reset_measurement_data() {
        for (size_t i = 0; i < measurement_counts_.size(); ++i) {
            measurement_counts_[i] = 0;
            imu_data_samples_[i].clear();
        }
    }





private:
    std::vector<IMU> imus_;
    std::string device_id_;
    const static uint8_t max_measurements = PACKET_STACK_SIZE;
    std::vector<std::vector<json>> imu_data_samples_; // Stores samples per IMU
    std::vector<uint8_t> measurement_counts_; // Counts per IMU

    void initialize_cs_pins() {
        for (uint pin : SPI0_CS_PINS) {
            gpio_init(pin);
            gpio_set_dir(pin, GPIO_OUT);
            gpio_put(pin, 1); // Set CS high
        }
        for (uint pin : SPI1_CS_PINS) {
            gpio_init(pin);
            gpio_set_dir(pin, GPIO_OUT);
            gpio_put(pin, 1); // Set CS high
        }
    }

    void enqueue_json(const std::string& json_str) {
        // Append newline character to indicate end of message
        std::string json_with_newline = json_str + "\n";

        // Ensure the JSON string does not exceed the maximum length
        if (json_with_newline.length() >= JSON_STRING_MAX_LENGTH) {
            printf("JSON string too long to enqueue.\n");
            return;
        }

        // Allocate memory for the JSON string
        char* json_str_buffer = (char*)malloc(json_with_newline.length() + 1);
        if (json_str_buffer == NULL) {
            printf("Failed to allocate memory for JSON string.\n");
            return;
        }
        strcpy(json_str_buffer, json_with_newline.c_str());

        // Enqueue the pointer to the JSON string
        if (!queue_try_add(&json_queue, &json_str_buffer)) {
            printf("Failed to enqueue JSON string.\n");
            free(json_str_buffer);
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

    // Function to wait for 'Connected' message over UART
    void wait_for_connected_message() {
        const int BUFFER_SIZE = 256;
        char buffer[BUFFER_SIZE];
        int idx = 0;
        printf("Waiting for 'Connected' message over UART...\n");
        while (true) {
            // Read one byte at a time
            if (uart_is_readable(UART_ID)) {
                char ch = uart_getc(UART_ID);
                buffer[idx++] = ch;
                if (ch == '\n' || idx >= BUFFER_SIZE - 1) {
                    printf("Received message: %s\n", buffer);
                    buffer[idx] = '\0'; // Null-terminate
                    if (strcmp(buffer, "Connected\n") == 0 || strcmp(buffer, "Connected") == 0) {
                        printf("Received 'Connected' message over UART.\n");
                        break;
                    } else {
                        // Reset index and buffer if message is not 'Connected'
                        idx = 0;
                        memset(buffer, 0, BUFFER_SIZE);
                    }
                }
            } else {
                sleep_ms(100); // Sleep to prevent tight loop
            }
        }
    }
};

int main() {
    // Initialize stdio
    stdio_init_all();
    // sleep_ms(15000);  // Wait for serial connection
    sleep_ms(3000);  // Wait for serial connection
    printf("Starting RP2040 Controller\n");

    // Create RP2040Controller instance
    RP2040Controller controller;

    printf("RP2040 Controller initialized\n");
    controller.run();

    return 0;
}
