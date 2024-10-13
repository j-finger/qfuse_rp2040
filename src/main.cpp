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
#include "hardware/flash.h"

// ICM42688 namespaces
#include "ICM42688.h" // ICM-42688-P registers and parameters
using namespace ICM42688REG;
using namespace ICM42688SET;

// JSON library
#include "json.hpp" 
using json = nlohmann::json;

/* ----- General Macros ----- */

#define IMU_COUNT 1 // Number of IMUs connected
#define DEVICE_ID_LENGTH 16

/* ----- UART Macros ----- */

#define UART_ID uart1
#define BAUD_RATE 115200
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY    UART_PARITY_NONE
#define UART_TX_PIN 8
#define UART_RX_PIN 9

/* ----- SPI Macros ----- */

#define SPI_SCLK_PIN 18
#define SPI_MOSI_PIN 19
#define SPI_MISO_PIN 20

#define SPI_PORT spi0
#define SPI_BAUD_RATE 1000 * 1000 // 1 MHz

const uint CS_PINS[IMU_COUNT] = {21};


/* ----- Sensor Settings Structures ----- */

// Universal struct for settings
struct SensorSetting {
    uint8_t value;
    std::string name;
};

// Boolean setting struct
struct BoolSetting {
    bool enabled;
    std::string name;
};

// Anti-Aliasing Filter Setting struct
struct AAFSetting {
    BoolSetting enabled;  // AAF Enable/Disable
    uint8_t delt;
    uint16_t deltsqr;
    uint8_t bitshift;
};

// Power Modes for Accelerometer
static const SensorSetting AccelPowerModes[] = {
    {0x00, "Accelerometer Off (default)"},  // 00: Turns accelerometer off (default)
    {0x02, "Low Power (LP) Mode"},          // 10: Places accelerometer in Low Power (LP) Mode
    {0x03, "Low Noise (LN) Mode"}           // 11: Places accelerometer in Low Noise (LN) Mode
};

// Power Modes for Gyroscope
static const SensorSetting GyroPowerModes[] = {
    {0x00, "Gyroscope Off (default)"},      // 00: Turns gyroscope off (default)
    {0x01, "Standby Mode"},                 // 01: Places gyroscope in Standby Mode
    {0x03, "Low Noise (LN) Mode"}           // 11: Places gyroscope in Low Noise (LN) Mode
};


// Clock Sources
static const SensorSetting ClockSources[] = {
    {0x00, "Internal"},
    {0x01, "Auto"},
    {0x02, "Reserved"},
    {0x03, "External"}
};

// Accelerometer ODR settings
static const SensorSetting AccelODRSettings[] = {
    {ACCEL_ODR_32K, "32kHz"}, //0 LN
    {ACCEL_ODR_16K, "16kHz"}, //1 LN
    {ACCEL_ODR_8K, "8kHz"}, //2 LN
    {ACCEL_ODR_4K, "4kHz"}, //3 LN
    {ACCEL_ODR_2K, "2kHz"}, //4 LN
    {ACCEL_ODR_1K, "1kHz"}, //5  LN
    {ACCEL_ODR_200, "200Hz"}, //6 LN or LP
    {ACCEL_ODR_100, "100Hz"}, //7 LN or LP
    {ACCEL_ODR_50, "50Hz"}, //8 LN or LP
    {ACCEL_ODR_25, "25Hz"}, //9 LN or LP
    {ACCEL_ODR_12, "12.5Hz"}, //10 LN or LP
    {ACCEL_ODR_6, "6.25Hz"}, //11 LP
    {ACCEL_ODR_3, "3.125Hz"}, //12 LP
    {ACCEL_ODR_1, "1.5625Hz"}, //13 LP
    {ACCEL_ODR_500, "500Hz"} //14 LN or LP
};

// Accelerometer FSR settings
static const SensorSetting AccelFSRSettings[] = {
    {ACCEL_FS_SEL_16, "±16g"},
    {ACCEL_FS_SEL_8, "±8g"},
    {ACCEL_FS_SEL_4, "±4g"},
    {ACCEL_FS_SEL_2, "±2g"}
};

// Gyroscope ODR settings
static const SensorSetting GyroODRSettings[] = {
    {GYRO_ODR_32K, "32kHz"}, //0
    {GYRO_ODR_16K, "16kHz"}, //1
    {GYRO_ODR_8K, "8kHz"},  //2
    {GYRO_ODR_4K, "4kHz"}, //3
    {GYRO_ODR_2K, "2kHz"}, //4
    {GYRO_ODR_1K, "1kHz"}, //5
    {GYRO_ODR_200, "200Hz"}, //6
    {GYRO_ODR_100, "100Hz"}, //7
    {GYRO_ODR_50, "50Hz"}, //8
    {GYRO_ODR_25, "25Hz"}, //9
    {GYRO_ODR_12, "12.5Hz"}, //10
    {GYRO_ODR_500, "500Hz"} //11
};

// Gyroscope FSR settings
static const SensorSetting GyroFSRSettings[] = {
    {GYRO_FS_SEL_2000, "±2000dps"},
    {GYRO_FS_SEL_1000, "±1000dps"},
    {GYRO_FS_SEL_500, "±500dps"},
    {GYRO_FS_SEL_250, "±250dps"},
    {GYRO_FS_SEL_125, "±125dps"},
    {GYRO_FS_SEL_62, "±62.5dps"},
    {GYRO_FS_SEL_31, "±31.25dps"},
    {GYRO_FS_SEL_15, "±15.625dps"}
};


// Mapping of FSR to sensitivity for accelerometer and gyroscope
std::map<uint8_t, float> accel_sensitivity_map = {
    {ACCEL_FS_SEL_2, 16384.0f},
    {ACCEL_FS_SEL_4, 8192.0f},
    {ACCEL_FS_SEL_8, 4096.0f},
    {ACCEL_FS_SEL_16, 2048.0f}
};

std::map<uint8_t, float> gyro_sensitivity_map = {
    {GYRO_FS_SEL_2000, 16.4f},
    {GYRO_FS_SEL_1000, 32.8f},
    {GYRO_FS_SEL_500, 65.5f},
    {GYRO_FS_SEL_250, 131.0f},
    {GYRO_FS_SEL_125, 262.0f},
    {GYRO_FS_SEL_62, 524.3f},
    {GYRO_FS_SEL_31, 1048.6f},
    {GYRO_FS_SEL_15, 2097.2f}
};

/* ----- Output Data Structures ----- */
// Data structures for sensor data
struct AccelerometerData {
    float x;
    float y;
    float z;
};

struct GyroscopeData {
    float x;
    float y;
    float z;
};

struct TemperatureData {
    float celcius;
};

struct SensorData {
    AccelerometerData accel;
    GyroscopeData gyro;
    TemperatureData temp;
    uint32_t imu_timestamp; // IMU timestamp (24-bit)
};



// IMU Class
class IMU {
public:
    // Constructor
    IMU(spi_inst_t* spi_port, uint cs_pin, uint8_t subdevice_id)
        : spi_port_(spi_port), cs_pin_(cs_pin), subdevice_id_(subdevice_id) {
        initialize();
    }

    // Initialize the IMU
    void initialize() {
        setup_spi();
        configure_sensor();
        uint8_t who_am_i = read_register(WHO_AM_I);
        if (who_am_i != 0x47) {
            printf("IMU %d not detected! WHO_AM_I: 0x%02X\n", subdevice_id_, who_am_i);
        } else {
            printf("IMU %d detected successfully. WHO_AM_I: 0x%02X\n", subdevice_id_, who_am_i);
        }
    }

    // Read sensor data
    SensorData read_sensor_data() {
        printf("Reading sensor data from IMU %d\n", subdevice_id_);
        SensorData data;
        data.accel = read_accel();
        data.gyro = read_gyro();
        data.temp = read_temp();
        data.imu_timestamp = read_imu_timestamp();
        return data;
    }

    // Generate JSON data
    json jsonify_data(const SensorData& data_in, const std::string& device_id) {
        json sensor_data_json;
        sensor_data_json["subdevice"] = subdevice_id_;
        sensor_data_json["accelerometer"]["x"] = data_in.accel.x;
        sensor_data_json["accelerometer"]["y"] = data_in.accel.y;
        sensor_data_json["accelerometer"]["z"] = data_in.accel.z;
        sensor_data_json["gyroscope"]["x"] = data_in.gyro.x;
        sensor_data_json["gyroscope"]["y"] = data_in.gyro.y;
        sensor_data_json["gyroscope"]["z"] = data_in.gyro.z;
        sensor_data_json["temperature"] = data_in.temp.celcius;
        char hex_timestamp[9]; // 8 characters for hex + 1 for null terminator
        sprintf(hex_timestamp, "%08x", data_in.imu_timestamp);
        sensor_data_json["timestamp"] = hex_timestamp;
        return sensor_data_json;
    }

    // Generate JSON settings
    json jsonify_settings(const std::string& device_id) {
        json settings_data_json;
        settings_data_json["device"] = device_id;
        settings_data_json["subdevice"] = subdevice_id_;

        settings_data_json["settings"]["accel_odr"] = accel_odr_setting_.name;
        settings_data_json["settings"]["accel_fsr"] = accel_fsr_setting_.name;
        settings_data_json["settings"]["accel_sensitivity"] = accel_sensitivity_;

        settings_data_json["settings"]["gyro_odr"] = gyro_odr_setting_.name;
        settings_data_json["settings"]["gyro_fsr"] = gyro_fsr_setting_.name;
        settings_data_json["settings"]["gyro_sensitivity"] = gyro_sensitivity_;


        settings_data_json["settings"]["power"]["temp_disabled"] = temp_disabled_;
        settings_data_json["settings"]["power"]["accel_mode"] = accel_power_mode_.name;
        settings_data_json["settings"]["power"]["gyro_mode"] = gyro_power_mode_.name;

        settings_data_json["settings"]["clock"]["source"] = clock_source_setting_.name;
        settings_data_json["settings"]["clock"]["rtc_mode_enabled"] = rtc_mode_enabled_;

        settings_data_json["settings"]["timestamp_enabled"] = timestamp_enabled_;

        return settings_data_json;
    }

 

    // Read IMU timestamp
    uint32_t read_imu_timestamp() {


        uint8_t signal_path_reset_val = 0b01100100;
        write_register(SIGNAL_PATH_RESET, signal_path_reset_val);

        // Switch to User Bank 1
        select_register_bank(1);
        // write_register(REG_BANK_SEL, 0b00000001); // Select User Bank 1

        uint8_t tmstval0 = read_register(TMSTVAL0);
        uint8_t tmstval1 = read_register(TMSTVAL1);
        uint8_t tmstval2 = read_register(TMSTVAL2);

        // print the values for debugging
        printf("TMSTVAL0: 0x%02X\n", tmstval0);
        printf("TMSTVAL1: 0x%02X\n", tmstval1);
        printf("TMSTVAL2: 0x%02X\n", tmstval2);
        
        // Return to User Bank 0
        select_register_bank(0);
        // write_register(REG_BANK_SEL, 0b00000000); // Reset - Select User Bank 0

        uint32_t imu_timestamp = ((uint32_t)tmstval2 << 16) | ((uint32_t)tmstval1 << 8) | (uint32_t)tmstval0;
        // return imu_timestamp & 0xFFFFFF; // 24-bit timestamp
        return imu_timestamp; // 24-bit timestamp
    }


private:
    // SPI variables
    spi_inst_t* spi_port_;
    uint cs_pin_;
    uint8_t subdevice_id_;

    // Settings
    SensorSetting accel_odr_setting_;
    SensorSetting accel_fsr_setting_;
    SensorSetting gyro_odr_setting_;
    SensorSetting gyro_fsr_setting_;
    uint defaultFSR_ = 2; // ±4g accel, ±500 dps gyro
    uint defaultODR_ = 10; // 12.5 Hz

    // Sensitivity values
    float accel_sensitivity_;
    float gyro_sensitivity_;


    // Power modes
    SensorSetting accel_power_mode_;
    SensorSetting gyro_power_mode_;
    bool temp_disabled_;

    // Clock settings
    SensorSetting clock_source_setting_;
    bool rtc_mode_enabled_;

    // Timestamp enabled
    bool timestamp_enabled_;

    // Filter settings
    AAFSetting accel_aaf_setting_;
    AAFSetting gyro_aaf_setting_;



    /* ----- Register Functions ----- */

    // SPI communication methods
    void cs_select() {
        asm volatile("nop \n nop \n nop");
        gpio_put(cs_pin_, 0); // Active Low
        asm volatile("nop \n nop \n nop");
        // sleep_us(1);
    }

    void cs_deselect() {
        asm volatile("nop \n nop \n nop");
        gpio_put(cs_pin_, 1); // Active Low
        asm volatile("nop \n nop \n nop");
        // sleep_us(1);
    }

    void write_register(uint8_t reg_addr, uint8_t data) {
        uint8_t tx_buf[2];
        tx_buf[0] = reg_addr & 0x7F; // Clear MSB for write operation
        tx_buf[1] = data;

        cs_select();
        spi_write_blocking(spi_port_, tx_buf, 2);
        cs_deselect();
        sleep_ms(10);
    }

    uint8_t read_register(uint8_t reg_addr) {
        uint8_t tx_buf[2];
        uint8_t rx_buf[2];

        tx_buf[0] = reg_addr | 0x80; // Set MSB for read operation
        tx_buf[1] = 0x00;            // Dummy byte

        cs_select();
        spi_write_read_blocking(spi_port_, tx_buf, rx_buf, 2);
        cs_deselect();

        return rx_buf[1]; // The second byte contains the data
    }

    void read_registers(uint8_t reg_addr, uint8_t* data, size_t length) {
        uint8_t tx_buf[length + 1];
        uint8_t rx_buf[length + 1];

        tx_buf[0] = reg_addr | 0x80; // Set MSB for read operation
        memset(&tx_buf[1], 0x00, length); // Dummy bytes

        cs_select();
        spi_write_read_blocking(spi_port_, tx_buf, rx_buf, length + 1);
        cs_deselect();

        memcpy(data, &rx_buf[1], length);
    }

    // Select register bank
    void select_register_bank(uint8_t bank) {
        // write_register(REG_BANK_SEL, bank & 0x07);
        write_register(REG_BANK_SEL, bank);
    }

    // Print register value in hex and binary
    void print_register(uint8_t reg_addr) {
        uint8_t reg_value = read_register(reg_addr);
        printf("\nRegister 0x%02X: 0x%02X\n", reg_addr, reg_value); // print in hex
        printf("Register 0x%02X: 0b\n", reg_addr); // print in binary
    }

    /* ----- SPI setup ----- */
    void setup_spi() {
        // CS pin setup
        gpio_init(cs_pin_);
        gpio_set_dir(cs_pin_, GPIO_OUT);
        gpio_put(cs_pin_, 1); // Deselect the device
    }


    /* ----- Device Setup ----- */
    void reset_device_configuration(){
        write_register(DEVICE_CONFIG, 0x01);
        sleep_ms(100);
    }

    // Sensor configuration
    void configure_sensor() {
        reset_device_configuration();
        
        set_power_modes(GyroPowerModes[2], AccelPowerModes[2], false); // Accel & Gyro Low Noise, Temp Enabled
        // set_clock();

        // Set custom accelerometer settings
        set_accel_odr(7); // 100Hz
        set_accel_fsr(2); // ±4g

        // Set custom gyroscope settings
        set_gyro_odr(7); // 100Hz
        set_gyro_fsr(2); // ±500dps


        // Calculate sensitivity
        calculate_sensitivity();
    }


    /* ----- Data Read ----- */
    // Read accelerometer data
    AccelerometerData read_accel() {
        uint8_t accel_raw_data[6];
        int16_t accel_x_raw, accel_y_raw, accel_z_raw;
        float accel_x_g, accel_y_g, accel_z_g;

        read_registers(ACCEL_DATA_X1, accel_raw_data, 6);

        accel_x_raw = (int16_t)((accel_raw_data[0] << 8) | accel_raw_data[1]);
        accel_y_raw = (int16_t)((accel_raw_data[2] << 8) | accel_raw_data[3]);
        accel_z_raw = (int16_t)((accel_raw_data[4] << 8) | accel_raw_data[5]);

        accel_x_g = (float)accel_x_raw / accel_sensitivity_;
        accel_y_g = (float)accel_y_raw / accel_sensitivity_;
        accel_z_g = (float)accel_z_raw / accel_sensitivity_;

        return {accel_x_g, accel_y_g, accel_z_g};
    }

    // Read gyroscope data
    GyroscopeData read_gyro() {
        uint8_t gyro_raw_data[6];
        int16_t gyro_x_raw, gyro_y_raw, gyro_z_raw;
        float gyro_x_dps, gyro_y_dps, gyro_z_dps;

        read_registers(GYRO_DATA_X1, gyro_raw_data, 6);

        gyro_x_raw = (int16_t)((gyro_raw_data[0] << 8) | gyro_raw_data[1]);
        gyro_y_raw = (int16_t)((gyro_raw_data[2] << 8) | gyro_raw_data[3]);
        gyro_z_raw = (int16_t)((gyro_raw_data[4] << 8) | gyro_raw_data[5]);

        gyro_x_dps = (float)gyro_x_raw / gyro_sensitivity_;
        gyro_y_dps = (float)gyro_y_raw / gyro_sensitivity_;
        gyro_z_dps = (float)gyro_z_raw / gyro_sensitivity_;

        return {gyro_x_dps, gyro_y_dps, gyro_z_dps};
    }

    // Read temperature data
    TemperatureData read_temp() {
        uint8_t temp_raw_data[2];
        int16_t temp_data_raw;
        float temperature_c;

        read_registers(TEMP_DATA1, temp_raw_data, 2);

        temp_data_raw = (int16_t)((temp_raw_data[0] << 8) | temp_raw_data[1]);
        temperature_c = ((float)temp_data_raw / 132.48f) + 25.0f;

        return {temperature_c};
    }


    // Set accelerometer ODR
    void set_accel_odr(const uint odr) {
        
        // Check if the ODR setting is valid
        if (odr >= sizeof(AccelODRSettings) / sizeof(AccelODRSettings[0])) {
            accel_odr_setting_ = AccelODRSettings[defaultODR_];
            printf("Invalid ODR setting. Defaulting to %s\n", accel_odr_setting_.name.c_str());
        } else {
            accel_odr_setting_ = AccelODRSettings[odr];
        }
        
        // Write the new ODR setting to the register
        uint8_t reg_value = ((uint8_t)(accel_fsr_setting_.value) << 5) | ((uint8_t)(accel_odr_setting_.value) & 0x0F);
        write_register(ACCEL_CONFIG0, reg_value);
        calculate_sensitivity();
    }

    // Set accelerometer FSR
    void set_accel_fsr(const uint fsr) {
        
        // Check if the FSR setting is valid
        if (fsr >= sizeof(AccelFSRSettings) / sizeof(AccelFSRSettings[0])) {
            accel_fsr_setting_ = AccelFSRSettings[defaultFSR_];
            printf("Invalid FSR setting. Defaulting to %s\n", accel_fsr_setting_.name.c_str());
        } else {
            accel_fsr_setting_ = AccelFSRSettings[fsr];
        }
        
        // Write the new FSR setting to the register
        uint8_t reg_value = ((uint8_t)(accel_fsr_setting_.value) << 5) | ((uint8_t)(accel_odr_setting_.value) & 0x0F);
        write_register(ACCEL_CONFIG0, reg_value);
        calculate_sensitivity();
    }

    // Set gyroscope ODR
    void set_gyro_odr(const uint odr) {
        
        // Check if the ODR setting is valid
        if (odr >= sizeof(GyroODRSettings) / sizeof(GyroODRSettings[0])) {
            gyro_odr_setting_ = GyroODRSettings[defaultODR_];
            printf("Invalid ODR setting. Defaulting to %s\n", gyro_odr_setting_.name.c_str());
        } else {
            gyro_odr_setting_ = GyroODRSettings[odr];
        }

        // Write the new ODR setting to the register
        uint8_t reg_value = ((uint8_t)(gyro_fsr_setting_.value) << 5) | ((uint8_t)(gyro_odr_setting_.value) & 0x0F);
        write_register(GYRO_CONFIG0, reg_value);
        calculate_sensitivity();
    }


    // Set gyroscope FSR
    void set_gyro_fsr(const uint fsr) {

        // Check if the FSR setting is valid
        if (fsr >= sizeof(GyroFSRSettings) / sizeof(GyroFSRSettings[0])) {
            gyro_fsr_setting_ = GyroFSRSettings[defaultFSR_];
            printf("Invalid FSR setting. Defaulting to %s\n", gyro_fsr_setting_.name.c_str());
        } else {
            gyro_fsr_setting_ = GyroFSRSettings[fsr];
        }

        // Write the new FSR setting to the register
        uint8_t reg_value = ((uint8_t)(gyro_fsr_setting_.value) << 5) | ((uint8_t)(gyro_odr_setting_.value) & 0x0F);
        write_register(GYRO_CONFIG0, reg_value);
        calculate_sensitivity();
    }

    // Calculate sensitivity based on FSR
    void calculate_sensitivity() {
        // Accelerometer sensitivity (LSB/g)
        if (accel_sensitivity_map.find(accel_fsr_setting_.value) != accel_sensitivity_map.end()) {
            accel_sensitivity_ = accel_sensitivity_map[accel_fsr_setting_.value];
        } else {
            accel_sensitivity_ = accel_sensitivity_map[ACCEL_FS_SEL_4]; // Default to ±4g
        }

        // Gyroscope sensitivity (LSB/dps)
        if (gyro_sensitivity_map.find(gyro_fsr_setting_.value) != gyro_sensitivity_map.end()) {
            gyro_sensitivity_ = gyro_sensitivity_map[gyro_fsr_setting_.value];
        } else {
            gyro_sensitivity_ = gyro_sensitivity_map[GYRO_FS_SEL_500]; // Default to ±500dps
        }
    }


    /* ----- Filters ----- */


    /* ----- Power ----- */
    // Set power modes
    void set_power_modes(const SensorSetting& accel_mode, const SensorSetting& gyro_mode, bool temp_disabled) {
        temp_disabled_ = temp_disabled;

        uint8_t pwr_mgmt0 = 0;
        pwr_mgmt0 |= (temp_disabled_ ? 0x20 : 0x00); // TEMP_DIS bit (bit 5)
        pwr_mgmt0 |= ((gyro_mode.value & 0x03) << 2); // GYRO_MODE bits (bits 3:2)
        pwr_mgmt0 |= (accel_mode.value & 0x03);       // ACCEL_MODE bits (bits 1:0)
        write_register(PWR_MGMT0, pwr_mgmt0);

        // Store settings
        accel_power_mode_ = accel_mode;
        gyro_power_mode_ = gyro_mode;
    }

    /* ----- Clock ----- */
    void set_clock() {

        // Configure TMST_CONFIG to enable timestamps, FSYNC, and delta timestamps
        uint8_t tmst_config_val = 0b00011011;  // Enable TMST_EN | TMST_FSYNC_EN | TMST_RES, without TMST_DELTA_EN
        write_register(TMST_CONFIG, tmst_config_val);
        
        // Set INTF_CONFIG1 to use external RTC clock and set proper RTC mode
        uint8_t intf_config1_val = 0b00000101;  // CLKSEL = 01 (external clock) | RTC_MODE enabled
        write_register(INTF_CONFIG1, intf_config1_val);

        // Configure INT2 to be push-pull, active low
        uint8_t int_config_val = 0b00110000;  // INT2_DRIVE_CIRCUIT = push-pull | INT2_POLARITY = active low
        write_register(INT_CONFIG, int_config_val);

        // Latch the timestamp once at the beginning, trigger TMST_STROBE
        uint8_t signal_path_reset_val = 0b01100100;  // TMST_STROBE = 1 | DMP_MEM_RESET_EN | DMP_INIT_EN
        write_register(SIGNAL_PATH_RESET, signal_path_reset_val);

        // Configure Pin 9 as CLKIN for external clock
        select_register_bank(1);
        uint8_t intf_config5_val = 0b00000100;  // PIN9_FUNCTION = CLKIN
        write_register(INTF_CONFIG5, intf_config5_val);
        select_register_bank(0);
    }

};


// RP2040 Controller Class
class RP2040Controller {
public:
    RP2040Controller() {

        // Get RP2040 device ID
        read_device_id();

        // Initialize SPI
        spi_init(SPI_PORT, SPI_BAUD_RATE);
        gpio_set_function(SPI_SCLK_PIN, GPIO_FUNC_SPI);
        gpio_set_function(SPI_MOSI_PIN, GPIO_FUNC_SPI);
        gpio_set_function(SPI_MISO_PIN, GPIO_FUNC_SPI);

        // Initialize IMUs
        for (int i = 0; i < IMU_COUNT; i++) {
            IMU imu(SPI_PORT, CS_PINS[i], i);
            imus_.push_back(imu);
        }

        // Send settings JSON for each IMU
        for (auto& imu : imus_) {
            json settings_json = imu.jsonify_settings(device_id_);
            // send_json(settings_json);
            printf("Data:\n%s\n\n", settings_json.dump(4).c_str());
        }


        // TODO: Send settings JSON over UART
        
    }

    void run() {
        printf("Running RP2040 Controller\n");
        while (true) {
            // Read sensor data from each IMU
            for (auto& imu : imus_) {
                SensorData data = imu.read_sensor_data();
                // uint64_t real_time = imu.get_real_time(data.imu_timestamp);
                json data_json = imu.jsonify_data(data, device_id_);
                // send_json(data_json);
                printf("Data:\n%s\n\n", data_json.dump(4).c_str());
                // TODO: Send data JSON over UART
            }
            sleep_ms(1000); // Adjust as needed
        }
    }


private:
    std::vector<IMU> imus_;
    std::string device_id_;
    std::string session_id_;
    uint64_t reference_time_;
    uint32_t reference_imu_timestamp_;


    // Read device ID from flash memory
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

/* ----- UART ----- */

// TODO: Turn these into member functions of RP2040Controller

// Function to initialize UART
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

// Function to receive data over UART
void uart_receive(char* data) {
    int idx = 0;
    while (uart_is_readable(UART_ID)) {
        char ch = uart_getc(UART_ID);
        data[idx++] = ch;
        if (ch == '}') { // Assuming JSON ends with '}'
            break;
        }
        if (idx >= 255) { // Prevent buffer overflow
            break;
        }
    }
    data[idx] = '\0'; // Null-terminate the string
}

// Main Function
int main() {
    // Initialize stdio
    stdio_init_all();
    sleep_ms(5000); // Wait for serial connection
    printf("Starting RP2040 Controller\n");

    
    printf("Initializing UART\n");
    uart_initialize();
    char data[256];

    // Create RP2040Controller instance
    RP2040Controller controller;

    printf("RP2040 Controller initialized\n");
    controller.run();
    
    return 0;
}
