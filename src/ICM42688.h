#ifndef ICM42688
#define ICMD42688
#endif

#include <stdint.h>

namespace ICM42688REG
{
    // User Bank 0 registers
    static constexpr uint8_t REG_BANK_SEL = 0x76;
        // 000: Bank 0 (default)
        // 001: Bank 1
        // 010: Bank 2
        // 011: Bank 3
        // 100: Bank 4

    // Configuration registers
    static constexpr uint8_t DEVICE_CONFIG = 0x11;
    static constexpr uint8_t DRIVE_CONFIG = 0x13;
    static constexpr uint8_t INT_CONFIG = 0x14;
    static constexpr uint8_t FIFO_CONFIG = 0x16;
    // Temperature data registers
    static constexpr uint8_t TEMP_DATA1 = 0x1D;
    static constexpr uint8_t TEMP_DATA0 = 0x1E;
    // Accelerometer and Gyroscope data registers
    static constexpr uint8_t ACCEL_DATA_X1 = 0x1F;
    static constexpr uint8_t ACCEL_DATA_X0 = 0x20;
    static constexpr uint8_t ACCEL_DATA_Y1 = 0x21;
    static constexpr uint8_t ACCEL_DATA_Y0 = 0x22;
    static constexpr uint8_t ACCEL_DATA_Z1 = 0x23;
    static constexpr uint8_t ACCEL_DATA_Z0 = 0x24;
    static constexpr uint8_t GYRO_DATA_X1 = 0x25;
    static constexpr uint8_t GYRO_DATA_X0 = 0x26;
    static constexpr uint8_t GYRO_DATA_Y1 = 0x27;
    static constexpr uint8_t GYRO_DATA_Y0 = 0x28;
    static constexpr uint8_t GYRO_DATA_Z1 = 0x29;
    static constexpr uint8_t GYRO_DATA_Z0 = 0x2A;
    // TMST_FSYNC register
    static constexpr uint8_t TMST_FSYNCH = 0x2B;
    static constexpr uint8_t TMST_FSYNCL = 0x2C;
    static constexpr uint8_t INT_STATUS = 0x2D;
    // FIFO registers
    static constexpr uint8_t FIFO_COUNTH = 0x2E;
    static constexpr uint8_t FIFO_COUNTL = 0x2F;
    static constexpr uint8_t FIFO_DATA = 0x30;
    // Apex registers
    static constexpr uint8_t APEX_DATA0 = 0x31;
    static constexpr uint8_t APEX_DATA1 = 0x32;
    static constexpr uint8_t APEX_DATA2 = 0x33;
    static constexpr uint8_t APEX_DATA3 = 0x34;
    static constexpr uint8_t APEX_DATA4 = 0x35;
    static constexpr uint8_t APEX_DATA5 = 0x36;
    // Interrupt status registers
    static constexpr uint8_t INT_STATUS2 = 0x37;
    static constexpr uint8_t INT_STATUS3 = 0x38;
    static constexpr uint8_t SIGNAL_PATH_RESET = 0x4B;
    static constexpr uint8_t INTF_CONFIG0 = 0x4C;
    static constexpr uint8_t INTF_CONFIG1 = 0x4D;
    static constexpr uint8_t PWR_MGMT0 = 0x4E;
    // Configuration registers - Accel and Gyro
    static constexpr uint8_t GYRO_CONFIG0 = 0x4F;
    static constexpr uint8_t ACCEL_CONFIG0 = 0x50;
    static constexpr uint8_t GYRO_CONFIG1 = 0x51;
    static constexpr uint8_t GYRO_ACCEL_CONFIG0 = 0x52;
    static constexpr uint8_t ACCEL_CONFIG1 = 0x53;
    // Configuration registers - TMST, Apex, SMD, FIFO, FSYNC
    static constexpr uint8_t TMST_CONFIG = 0x54;
    static constexpr uint8_t APEX_CONFIG0 = 0x56;
    static constexpr uint8_t SMD_CONFIG = 0x57;
    static constexpr uint8_t FIFO_CONFIG1 = 0x5F;
    static constexpr uint8_t FIFO_CONFIG2 = 0x60;
    static constexpr uint8_t FIFO_CONFIG3 = 0x61;
    static constexpr uint8_t FSYNC_CONFIG = 0x62;
    // Interrupt configuration registers
    static constexpr uint8_t INT_CONFIG0 = 0x63;
    static constexpr uint8_t INT_CONFIG1 = 0x64;
    static constexpr uint8_t INT_SOURCE0 = 0x65;
    static constexpr uint8_t INT_SOURCE1 = 0x66;
    static constexpr uint8_t INT_SOURCE3 = 0x68;
    static constexpr uint8_t INT_SOURCE4 = 0x69;

    static constexpr uint8_t FIFO_LOST_PKT0 = 0x6C;
    static constexpr uint8_t FIFO_LOST_PKT1 = 0x6D;
    static constexpr uint8_t SELF_TEST_CONFIG = 0x70;
    static constexpr uint8_t WHO_AM_I = 0x75;

    // User Bank 1 registers
    // Sensor configuration registers
    static constexpr uint8_t SENSOR_CONFIG0 = 0x03;
    // Gyro static configuration registers
    static constexpr uint8_t GYRO_CONFIG_STATIC2 = 0x0B;
    static constexpr uint8_t GYRO_CONFIG_STATIC3 = 0x0C;
    static constexpr uint8_t GYRO_CONFIG_STATIC4 = 0x0D;
    static constexpr uint8_t GYRO_CONFIG_STATIC5 = 0x0E;
    static constexpr uint8_t GYRO_CONFIG_STATIC6 = 0x0F;
    static constexpr uint8_t GYRO_CONFIG_STATIC7 = 0x10;
    static constexpr uint8_t GYRO_CONFIG_STATIC8 = 0x11;
    static constexpr uint8_t GYRO_CONFIG_STATIC9 = 0x12;
    static constexpr uint8_t GYRO_CONFIG_STATIC10 = 0x13;
    // X, Y, Z gyro data registers
    static constexpr uint8_t XG_ST_DATA = 0x5F;
    static constexpr uint8_t YG_ST_DATA = 0x60;
    static constexpr uint8_t ZG_ST_DATA = 0x61;
    // Time stamp registers
    static constexpr uint8_t TMSTVAL0 = 0x62;
    static constexpr uint8_t TMSTVAL1 = 0x63;
    static constexpr uint8_t TMSTVAL2 = 0x64;
    // Interrupt configuration registers
    static constexpr uint8_t INTF_CONFIG4 = 0x7A;
    static constexpr uint8_t INTF_CONFIG5 = 0x7B;
    static constexpr uint8_t INTF_CONFIG6 = 0x7C;

    // User Bank 2 registers
    static constexpr uint8_t ACCEL_CONFIG_STATIC2 = 0x03;
    static constexpr uint8_t ACCEL_CONFIG_STATIC3 = 0x04;
    static constexpr uint8_t ACCEL_CONFIG_STATIC4 = 0x05;
    // X, Y, Z accel data registers
    static constexpr uint8_t XA_ST_DATA = 0x3B;
    static constexpr uint8_t YA_ST_DATA = 0x3C;
    static constexpr uint8_t ZA_ST_DATA = 0x3D;

    // User Bank 3 registers
    // Clock divider configuration registers
    static constexpr uint8_t CLKDIV = 0x2A;

    // User Bank 4 registers
    // Apex configuration registers
    static constexpr uint8_t APEX_CONFIG1 = 0x40;
    static constexpr uint8_t APEX_CONFIG2 = 0x41;
    static constexpr uint8_t APEX_CONFIG3 = 0x42;
    static constexpr uint8_t APEX_CONFIG4 = 0x43;
    static constexpr uint8_t APEX_CONFIG5 = 0x44;
    static constexpr uint8_t APEX_CONFIG6 = 0x45;
    static constexpr uint8_t APEX_CONFIG7 = 0x46;
    static constexpr uint8_t APEX_CONFIG8 = 0x47;
    static constexpr uint8_t APEX_CONFIG9 = 0x48;
    // Accel WOM registers
    static constexpr uint8_t ACCEL_WOM_X_THR = 0x4A;
    static constexpr uint8_t ACCEL_WOM_Y_THR = 0x4B;
    static constexpr uint8_t ACCEL_WOM_Z_THR = 0x4C;
    // Interrupt source registers
    static constexpr uint8_t INT_SOURCE6 = 0x4D;
    static constexpr uint8_t INT_SOURCE7 = 0x4E;
    static constexpr uint8_t INT_SOURCE8 = 0x4F;
    static constexpr uint8_t INT_SOURCE9 = 0x50;
    static constexpr uint8_t INT_SOURCE10 = 0x51;
    // Offset registers
    static constexpr uint8_t OFFSET_USER0 = 0x77;
    static constexpr uint8_t OFFSET_USER1 = 0x78;
    static constexpr uint8_t OFFSET_USER2 = 0x79;
    static constexpr uint8_t OFFSET_USER3 = 0x7A;
    static constexpr uint8_t OFFSET_USER4 = 0x7B;
    static constexpr uint8_t OFFSET_USER5 = 0x7C;
    static constexpr uint8_t OFFSET_USER6 = 0x7D;
    static constexpr uint8_t OFFSET_USER7 = 0x7E;
    static constexpr uint8_t OFFSET_USER8 = 0x7F;
}

namespace ICM42688SET {
    // GYRO_FS_SEL DEG/SEC
    static constexpr uint8_t GYRO_FS_SEL_2000   = 0b000;
    static constexpr uint8_t GYRO_FS_SEL_1000   = 0b001;
    static constexpr uint8_t GYRO_FS_SEL_500    = 0b010;
    static constexpr uint8_t GYRO_FS_SEL_250    = 0b011;
    static constexpr uint8_t GYRO_FS_SEL_125    = 0b100;
    static constexpr uint8_t GYRO_FS_SEL_62     = 0b101;
    static constexpr uint8_t GYRO_FS_SEL_31     = 0b110;
    static constexpr uint8_t GYRO_FS_SEL_15     = 0b111;

    // GYRO_ODR Hz
    static constexpr uint8_t GYRO_ODR_32K       = 0b0001;
    static constexpr uint8_t GYRO_ODR_16K       = 0b0010;
    static constexpr uint8_t GYRO_ODR_8K        = 0b0011;
    static constexpr uint8_t GYRO_ODR_4K        = 0b0100;
    static constexpr uint8_t GYRO_ODR_2K        = 0b0101;
    static constexpr uint8_t GYRO_ODR_1K        = 0b0110;
    static constexpr uint8_t GYRO_ODR_200       = 0b0111;
    static constexpr uint8_t GYRO_ODR_100       = 0b1000;
    static constexpr uint8_t GYRO_ODR_50        = 0b1001;
    static constexpr uint8_t GYRO_ODR_25        = 0b1010;
    static constexpr uint8_t GYRO_ODR_12        = 0b1011;
    static constexpr uint8_t GYRO_ODR_500       = 0b1111;

    // ACCEL_FS_SEL g
    static constexpr uint8_t ACCEL_FS_SEL_16    = 0b000;
    static constexpr uint8_t ACCEL_FS_SEL_8     = 0b001;
    static constexpr uint8_t ACCEL_FS_SEL_4     = 0b010;
    static constexpr uint8_t ACCEL_FS_SEL_2     = 0b011;

    // ACCEL_ODR Hz
    static constexpr uint8_t ACCEL_ODR_32K      = 0b0001;
    static constexpr uint8_t ACCEL_ODR_16K      = 0b0010;
    static constexpr uint8_t ACCEL_ODR_8K       = 0b0011;
    static constexpr uint8_t ACCEL_ODR_4K       = 0b0100;
    static constexpr uint8_t ACCEL_ODR_2K       = 0b0101;
    static constexpr uint8_t ACCEL_ODR_1K       = 0b0110;
    static constexpr uint8_t ACCEL_ODR_200      = 0b0111;
    static constexpr uint8_t ACCEL_ODR_100      = 0b1000;
    static constexpr uint8_t ACCEL_ODR_50       = 0b1001;
    static constexpr uint8_t ACCEL_ODR_25       = 0b1010;
    static constexpr uint8_t ACCEL_ODR_12       = 0b1011;
    static constexpr uint8_t ACCEL_ODR_6        = 0b1100;
    static constexpr uint8_t ACCEL_ODR_3        = 0b1101;
    static constexpr uint8_t ACCEL_ODR_1        = 0b1110;
    static constexpr uint8_t ACCEL_ODR_500      = 0b1111;
}