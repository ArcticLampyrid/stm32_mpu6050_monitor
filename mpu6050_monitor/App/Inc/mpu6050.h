#pragma once
#include "i2c.h"
#include "kalman_attitude.h"
#include <cstdint>

typedef struct
{
    int16_t accel_x_raw;
    int16_t accel_y_raw;
    int16_t accel_z_raw;
    int16_t gyro_x_raw;
    int16_t gyro_y_raw;
    int16_t gyro_z_raw;
    int16_t temperature_raw;
} mpu6050_raw_data_t;

typedef struct
{
    /**
     * Unit: g
     */
    double accel_x;
    /**
     * Unit: g
     */
    double accel_y;
    /**
     * Unit: g
     */
    double accel_z;

    /**
     * Unit: degree Celsius
     */
    double temperature;

    /**
     * Unit: rad/s
     */
    double gyro_x;
    /**
     * Unit: rad/s
     */
    double gyro_y;
    /**
     * Unit: rad/s
     */
    double gyro_z;

    /**
     * Unit: rad
     */
    double roll;
    /**
     * Unit: rad
     */
    double pitch;
    /**
     * Unit: rad
     * @note Due to the lack of a magnetometer, the static drift of this value is severe.
     */
    double yaw;
} mpu6050_data_t;

enum class accel_full_scale_range : uint8_t
{
    fsr_2g = 0 << 3,
    fsr_4g = 1 << 3,
    fsr_8g = 2 << 3,
    fsr_16g = 3 << 3,
};

enum class gyro_full_scale_range : uint8_t
{
    fsr_250dps = 0 << 3,
    fsr_500dps = 1 << 3,
    fsr_1000dps = 2 << 3,
    fsr_2000dps = 3 << 3,
};

class mpu6050_t
{
  private:
    I2C_HandleTypeDef *I2Cx;
    uint32_t i2c_timeout;
    kalman_attitude_t kalman_pitch{};
    kalman_attitude_t kalman_roll{};
    unsigned long last_updated_at;
    double yaw_via_integrate{};

  public:
    /**
     * Create an instance of MPU6050 driver.
     * @param I2Cx I2C bus
     * @param i2c_timeout timeout of I2C bus
     * @note The timeout is determined via HAL_GetTick. Remember to check the priority of SysTick interrupt.
     */
    explicit mpu6050_t(I2C_HandleTypeDef *I2Cx, uint32_t i2c_timeout = 100);
    /**
     * The scale of accel_x_raw, usually set by the function `init` automatically.
     */
    double accel_x_scale{};
    /**
     * The scale of accel_y_raw, usually set by the function `init` automatically.
     */
    double accel_y_scale{};
    /**
     * The scale of accel_z_raw, usually set by the function `init` automatically.
     */
    double accel_z_scale{};
    /**
     * The scale of gyro_x_raw, usually set by the function `init` automatically.
     */
    double gyro_x_scale{};
    /**
     * The scale of gyro_y_raw, usually set by the function `init` automatically.
     */
    double gyro_y_scale{};
    /**
     * The scale of gyro_z_raw, usually set by the function `init` automatically.
     */
    double gyro_z_scale{};
    /**
     * The bias of gyro_z_raw, usually set by the function `calibrate_gyro_z_bias` automatically.
     */
    int16_t gyro_z_raw_bias{};
    /**
     * Initialize the mpu6050
     * @param accel_fsr the full scale range for accel
     * @param gyro_fsr the full scale range for gyro
     * @param sample_rate_div the divider for sample rate
     * @return succeed or not
     */
    bool init(accel_full_scale_range accel_fsr = accel_full_scale_range::fsr_2g,
              gyro_full_scale_range gyro_fsr = gyro_full_scale_range::fsr_250dps, uint8_t sample_rate_div = 0);
    /**
     * @note Keep stable in z-axis when calibrating
     * @return The calculated bias
     */
    int16_t calibrate_gyro_z_bias();
    /**
     * Read the raw data of MPU6050.
     * @return The raw data
     */
    mpu6050_raw_data_t read_raw_data();
    /**
     * Update the attitude. This should be called at intervals.
     * @return The new attitude
     */
    mpu6050_data_t update();
};