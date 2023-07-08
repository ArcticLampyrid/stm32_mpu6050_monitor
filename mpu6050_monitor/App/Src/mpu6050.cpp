#include "mpu6050.h"
#include <cmath>
#include <cstring>

#define DEVICE_RESET (0x80)

#define WHO_AM_I_REG (0x75)
#define PWR_MGMT_1_REG (0x6B)
#define PWR_MGMT_2_REG (0x6C)
#define SMPLRT_DIV_REG (0x19)
#define ACCEL_CONFIG_REG (0x1C)
#define ACCEL_X_OUT_H_REG (0x3B)
#define GYRO_CONFIG_REG (0x1B)
#define SIGNAL_PATH_RESET (0x68)

#define TEMP_RESET (1)
#define ACCEL_RESET (2)
#define GYRO_RESET (4)

#define MPU6050_UNSHIFT_ADDR (0x68)
#define MPU6050_ADDR (MPU6050_UNSHIFT_ADDR << 1)

inline unsigned long my_micros()
{
    uint32_t m0 = HAL_GetTick();
    __IO uint32_t u0 = SysTick->VAL;
    uint32_t m1 = HAL_GetTick();
    __IO uint32_t u1 = SysTick->VAL;
    const uint32_t tms = SysTick->LOAD + 1;
    if (m1 != m0)
    {
        return (m1 * 1000 + ((tms - u1) * 1000) / tms);
    }
    else
    {
        return (m0 * 1000 + ((tms - u0) * 1000) / tms);
    }
}

mpu6050_t::mpu6050_t(I2C_HandleTypeDef *I2Cx, uint32_t i2c_timeout)
    : I2Cx(I2Cx), i2c_timeout(i2c_timeout), last_updated_at(0)
{
}

bool mpu6050_t::init(accel_full_scale_range accel_fsr, gyro_full_scale_range gyro_fsr, uint8_t sample_rate_div)
{
    switch (accel_fsr)
    {
    case accel_full_scale_range::fsr_2g:
        accel_x_scale = accel_y_scale = accel_z_scale = 16384.0;
        break;
    case accel_full_scale_range::fsr_4g:
        accel_x_scale = accel_y_scale = accel_z_scale = 8192.0;
        break;
    case accel_full_scale_range::fsr_8g:
        accel_x_scale = accel_y_scale = accel_z_scale = 4096.0;
        break;
    case accel_full_scale_range::fsr_16g:
        accel_x_scale = accel_y_scale = accel_z_scale = 2048.0;
        break;
    }
    switch (gyro_fsr)
    {
    case gyro_full_scale_range::fsr_250dps:
        gyro_x_scale = gyro_y_scale = gyro_z_scale = 7509.872412338726;
        break;
    case gyro_full_scale_range::fsr_500dps:
        gyro_x_scale = gyro_y_scale = gyro_z_scale = 3754.936206169363;
        break;
    case gyro_full_scale_range::fsr_1000dps:
        gyro_x_scale = gyro_y_scale = gyro_z_scale = 1877.4681030846814;
        break;
    case gyro_full_scale_range::fsr_2000dps:
        gyro_x_scale = gyro_y_scale = gyro_z_scale = 938.7340515423407;
        break;
    }

    uint8_t data;

    // Reset
    data = DEVICE_RESET;
    if (HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, I2C_MEMADD_SIZE_8BIT, &data, 1, i2c_timeout) != HAL_OK)
    {
        return false;
    }
    HAL_Delay(100);

    data = GYRO_RESET | ACCEL_RESET | TEMP_RESET;
    if (HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, SIGNAL_PATH_RESET, I2C_MEMADD_SIZE_8BIT, &data, 1, i2c_timeout) != HAL_OK)
    {
        return false;
    }
    HAL_Delay(100);

    // Wake up
    data = 1; // PLL with X axis gyroscope reference
    if (HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, I2C_MEMADD_SIZE_8BIT, &data, 1, i2c_timeout) != HAL_OK)
    {
        return false;
    }
    data = 0;
    if (HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_2_REG, I2C_MEMADD_SIZE_8BIT, &data, 1, i2c_timeout) != HAL_OK)
    {
        return false;
    }

    // Check
    if (HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, WHO_AM_I_REG, I2C_MEMADD_SIZE_8BIT, &data, 1, i2c_timeout) != HAL_OK)
    {
        return false;
    }
    if (data != MPU6050_UNSHIFT_ADDR)
    {
        return false;
    }

    // Set DATA RATE of 1KHz by writing SMPLRT_DIV register
    data = sample_rate_div;
    if (HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, SMPLRT_DIV_REG, I2C_MEMADD_SIZE_8BIT, &data, 1, i2c_timeout) != HAL_OK)
    {
        return false;
    }

    // Set accelerometer configuration in ACCEL_CONFIG Register
    // XA_ST=0,YA_ST=0,ZA_ST=0,FS_SEL=0
    data = static_cast<uint8_t>(accel_fsr);
    if (HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ACCEL_CONFIG_REG, I2C_MEMADD_SIZE_8BIT, &data, 1, i2c_timeout) != HAL_OK)
    {
        return false;
    }

    // Set Gyroscopic configuration in GYRO_CONFIG Register
    // XG_ST=0,YG_ST=0,ZG_ST=0,FS_SEL=0
    data = static_cast<uint8_t>(gyro_fsr);
    if (HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG_REG, I2C_MEMADD_SIZE_8BIT, &data, 1, i2c_timeout) != HAL_OK)
    {
        return false;
    }

    return true;
}

mpu6050_raw_data_t mpu6050_t::read_raw_data()
{
    mpu6050_raw_data_t result;
    uint8_t Rec_Data[14];
    if (HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_X_OUT_H_REG, I2C_MEMADD_SIZE_8BIT, Rec_Data, 14, i2c_timeout) !=
        HAL_OK)
    {
        memset(&result, 0, sizeof(result));
        return result;
    }
    result.accel_x_raw = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    result.accel_y_raw = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    result.accel_z_raw = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
    result.temperature_raw = (int16_t)(Rec_Data[6] << 8 | Rec_Data[7]);
    result.gyro_x_raw = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
    result.gyro_y_raw = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
    result.gyro_z_raw = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);
    return result;
}

mpu6050_data_t mpu6050_t::update()
{
    mpu6050_data_t result;
    unsigned long now_at = my_micros();
    double dt = (now_at - last_updated_at) / 1000000.0;
    last_updated_at = now_at;

    mpu6050_raw_data_t raw_data = read_raw_data();
    result.accel_x = raw_data.gyro_x_raw / accel_x_scale;
    result.accel_y = raw_data.gyro_y_raw / accel_y_scale;
    result.accel_z = raw_data.gyro_z_raw / accel_z_scale;
    result.temperature = raw_data.temperature_raw / 340.0 + 36.53;
    result.gyro_x = raw_data.gyro_x_raw / gyro_x_scale;
    result.gyro_y = raw_data.gyro_y_raw / gyro_y_scale;
    result.gyro_z = (static_cast<int32_t>(raw_data.gyro_z_raw) - gyro_z_raw_bias) / gyro_z_scale;

    double roll;
    double roll_sqrt = sqrt(static_cast<int32_t>(raw_data.accel_x_raw) * raw_data.accel_x_raw +
                            static_cast<int32_t>(raw_data.accel_z_raw) * raw_data.accel_z_raw);
    if (roll_sqrt != 0.0)
    {
        roll = atan(raw_data.accel_y_raw / roll_sqrt);
    }
    else
    {
        roll = 0.0;
    }
    double pitch = atan2(-raw_data.accel_x_raw, raw_data.accel_z_raw);

    if (last_updated_at == 0)
    {
        // Init
        kalman_pitch.set_angle(pitch);
        kalman_roll.set_angle(roll);
        result.pitch = pitch;
        result.roll = roll;
        result.yaw = this->yaw_via_integrate = 0;
        return result;
    }

    if ((pitch < -M_PI_2 && kalman_pitch.get_angle() > M_PI_2) ||
        (pitch > M_PI_2 && kalman_pitch.get_angle() < -M_PI_2))
    {
        kalman_pitch.set_angle(pitch);
    }
    else
    {
        pitch = kalman_pitch.update(pitch, result.gyro_y, dt);
    }

    roll = kalman_roll.update(roll, std::abs(pitch) > M_PI_2 ? -result.gyro_x : result.gyro_x, dt);

    this->yaw_via_integrate += result.gyro_z * dt;
    // normalize to [-pi, +pi]
    this->yaw_via_integrate = fmod(this->yaw_via_integrate + M_PI, 2 * M_PI);
    if (this->yaw_via_integrate < 0)
    {
        this->yaw_via_integrate += 2 * M_PI;
    }
    this->yaw_via_integrate -= M_PI;

    result.pitch = pitch;
    result.roll = roll;
    result.yaw = this->yaw_via_integrate;

    return result;
}

int16_t mpu6050_t::calibrate_gyro_z_bias()
{
    uint32_t tick_start = HAL_GetTick();
    constexpr const uint32_t duration = 10;
    constexpr const uint16_t times = 100;
    int32_t new_bias = 0;
    for (uint32_t i = 0; i < times; ++i)
    {
        mpu6050_raw_data_t raw_data = read_raw_data();

        new_bias += raw_data.gyro_z_raw;
#pragma clang diagnostic push
#pragma ide diagnostic ignored "LoopDoesntUseConditionVariableInspection"
        while (HAL_GetTick() - tick_start < duration)
            ;
#pragma clang diagnostic pop
        tick_start += duration;
    }
    this->gyro_z_raw_bias = static_cast<int16_t>(new_bias / times);
    return this->gyro_z_raw_bias;
}