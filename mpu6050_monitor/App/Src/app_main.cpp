#include "../Inc/app_main.h"
#include "i2c.h"
#include "mpu6050.h"
#include "oled.h"
#include "uexecuter.h"
#include "uexecuter_service.h"
#include "uexecuter_transport_stm32.h"
#include <cstdio>
#include <cstring>

static uexecuter_transport_stm32_t uexecuter_transport;
static uexecuter_t uexecuter;
static mpu6050_t mpu6050(&hi2c1);
static volatile mpu6050_data_t mpu6050_data;

static double get_pitch()
{
    return mpu6050_data.pitch;
}
static double get_roll()
{
    return mpu6050_data.roll;
}
static double get_yaw()
{
    return mpu6050_data.yaw;
}
static double calibrate_gyro_z_bias()
{
    return mpu6050.calibrate_gyro_z_bias();
}
static int16_t get_gyro_z_raw_bias()
{
    return mpu6050.gyro_z_raw_bias;
}

UEXECUTER_DEFINE_SERVICE(my_uexecuter_service){
    UEXECUTER_FUNCTION_PROTOTYPE_AUTO(get_pitch, "get_pitch"),
    UEXECUTER_FUNCTION_PROTOTYPE_AUTO(get_roll, "get_roll"),
    UEXECUTER_FUNCTION_PROTOTYPE_AUTO(get_yaw, "get_yaw"),
    UEXECUTER_FUNCTION_PROTOTYPE_AUTO(calibrate_gyro_z_bias, "calibrate_gyro_z_bias"),
    UEXECUTER_FUNCTION_PROTOTYPE_AUTO(get_gyro_z_raw_bias, "get_gyro_z_raw_bias"),
};

extern "C" void app_pre_init()
{
    // do nothing
}
extern "C" void app_init()
{
    // do nothing
}
extern "C" void app_sys_init()
{
    // do nothing
}

extern "C" void app_main()
{
    uexecuter_transport_stm32_init(&uexecuter_transport, USART1);
    uexecuter_init(&uexecuter, my_uexecuter_service, UEXECUTER_SERVICE_N_FUNC(my_uexecuter_service));
    uexecuter_bind(&uexecuter, &uexecuter_transport.base);
    uexecuter_transport_stm32_begin(&uexecuter_transport);
    oled_init();
    u8g2_ClearBuffer(&u8g2);
    u8g2_ClearDisplay(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_6x13_mf);
    bool mpu6050_ok = mpu6050.init();
    if (mpu6050_ok)
    {
        HAL_Delay(100);
        mpu6050.calibrate_gyro_z_bias();
    }
    uint32_t screen_last_updated = 0;
    for (;;)
    {
        mpu6050_data_t data = mpu6050.update();
        memcpy((void *)&mpu6050_data, &data, sizeof(mpu6050_data_t));
        if (screen_last_updated - HAL_GetTick() > 50)
        {
            u8g2_ClearBuffer(&u8g2);
            if (mpu6050_ok)
            {
                char str_buf[32];
                sprintf(str_buf, "Pitch: %.6f", data.pitch);
                u8g2_DrawStr(&u8g2, 0, 16, str_buf);
                sprintf(str_buf, "Roll: %.6f", data.roll);
                u8g2_DrawStr(&u8g2, 0, 32, str_buf);
                sprintf(str_buf, "Yaw: %.6f", data.yaw);
                u8g2_DrawStr(&u8g2, 0, 48, str_buf);
                sprintf(str_buf, "Temp: %.6f", data.temperature);
                u8g2_DrawStr(&u8g2, 0, 64, str_buf);
            }
            else
            {
                u8g2_DrawStr(&u8g2, 50, 16, "ERROR");
            }
            u8g2_SendBuffer(&u8g2);
            screen_last_updated = HAL_GetTick();
        }
        if (HAL_GetTick() % 200 < 100)
        {
            HAL_GPIO_WritePin(BoardLED_GPIO_Port, BoardLED_Pin, GPIO_PIN_RESET);
        }
        else
        {
            HAL_GPIO_WritePin(BoardLED_GPIO_Port, BoardLED_Pin, GPIO_PIN_SET);
        }
    }
}

extern "C" void usart1_irq()
{
    uexecuter_transport_stm32_handle_isr(&uexecuter_transport);
}