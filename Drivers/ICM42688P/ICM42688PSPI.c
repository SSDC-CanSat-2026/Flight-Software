#include "ICM42688PSPI.h"

#include <stdint.h>

static SPI_HandleTypeDef *hspi;

/* Private GPIO CS Pin Variables */
static GPIO_TypeDef *ChipSelect_GPIO_Port;
static uint16_t ChipSelect_Pin;

#define ACCEL_FS_SEL_0 2048
#define GYRO_FS_SEL_0 16.4

static void ICM42688P_disable_chip_select()
{
    HAL_GPIO_WritePin(ChipSelect_GPIO_Port, ChipSelect_Pin, GPIO_PIN_RESET);
}

static void ICM42688P_enable_chip_select()
{
    HAL_GPIO_WritePin(ChipSelect_GPIO_Port, ChipSelect_Pin, GPIO_PIN_SET);
}

static HAL_StatusTypeDef ICM42688P_write_reg(uint8_t reg, uint8_t data)
{
    uint8_t tx[2] = {reg, data};
    ICM42688P_disable_chip_select();
    HAL_SPI_Transmit(hspi, tx, 2, HAL_MAX_DELAY);
    ICM42688P_enable_chip_select();
    return HAL_OK;
}

int16_t ICM42688P_read_reg(uint8_t reg)
{
    uint8_t tx[3] = { reg | 0x80, 0x00, 0x00 }; // 0x80 = read bit
    uint8_t rx[3] = {0};
    ICM42688P_disable_chip_select();
    HAL_SPI_TransmitReceive(hspi, (char*)&tx, (char*)&rx, 3, HAL_MAX_DELAY);
    ICM42688P_enable_chip_select();

//    unt16_t shifted = rx[1] << 8;
//    unt16_t lower = rx[2];
    int16_t value = (int16_t)((rx[1] << 8) | rx[2]);
    return value;
}

uint8_t ICM42688P_init(SPI_HandleTypeDef *spi_handle, GPIO_TypeDef *chip_select_port, uint16_t chip_select_gpio_pin)
{
    hspi = spi_handle;
    ChipSelect_GPIO_Port = chip_select_port;
    ChipSelect_Pin = chip_select_gpio_pin;

    /* // ???? WHAT??????
    HAL_Delay(100);
    ICM42688P_write_reg(0x4F, 0x04);  // Reset device
    HAL_Delay(100);
    ICM42688P_write_reg(0x11, 0x00);  // Power management
    ICM42688P_write_reg(0x10, 0x0F);  // Gyro and accel config
    */



    ICM42688P_write_reg(0x11, 0x01); // Reset Device
    HAL_Delay(100);

    uint8_t who = ICM42688P_read_reg(0x75);

    ICM42688P_write_reg(0x4E, (0b11 << 2) | (0b11 << 0)); // Enable gyro & accelerometer
    ICM42688P_write_reg(0x50, 0x06); // accel: ±16g, ~1kHz
    ICM42688P_write_reg(0x4F, 0x06); // gyro: ±2000 dps, ~1kHz

    // In order to enable CLKIN you need to change Register Banks
    ICM42688P_write_reg(0x76, (0b001));					  // Change to Bank 1
    ICM42688P_write_reg(0x7B, (0b10 << 1));               // Enable CLKIN
    ICM42688P_write_reg(0x76, (0b000));					  // Change back to Bank 0

    return 0;
}

/*int16_t Get_Accel_P(int16_t gyro_p, TickType_t time)
{
    return (gyro_old_p - gyro_p) / ((xTaskGetTickCount() - old_time) / configTICK_RATE_HZ); // configTICK_RATE_HZ is in FreeRTOSConfig.h
}

int16_t Get_Accel_Y(int16_t gyro_y, TickType_t time)
{
    return (gyro_old_y - gyro_y) / ((xTaskGetTickCount() - old_time) / configTICK_RATE_HZ);
}

int16_t Get_Accel_R(int16_t gyro_r, TickType_t time)
{
    return (gyro_old_r - gyro_r) / ((xTaskGetTickCount() - old_time) / configTICK_RATE_HZ);
}*/

void ICM42688P_read_data(ICM42688P_AccelData *data)
{
	// Traditional "linear" accelerations
    data->accel_x 	= (float)ICM42688P_read_reg(0x1F) / ACCEL_FS_SEL_0;
    data->accel_yaw = (float)ICM42688P_read_reg(0x21) / ACCEL_FS_SEL_0;
    data->accel_z 	= (float)ICM42688P_read_reg(0x23) / ACCEL_FS_SEL_0;

    data->gyro_p = (float)ICM42688P_read_reg(0x25) / GYRO_FS_SEL_0;
    data->gyro_y = (float)ICM42688P_read_reg(0x27) / GYRO_FS_SEL_0;
    data->gyro_r = (float)ICM42688P_read_reg(0x29) / GYRO_FS_SEL_0;

    TickType_t curr_time = xTaskGetTickCount();

    // Calculating acceleration
    data->accel_p 	= (data->gyro_p - data->gyro_old_p) / ((curr_time - data->old_time_tick) / configTICK_RATE_HZ);
    data->accel_y 	= (data->gyro_y - data->gyro_old_y) / ((curr_time - data->old_time_tick) / configTICK_RATE_HZ);
    data->accel_r 	= -((data->gyro_r - data->gyro_old_r) / ((curr_time - data->old_time_tick) / configTICK_RATE_HZ));

    data->gyro_old_p = data->gyro_p;
    data->gyro_old_y = data->gyro_y;
    data->gyro_old_r = data->gyro_r;
    data->old_time_tick = curr_time;
}
