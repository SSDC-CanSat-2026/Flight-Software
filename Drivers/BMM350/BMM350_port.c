#include "BMM350_port.h"
#include <stdint.h>
#include "cmsis_os.h"

// This is for the interface pointer that BOSCH likes to use
typedef struct
{
    I2C_HandleTypeDef *hi2c;
    uint8_t dev_id;
} bmm350_intf_t;

static bmm350_intf_t intf;

void BMM350_delay(uint32_t period, void *intf_ptr)
{
    (void)intf_ptr;

    osDelay((period + 999) / 1000);  // round up to ms
}

//BMM350_INTF_RET_TYPE BMM350_I2C_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len)
//{
//    if (HAL_I2C_Mem_Read(bmm350_I2C_handle, dev_id << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, HAL_MAX_DELAY) != HAL_OK)
//    {
//        return -1;
//    }
//    return 0;
//}

BMM350_INTF_RET_TYPE BMM350_I2C_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr)
{
    bmm350_intf_t *intf = (bmm350_intf_t *)intf_ptr;

    if (HAL_I2C_Mem_Read(intf->hi2c, intf->dev_id << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, data, len, HAL_MAX_DELAY) != HAL_OK)
    {
        return -1;
    }

    return 0;
}

//BMM350_INTF_RET_TYPE BMM350_I2C_write(uint8_t dev_id, uint8_t reg_addr, const uint8_t *data, uint16_t len)
//{
//    if (HAL_I2C_Mem_Write(bmm350_I2C_handle, dev_id << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, (uint8_t*)data, len, HAL_MAX_DELAY) != HAL_OK)
//    {
//        return -1;
//    }
//    return 0;
//}

BMM350_INTF_RET_TYPE BMM350_I2C_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr)
{
    bmm350_intf_t *intf = (bmm350_intf_t *)intf_ptr;

    if (HAL_I2C_Mem_Write(intf->hi2c, intf->dev_id << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, (uint8_t *)data, len, HAL_MAX_DELAY) != HAL_OK)
    {
        return -1;
    }

    return 0;
}

BMM350_INTF_RET_TYPE BMM350_init(struct bmm350_dev* bmm350, I2C_HandleTypeDef* I2C_handle)
{
    if (bmm350 == NULL)
    {
        return BMM350_E_NULL_PTR;
    }

    intf.hi2c = I2C_handle;
    intf.dev_id = BMM350_I2C_ADSEL_SET_LOW;

    bmm350->intf_ptr = &intf;
    bmm350->delay_us = BMM350_delay;
    bmm350->read = BMM350_I2C_read;
    bmm350->write = BMM350_I2C_write;

    int8_t result = bmm350_init(bmm350);

    result = bmm350_set_powermode(BMM350_NORMAL_MODE, bmm350);
    result = bmm350_configure_interrupt(BMM350_PULSED,
                                      BMM350_ACTIVE_HIGH,
                                      BMM350_INTR_PUSH_PULL,
                                      BMM350_UNMAP_FROM_PIN,
                                      bmm350);


    result = bmm350_enable_interrupt(BMM350_ENABLE_INTERRUPT, bmm350);

    result = bmm350_set_odr_performance(BMM350_DATA_RATE_25HZ, BMM350_AVERAGING_8, bmm350);

    result = bmm350_enable_axes(BMM350_X_EN, BMM350_Y_EN, BMM350_Z_EN, bmm350);

    return result;
//    return 0;
}

BMM350_INTF_RET_TYPE BMM350_read_mag_data(struct bmm350_dev* bmm350, struct bmm350_mag_temp_data* mag_data)
{
    if (bmm350 == NULL || mag_data == NULL)
    {
        return BMM350_E_NULL_PTR;
    } 

    return bmm350_get_compensated_mag_xyz_temp_data(mag_data, bmm350);
}

BMM350_INTF_RET_TYPE BMM350_read_mag_data_interrupt(struct bmm350_dev* bmm350, struct bmm350_mag_temp_data* mag_data)
{
    if (bmm350 == NULL || mag_data == NULL)
    {
        return BMM350_E_NULL_PTR;
    }

    uint8_t data_ready_interrupt_status = 0;

    int8_t result = bmm350_get_regs(BMM350_REG_INT_STATUS, &data_ready_interrupt_status, 1, bmm350);

    if (!(data_ready_interrupt_status & BMM350_DRDY_DATA_REG_MSK))
    {
        return 0;
    }

    result = bmm350_get_compensated_mag_xyz_temp_data(mag_data, bmm350);

    return result;
}
