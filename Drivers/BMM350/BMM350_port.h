#ifndef _BMM350_PORT_
#define _BMM350_PORT_

#include "bmm350.h"
#include "stm32g4xx_hal.h"

/*
*  @details This function initializes the bmm350 struct with the necessary settings, mostly all normal and default mode, and calls the BMM350 API init function from the manufacturer
*  @param[in,out] bmm350 : Pointer to struct bmm350_dev that contains state for sensor
*  @param[in] I2C_handle: Pointer to the HAL I2C handle for the sensor. While using the sensor, the handle address must remain valid.
*  @return Result of initialization status execution
*  @retval = 0 -> Success
*  @retval < 0 -> Error
*/
BMM350_INTF_RET_TYPE BMM350_init(struct bmm350_dev* bmm350, I2C_HandleTypeDef* I2C_handle);

/*
*  @details This function calls the internal manufacturer provided read data function and writes it to the mag_data struct
*  @param[in] bmm350 : Pointer to struct bmm350_dev that contains state for sensor
*  @param[out] mag_data: Pointer to struct bmm350_mag_temp_data containing temperature and magnetometer data
*  @return Result of magentometer and temperature read execution
*  @retval = 0 -> Success
*  @retval < 0 -> Error
*/
BMM350_INTF_RET_TYPE BMM350_read_mag_data(struct bmm350_dev* bmm350, struct bmm350_mag_temp_data* mag_data);

/*
*  @details This function checks if there was a data interrupt (not on the actual pin as we have the interrupt pin disabled) and reads the data if it is ready
*  @param[in] bmm350 : Pointer to struct bmm350_dev that contains state for sensor
*  @param[out] mag_data: Pointer to struct bmm350_mag_temp_data containing temperature and magnetometer data
*  @return Result of magentometer and temperature read execution
*  @retval = 0 -> Success
*  @retval < 0 -> Error
*/
BMM350_INTF_RET_TYPE BMM350_read_mag_data_interrupt(struct bmm350_dev* bmm350, struct bmm350_mag_temp_data* mag_data);

#endif