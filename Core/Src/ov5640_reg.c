/**
  ******************************************************************************
  * @file    ov5640_reg.c
  * @author  MCD Application Team
  * @brief   This file provides unitary register function to control the OV5640
  *          Camera driver.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2018 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "ov5640.h"
#include "ov5640_reg.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart1;

/** @addtogroup BSP
  * @{
  */

/** @addtogroup Components
  * @{
  */

/** @addtogroup OV5640
  * @brief     This file provides a set of functions needed to drive the
  *            OV5640 Camera sensor.
  * @{
  */

/**
  * @brief  Read OV5640 component registers
  * @param  ctx component contex
  * @param  reg Register to read from
  * @param  pdata Pointer to data buffer
  * @param  length Number of data to read
  * @retval Component status
  */
int32_t ov5640_read_reg(ov5640_ctx_t *ctx, uint16_t reg, uint8_t *pdata, uint16_t length)
{
	uint16_t temp = reg;
	uint8_t package[2] = {0};
	package[1] = temp;
	temp >>= 8;
	package[0] = temp;
	if (HAL_I2C_Master_Transmit(&hi2c1, (OV5640_SLAVE_ADDR << 1) | I2C_WRITE_BIT, package, sizeof(package), 100) == HAL_OK)
	{
		if (HAL_I2C_Master_Receive(&hi2c1, (OV5640_SLAVE_ADDR << 1) | I2C_READ_BIT, pdata, length, 100) == HAL_OK)
			{
				HAL_UART_Transmit(&huart1, pdata, 1, 10);
				return OV5640_OK;
			}
	}
	HAL_UART_Transmit(&huart1, ERROR_MESSAGE, sizeof(ERROR_MESSAGE), 10);
	return OV5640_ERROR;
}

/**
  * @brief  Write OV5640 component registers
  * @param  ctx component contex
  * @param  reg Register to write to
  * @param  pdata Pointer to data buffer
  * @param  length Number of data to write
  * @retval Component status
  */
int32_t ov5640_write_reg(ov5640_ctx_t *ctx, uint16_t reg, uint8_t *data, uint16_t length)
{
	uint16_t temp = reg;
	uint8_t package[3] = {0};
	package[1] = temp;
	temp >>= 8;
	package[0] = temp;
	package[2] = *data;
	if (HAL_I2C_Master_Transmit(&hi2c1, (OV5640_SLAVE_ADDR << 1) | I2C_WRITE_BIT, package, sizeof(package), 100) == HAL_OK)
	{

		HAL_UART_Transmit(&huart1, package, 3, 10);
		return OV5640_OK;
	}
	HAL_UART_Transmit(&huart1, ERROR_MESSAGE, sizeof(ERROR_MESSAGE), 10);
	return OV5640_ERROR;
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
