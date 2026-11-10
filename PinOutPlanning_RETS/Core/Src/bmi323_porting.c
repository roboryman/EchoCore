#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>
#include "stm32f0xx_hal.h"
#include "bmi323_porting.h"
#include "main.h"


extern SPI_HandleTypeDef hspi2;


uint8_t GTXBuffer[512], GRXBuffer[2048];


void Enable_MCU_IMU_INT1_Pin(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	__HAL_RCC_GPIOB_CLK_ENABLE(); // <-- set to port GPIOA or GPIOB depending on pin (PB12 for int1)
	
	/*Configure GPIO pin : IMU_INT1_Pin PB12*/
	GPIO_InitStruct.Pin = IMU_INT1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(IMU_INT1_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(IMU_INT1_EXTI_IRQn, 1, 0);
  	HAL_NVIC_EnableIRQ(IMU_INT1_EXTI_IRQn);

  	PDEBUG("IMU Interrupt 1 Enabled\r\n");
}


void Disable_MCU_IMU_INT1_Pin(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	/*Configure GPIO pin : IMU_INT1_Pin PB12*/
	GPIO_InitStruct.Pin = IMU_INT1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(IMU_INT1_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(IMU_INT1_EXTI_IRQn, 5, 0);
  	HAL_NVIC_DisableIRQ(IMU_INT1_EXTI_IRQn);

  	PDEBUG("IMU Interrupt 1 Disabled\r\n");
}


void Enable_MCU_IMU_INT2_Pin(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	__HAL_RCC_GPIOA_CLK_ENABLE(); // <-- set to port GPIOA or GPIOB depending on pin (PA9 for int2)
	
	/*Configure GPIO pin : IMU_INT2_Pin PA9*/
	GPIO_InitStruct.Pin = IMU_INT2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(IMU_INT2_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(IMU_INT2_EXTI_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(IMU_INT2_EXTI_IRQn);

	PDEBUG("IMU Interrupt 2 Enabled\r\n");
}


void Disable_MCU_IMU_INT2_Pin(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	/*Configure GPIO pin : IMU_INT2_Pin PA9*/
	GPIO_InitStruct.Pin = IMU_INT2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(IMU_INT2_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(IMU_INT2_EXTI_IRQn, 5, 0);
	HAL_NVIC_DisableIRQ(IMU_INT2_EXTI_IRQn);

	PDEBUG("IMU Interrupt 2 Disabled\r\n");
}


void bst_delay_us(uint32_t period, void *intf_ptr)
{
	uint32_t i;

	while(period--)
	{
		for(i = 0; i < 84; i++)
		{
			;
		}
	}
}



int8_t SensorAPI_SPIx_Read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    GTXBuffer[0] = reg_addr | 0x80;

    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET); // NSS low

    //HAL_SPI_TransmitReceive(&hspi2, pTxData, pRxData, ReadNumbr+1, BUS_TIMEOUT); // timeout 1000msec;
    HAL_SPI_TransmitReceive(&SPI_HANDLE, GTXBuffer, GRXBuffer, length+1, BUS_TIMEOUT); // timeout 1000msec;
    while(SPI_HANDLE.State == HAL_SPI_STATE_BUSY);  // wait for xmission complete

    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET); // NSS high
    memcpy(reg_data, GRXBuffer+1, length);

    return 0;
}


int8_t SensorAPI_SPIx_Write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    GTXBuffer[0] = reg_addr & 0x7F;
    memcpy(&GTXBuffer[1], reg_data, length);

    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET); // NSS low

    //HAL_SPI_TransmitReceive(&hspi2, pTxData, pRxData, WriteNumbr+1, BUS_TIMEOUT); // send register address + write data
    HAL_SPI_Transmit(&SPI_HANDLE, GTXBuffer, length+1, BUS_TIMEOUT); // send register address + write data
    while(SPI_HANDLE.State == HAL_SPI_STATE_BUSY);  // wait for xmission complete

    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET); // NSS high

    return 0;
}
