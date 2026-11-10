#ifndef BMI323_PORTING_H__
#define BMI323_PORTING_H__

#include <stdint.h>


#define SPI_HANDLE	(hspi2) // IMU on SPI2 for EchoCore Rev A
#define BUS_TIMEOUT 1000


typedef enum
{
	BHI260_GPIO_PIN_LOW = 0,
	BHI260_GPIO_PIN_HIGH
} BHI260_GPIO_PinState;


void Enable_MCU_IMU_INT1_Pin(void);
void Disable_MCU_IMU_INT1_Pin(void);
void Enable_MCU_IMU_INT2_Pin(void);
void Disable_MCU_IMU_INT2_Pin(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);


void bst_delay_us(uint32_t period, void *intf_ptr);


int8_t SensorAPI_SPIx_Read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr);
int8_t SensorAPI_SPIx_Write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr);


#endif
