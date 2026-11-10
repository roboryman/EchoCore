/*
 * Copyright (c) 2024 Galahad Labs, LLC. All rights reserved.
 * ads131m02.c
 *
 *  Created on: Jan 3, 2024
 *      Author: Merrick Ryman
 */

#include <string.h>
#include "stm32f0xx_hal.h"
#include "ads131m02.h"
#include "main.h"
#include "usbd_cdc_if.h"

extern SPI_HandleTypeDef hspi1;

volatile uint8_t adc_flag = 0;
volatile uint8_t prev_flag_status = 0;
volatile uint32_t sample_number = 0;

uint8_t notify_flag = 0;


uint8_t ADS131M02_echocore_init(void)
{
	PDEBUG("Initializing ADS131M02...\r\n");

	/* Test */
	uint8_t rslt = ADS131M02_test();
	if(rslt != ADS_OK)
	{
		PDEBUG("ADC Initialization Failed\r\n");
		return ADS_GENERIC;
	}

	/* Reset before configuration setup */
	rslt = ADS131M02_reset();
	ADS131M02_print_error(rslt);
	HAL_Delay(1);

	uint16_t config_mask;

	/* Set MODE */
	config_mask = MASK_MODE_REG_CRC_DISABLE		|
				  MASK_MODE_SPI_CRC_DISABLE		|
				  MASK_MODE_CRC_TYPE_CCITT		|
				  MASK_MODE_NO_RESET			|
				  MASK_MODE_DATA_24BIT			|
				  MASK_MODE_SPI_TOUT_ENABLE		|
				  MASK_MODE_DRDY_SRC_MOST_LAG	|
				  MASK_MODE_DRDY_HIZ_LOGIC_HI	|
				  MASK_MODE_DRDY_FMT_LOGIC_LO;
	rslt = ADS131M02_wreg(REG_MODE, &config_mask, 1);
	ADS131M02_print_error(rslt);


	/* Set GAIN1 */
	config_mask = MASK_GAIN1_PGA_CH1_64	|
				  MASK_GAIN1_PGA_CH0_64;
	rslt = ADS131M02_wreg(REG_GAIN, &config_mask, 1);
	ADS131M02_print_error(rslt);


	/* Set CFG */
	config_mask = MASK_CFG_GC_DISABLE	|
				  MASK_CFG_CD_ANY		|
				  MASK_CFG_CD_DISABLE;
	rslt = ADS131M02_wreg(REG_CFG, &config_mask, 1);
	ADS131M02_print_error(rslt);


	/* Set THRSHLD_MSB */
	config_mask = MASK_EMPTY;
	rslt = ADS131M02_wreg(REG_THRSHLD_MSB, &config_mask, 1);
	ADS131M02_print_error(rslt);


	/* Set THRSHLD_LSB */
	config_mask = MASK_THRSHLD_LSB_DC_1024;
	rslt = ADS131M02_wreg(REG_THRSHLD_LSB, &config_mask, 1);
	ADS131M02_print_error(rslt);


	/* Set CH0_CFG */
	config_mask = MASK_CH0_CFG_DC_ENABLED	|
				  MASK_CH0_CFG_INPUT_PN;
	rslt = ADS131M02_wreg(REG_CH0_CFG, &config_mask, 1);
	ADS131M02_print_error(rslt);


	/* Set CH0_OCAL_MSB */
	config_mask = MASK_EMPTY;
	rslt = ADS131M02_wreg(REG_CH0_OCAL_MSB, &config_mask, 1);
	ADS131M02_print_error(rslt);


	/* Set CH0_OCAL_LSB */
	config_mask = MASK_EMPTY;
	rslt = ADS131M02_wreg(REG_CH0_OCAL_LSB, &config_mask, 1);
	ADS131M02_print_error(rslt);


	/* Set CH0_GCAL_MSB */
	config_mask = MASK_EMPTY;
	rslt = ADS131M02_wreg(REG_CH0_GCAL_MSB, &config_mask, 1);
	ADS131M02_print_error(rslt);


	/* Set CH0_GCAL_LSB */
	config_mask = MASK_EMPTY;
	rslt = ADS131M02_wreg(REG_CH0_GCAL_LSB, &config_mask, 1);
	ADS131M02_print_error(rslt);


	/* Set CH1_CFG */
	config_mask = MASK_CH1_CFG_DC_ENABLED	|
				  MASK_CH1_CFG_INPUT_PN;
	rslt = ADS131M02_wreg(REG_CH1_CFG, &config_mask, 1);
	ADS131M02_print_error(rslt);


	/* Set CH1_OCAL_MSB */
	config_mask = MASK_EMPTY;
	rslt = ADS131M02_wreg(REG_CH1_OCAL_MSB, &config_mask, 1);
	ADS131M02_print_error(rslt);


	/* Set CH1_OCAL_LSB */
	config_mask = MASK_EMPTY;
	rslt = ADS131M02_wreg(REG_CH1_OCAL_LSB, &config_mask, 1);
	ADS131M02_print_error(rslt);


	/* Set CH1_GCAL_MSB */
	config_mask = MASK_EMPTY;
	rslt = ADS131M02_wreg(REG_CH1_GCAL_MSB, &config_mask, 1);
	ADS131M02_print_error(rslt);


	/* Set CH1_GCAL_LSB */
	config_mask = MASK_EMPTY;
	rslt = ADS131M02_wreg(REG_CH1_GCAL_LSB, &config_mask, 1);
	ADS131M02_print_error(rslt);


	/* Set CLOCK */
	config_mask = MASK_CLOCK_CH1_ENABLE		|
				  MASK_CLOCK_CH0_ENABLE		|
				  MASK_CLOCK_TURBOMODE_OFF	|
				  MASK_CLOCK_OSR_16256		|
				  MASK_CLOCK_PWR_HIGH_RES;
	rslt = ADS131M02_wreg(REG_CLOCK, &config_mask, 1);
	ADS131M02_print_error(rslt);

	PDEBUG("ADS131M02 initialized successfully\r\n");

	return ADS_OK;
}


/* ADS131M02 Task */
void ADS131M02_task(void)
{
	if(adc_flag)
	{
		if(!notify_flag && prev_flag_status)
		{
			PDEBUG("WARN - Possibly skipping samples. ADC too fast / tasks too slow!\r\n");
			notify_flag = 1;
		}

		// Retrieve channel data using NULL command
		uint8_t data[FRAME_LENGTH_BYTES] = { 0 };
		ADS131M02_null(data);

		// First word: STATUS register starts at 0
		// Second word: Channel 0 data starts at FRAME_LENGTH_BYTES
		// Third word: Channel 1 data starts at FRAME_LENGTH_BYTES * 2
		// Fourth word: CRC starts at FRAME_LENGTH_BYTES * 3

		/* Put ADC data at appropriate locations in UserTxBufferFS */
		// Sample Number
		UserTxBufferFS[23] = (uint8_t)((sample_number >> 24) & 0xFF);
		UserTxBufferFS[24] = (uint8_t)((sample_number >> 16) & 0xFF);
		UserTxBufferFS[25] = (uint8_t)((sample_number >> 8) & 0xFF);
		UserTxBufferFS[26] = (uint8_t)(sample_number & 0xFF);

		// Channel 0 (24-bit twos-complement)
		uint32_t ch0_data = sign_extend_24bit_to_32bit(&data[FRAME_LENGTH_BYTES]);
		memcpy(&UserTxBufferFS[31], &ch0_data, 4);
		//UserTxBufferFS[31] = (uint8_t)((ch0_data >> 24) & 0xFF);
		//UserTxBufferFS[32] = (uint8_t)((ch0_data >> 16) & 0xFF);
		//UserTxBufferFS[33] = (uint8_t)((ch0_data >> 8) & 0xFF);
		//UserTxBufferFS[34] = (uint8_t)(ch0_data & 0xFF);

		// Channel 1 (24-bit twos-complement)
		uint32_t ch1_data = sign_extend_24bit_to_32bit(&data[2*FRAME_LENGTH_BYTES]);
		memcpy(&UserTxBufferFS[35], &ch1_data, 4);
		//UserTxBufferFS[35] = (uint8_t)((ch1_data >> 24) & 0xFF);
		//UserTxBufferFS[36] = (uint8_t)((ch1_data >> 16) & 0xFF);
		//UserTxBufferFS[37] = (uint8_t)((ch1_data >> 8) & 0xFF);
		//UserTxBufferFS[38] = (uint8_t)(ch1_data & 0xFF);

		sample_number++;

		adc_flag = 0;
		prev_flag_status = 1;
	}
	else
	{
		prev_flag_status = 0;
	}
}


/* Null Command */
uint8_t ADS131M02_null(uint8_t *response)
{
	/*
	 * NULL COMMAND FORMAT
	 * ======================
	 * 0b0000 0000 0000 0000
	 *           ^
	 *           |
	 *        Command
	 */

	/* Create NULL payload */
	// Total Words = 1 command + 1 CRC + 2 empty
	uint8_t null_payload[FRAME_LENGTH_BYTES] = { 0x00 };
	null_payload[0] = NULL_COMMAND >> 8;
	null_payload[1] = (NULL_COMMAND & 0xFF);

	/* Write RESET payload */
	ADS131M02_spi_write(null_payload, FRAME_LENGTH_BYTES);

	/* Shift out RX frame */
	//uint8_t response[FRAME_LENGTH_BYTES] = { 0 };
	ADS131M02_spi_read(response, FRAME_LENGTH_BYTES);

	return ADS_OK;
}


/* Reset Command */
uint8_t ADS131M02_reset(void)
{
	/*
	 * RESET COMMAND FORMAT
	 * ======================
	 * 0b0000 0000 0001 0001
	 *           ^
	 *           |
	 *        Command
	 */

	/* Create RESET payload */
	// Total Words = 1 command + 1 CRC + 2 empty
	uint8_t reset_payload[FRAME_LENGTH_BYTES] = { 0x00 };
	reset_payload[0] = RSET_COMMAND >> 8;
	reset_payload[1] = (RSET_COMMAND & 0xFF);

	/* Write RESET payload */
	ADS131M02_spi_write(reset_payload, FRAME_LENGTH_BYTES);

	/* Wait required time before communicating again */
	HAL_Delay(1);

	/* Confirm ADC was reset */
	uint8_t response[FRAME_LENGTH_BYTES] = { 0 };
	ADS131M02_spi_read(response, FRAME_LENGTH_BYTES);

	if(response[0] == 0x00 && response[1] == 0x11)
	{
		PDEBUG("RESET Failed Due To Incomplete Frame\r\n");
		return ADS_INCOMPLETE_FRAME;
	}

	if(response[0] != 0xFF || response[1] != 0x22)
	{
		PDEBUG("Invalid RESET Response 0x%02X%02X\r\n", response[0], response[1]);
		return ADS_INVALID_RESPONSE;
	}

	//PDEBUG("Reset ADC Success\r\n");

	return ADS_OK;
}


/* Standby Command */
uint8_t ADS131M02_standby(void)
{
	/*
	 * STANDBY COMMAND FORMAT
	 * ======================
	 * 0b0000 0000 0010 0010
	 *           ^
	 *           |
	 *        Command
	 */

	/* Create STANDBY payload */
	// Total Words = 1 command + 1 CRC + 2 empty
	uint8_t standby_payload[FRAME_LENGTH_BYTES] = { 0x00 };
	standby_payload[0] = STBY_COMMAND >> 8;
	standby_payload[1] = (STBY_COMMAND & 0xFF);

	/* Write STANDBY payload */
	ADS131M02_spi_write(standby_payload, FRAME_LENGTH_BYTES);

	/* Confirm ADC now in standby mode */
	uint8_t response[FRAME_LENGTH_BYTES] = { 0 };
	ADS131M02_spi_read(response, FRAME_LENGTH_BYTES);

	if(response[0] != (STBY_COMMAND >> 8) || response[1] != (STBY_COMMAND & 0xFF))
	{
		PDEBUG("Invalid STANDBY Response 0x%02X%02X\r\n", response[0], response[1]);
		return ADS_INVALID_RESPONSE;
	}

	//PDEBUG("Standby ADC Success (now in standby mode)\r\n");

	return ADS_OK;
}


/* Wakeup Command */
uint8_t ADS131M02_wakeup(void)
{
	/*
	 * WAKEUP COMMAND FORMAT
	 * ======================
	 * 0b0000 0000 0011 0011
	 *           ^
	 *           |
	 *        Command
	 */

	/* Create WAKEUP payload */
	// Total Words = 1 command + 1 CRC + 2 empty
	uint8_t wakeup_payload[FRAME_LENGTH_BYTES] = { 0x00 };
	wakeup_payload[0] = WKUP_COMMAND >> 8;
	wakeup_payload[1] = (WKUP_COMMAND & 0xFF);

	/* Write WAKEUP payload */
	ADS131M02_spi_write(wakeup_payload, FRAME_LENGTH_BYTES);

	/* Confirm ADC now in conversion mode */
	uint8_t response[FRAME_LENGTH_BYTES] = { 0 };
	ADS131M02_spi_read(response, FRAME_LENGTH_BYTES);

	if(response[0] != (WKUP_COMMAND >> 8) || response[1] != (WKUP_COMMAND & 0xFF))
	{
		PDEBUG("Invalid WAKEUP Response 0x%02X%02X\r\n", response[0], response[1]);
		return ADS_INVALID_RESPONSE;
	}

	//PDEBUG("Wakeup ADC Success (now in conversion mode)\r\n");

	return ADS_OK;
}


/* Lock Command */
uint8_t ADS131M02_lock(void)
{
	/*
	 * LOCK COMMAND FORMAT
	 * ======================
	 * 0b0000 0101 0101 0101
	 *           ^
	 *           |
	 *        Command
	 */

	/* Create LOCK payload */
	// Total Words = 1 command + 1 CRC + 2 empty
	uint8_t lock_payload[FRAME_LENGTH_BYTES] = { 0x00 };
	lock_payload[0] = LOCK_COMMAND >> 8;
	lock_payload[1] = (LOCK_COMMAND & 0xFF);

	/* Write LOCK payload */
	ADS131M02_spi_write(lock_payload, FRAME_LENGTH_BYTES);

	/* Confirm ADC interface is locked */
	uint8_t response[FRAME_LENGTH_BYTES] = { 0 };
	ADS131M02_spi_read(response, FRAME_LENGTH_BYTES);

	if(response[0] != (LOCK_COMMAND >> 8) || response[1] != (LOCK_COMMAND & 0xFF))
	{
		PDEBUG("Invalid LOCK Response 0x%02X%02X\r\n", response[0], response[1]);
		return ADS_INVALID_RESPONSE;
	}

	//PDEBUG("Lock ADC Interface Success\r\n");

	return ADS_OK;
}


/* Unlock Command */
uint8_t ADS131M02_unlock(void)
{
	/*
	 * UNLOCK COMMAND FORMAT
	 * ======================
	 * 0b0000 0110 0101 0101
	 *           ^
	 *           |
	 *        Command
	 */

	/* Create UNLOCK payload */
	// Total Words = 1 command + 1 CRC + 2 empty
	uint8_t unlock_payload[FRAME_LENGTH_BYTES] = { 0x00 };
	unlock_payload[0] = ULCK_COMMAND >> 8;
	unlock_payload[1] = (ULCK_COMMAND & 0xFF);

	/* Write UNLOCK payload */
	ADS131M02_spi_write(unlock_payload, FRAME_LENGTH_BYTES);

	/* Confirm ADC interface is unlocked */
	uint8_t response[FRAME_LENGTH_BYTES] = { 0 };
	ADS131M02_spi_read(response, FRAME_LENGTH_BYTES);

	if(response[0] != (ULCK_COMMAND >> 8) || response[1] != (ULCK_COMMAND & 0xFF))
	{
		PDEBUG("Invalid UNLOCK Response 0x%02X%02X\r\n", response[0], response[1]);
		return ADS_INVALID_RESPONSE;
	}

	//PDEBUG("Unlock ADC Interface Success\r\n");

	return ADS_OK;
}


/* Read Register Command */
uint8_t ADS131M02_rreg(uint8_t address, uint8_t *data, uint8_t len)
{
	/*
	 * RREG COMMAND FORMAT
	 * ======================
	 * 0b101a aaaa annn nnnn
	 *    ^    ^        ^
	 *    |    |        |
	 * Command |        |
	 *    Start Address |
	 *        Consecutive Registers
	 *
	 *  Max Start Address - 0b 0011 1111 = 0x3F
	 *  Max Consecutive Registers - 0b 0111 1111 = 0x7F
	 */

	/* Ensure data length is a multiple of 2 and non-zero, since registers are 16-bit */
	if(len & 1 || len == 0)
	{
		PDEBUG("Invalid RREG Length %d (odd bytes, non-zero)\r\n", len);
		return ADS_INVALID_DATA_LEN;
	}

	/* Calculate the ending address */
	uint8_t consecutive_addresses = ((len >> 1) - 1);
	uint16_t end_address = address + consecutive_addresses;

	/* Make sure ending address is within register mapping */
	if(end_address > MAX_ADDRESS)
	{
		PDEBUG("RREG End Address 0x%04X Exceeds Map Max\r\n", end_address);
		return ADS_REG_EXCEEDS_MAX;
	}

	/* Create RREG payload */
	// Total Words = 1 command + 1 CRC + 2 empty
	uint8_t rreg_payload[FRAME_LENGTH_BYTES] = { 0x00 };
	uint16_t rreg_command = RREG_COMMAND | (((uint16_t)address) << 7) | consecutive_addresses;
	rreg_payload[0] = rreg_command >> 8;
	rreg_payload[1] = (rreg_command & 0xFF);

	/* Write RREG payload */
	ADS131M02_spi_write(rreg_payload, FRAME_LENGTH_BYTES);

	/* Receive register contents */
	if(consecutive_addresses > 0)
	{
		/* Multiple Register Read Response */
		// Total Words = 1 ACK + 1 data + consecutive_addresses data + 1 CRC
		uint8_t response_bytes = WORD_LENGTH_BYTES * (3 + consecutive_addresses);
		uint8_t response[WORD_LENGTH_BYTES * (3 + consecutive_addresses)];
		ADS131M02_spi_read(response, FRAME_LENGTH_BYTES);

		if((response[0] & 0xE0) != 0xE0)
		{
			PDEBUG("Invalid RREG Response 0x%02X%02X\r\n", response[0], response[1]);
			return ADS_INVALID_RESPONSE;
		}

		/* First three bytes of frame are ACK (ignore) */
		/* Third bytes of register data are useless */
		uint8_t data_idx = 0;
		for(uint8_t i = 3; i < response_bytes; i += 3)
		{
			memcpy(data + data_idx, response + i, 2);
			data_idx += 2;
		}

		//PDEBUG("Read Registers 0x%02X+ Success\r\n", address);
	}
	else
	{
		/* Single Register Read Response */
		uint8_t response[FRAME_LENGTH_BYTES] = { 0 };
		ADS131M02_spi_read(response, FRAME_LENGTH_BYTES);

		/* First two bytes of frame are register contents */
		memcpy(data, response, 2);

		//PDEBUG("Read Register 0x%02X Success\r\n", address);
	}

	return ADS_OK;
}


/* Write Register Command */
uint8_t ADS131M02_wreg(uint8_t address, uint16_t *data, uint8_t len)
{
	/*
	 * WREG COMMAND FORMAT
	 * ======================
	 * 0b011a aaaa annn nnnn
	 *    ^    ^        ^
	 *    |    |        |
	 * Command |        |
	 *    Start Address |
	 *        Consecutive Registers
	 *
	 *  Max Start Address - 0b 0011 1111 = 0x3F
	 *  Max Consecutive Registers - 0b 0111 1111 = 0x7F
	 */

	/* Ensure data length is non-zero */
	if(!len)
	{
		PDEBUG("Invalid WREG Length %d\r\n", len);
		return ADS_INVALID_DATA_LEN;
	}

	/* Calculate the ending address */
	uint8_t consecutive_addresses = len - 1;
	uint16_t end_address = address + consecutive_addresses;

	/* Make sure ending address is within register mapping */
	if(end_address > MAX_ADDRESS)
	{
		PDEBUG("WREG End Address 0x%04X Exceeds Map Max\r\n", end_address);
		return ADS_REG_EXCEEDS_MAX;
	}

	/* Create WREG Payload */
	// Total Words = 1 command + 1 per register + 1 CRC + 1 empty
	uint8_t wreg_payload_bytes = WORD_LENGTH_BYTES * (3 + len);
	uint8_t wreg_payload[wreg_payload_bytes];
	uint16_t wreg_command = WREG_COMMAND | (((uint16_t)address) << 7) | consecutive_addresses;
	wreg_payload[0] = wreg_command >> 8;
	wreg_payload[1] = (wreg_command & 0xFF);
	memset(&wreg_payload[2], 0, wreg_payload_bytes - 2); // Cannot initialize VLA so use memset

	/*
	PDEBUG("Sending WREG payload 0x");
	for(int i = 0; i < wreg_payload_bytes; i++)
	{
		PDEBUG("%02X ", wreg_payload[i]);
	}
	PDEBUG("\r\n");
	*/

	// WREG Data
	// 16-bit: xx xx | yy yy | yy yy | 00 00							Next data word: 2 bytes
	//          CMD     DTA1    DTA2    CRC
	// 24-bit: xx xx 00 | yy yy 00 | yy yy 00 | 00 00 00				Next data word: 3 bytes
	//            CMD       DATA1      DATA2      CRC
	// 32-bit: xx xx 00 00 | yy yy 00 00 | yy yy 00 00 | 00 00 00 00	Next data word: 4 bytes
	//			   CMD           DTA1          DTA2          CRC
	uint8_t last_data_byte = (WORD_LENGTH_BYTES * (1 + len)) - 2;
	uint8_t data_inc = 0;
	for(uint8_t data_byte = WORD_LENGTH_BYTES; data_byte <= last_data_byte; data_byte += WORD_LENGTH_BYTES)
	{
		wreg_payload[data_byte] 	= data[data_inc] >> 8;
		wreg_payload[data_byte + 1] = (data[data_inc] & 0xFF);
		data_inc++;
	}

	/* Write WREG payload */
	ADS131M02_spi_write(wreg_payload, wreg_payload_bytes);

	/* Confirm number of registers written with ADC */
	uint8_t response[FRAME_LENGTH_BYTES] = { 0 };
	ADS131M02_spi_read(response, FRAME_LENGTH_BYTES);

	if(!(response[0] & 0x40) || ((response[1] & 0x7F) != consecutive_addresses))
	{
		PDEBUG("Invalid WREG Response 0x%02X%02X\r\n", response[0], response[1]);
		return ADS_INVALID_RESPONSE;
	}

	//PDEBUG("Write Register(s) 0x%02X+ Success\r\n", address);

	return ADS_OK;
}


static int32_t sign_extend_24bit_to_32bit(unsigned char *buffer)
{
    int32_t value = (buffer[0] << 16) | (buffer[1] << 8) | buffer[2];

    if (value & 0x00800000)
    {
        value |= 0xFF000000;
    }

    return value;
}


static void ADS131M02_print_error(uint8_t rslt)
{
    switch (rslt)
    {
        case ADS_OK:
            /*! Do nothing */
            break;

        case ADS_GENERIC:
        	PDEBUG("Error [%d] : Generic\r\n", rslt);
        	break;

        case ADS_NULL_PTR:
            PDEBUG("Error [%d] : Null pointer\r\n", rslt);
            break;

		case ADS_INCOMPLETE_FRAME:
			PDEBUG("Error [%d] : Incomplete MOSI frame\r\n", rslt);
            break;

		case ADS_INVALID_RESPONSE:
			PDEBUG("Error [%d] : Invalid response\r\n", rslt);
            break;

		case ADS_INVALID_DATA_LEN:
			PDEBUG("Error [%d] : Invalid data length\r\n", rslt);
            break;

		case ADS_REG_EXCEEDS_MAX:
			PDEBUG("Error [%d] : Register exceeds register map\r\n", rslt);
            break;

        default:
            PDEBUG("Error [%d] : Unknown error code\r\n", rslt);
            break;
    }
}


static uint8_t ADS131M02_test(void)
{
	uint8_t rslt;

	/* NULL Test */
	uint8_t response[FRAME_LENGTH_BYTES];
	rslt = ADS131M02_null(response);
	ADS131M02_print_error(rslt);
	if(rslt != ADS_OK)
	{
		PDEBUG("Test NULL - FAIL\r\n");
		return 1;
	}

	/* RESET Test */
	rslt = ADS131M02_reset();
	ADS131M02_print_error(rslt);
	if(rslt != ADS_OK)
	{
		PDEBUG("Test RESET - FAIL\r\n");
		return 1;
	}

	/* STANDBY Test */
	ADS131M02_standby();
	ADS131M02_print_error(rslt);
	if(rslt != ADS_OK)
	{
		PDEBUG("Test STANDBY - FAIL\r\n");
		return 1;
	}

	/* WAKEUP Test */
	ADS131M02_wakeup();
	ADS131M02_print_error(rslt);
	if(rslt != ADS_OK)
	{
		PDEBUG("Test WAKEUP - FAIL\r\n");
		return 1;
	}

	/* LOCK Test */
	ADS131M02_lock();
	ADS131M02_print_error(rslt);
	if(rslt != ADS_OK)
	{
		PDEBUG("Test LOCK - FAIL\r\n");
		return 1;
	}

	/* UNLOCK Test */
	ADS131M02_unlock();
	ADS131M02_print_error(rslt);
	if(rslt != ADS_OK)
	{
		PDEBUG("Test UNLOCK - FAIL\r\n");
		return 1;
	}

	/* RREG Test */
	uint8_t id_data[2] = { 0 };
	rslt = ADS131M02_rreg(REG_ID, id_data, 2);
	ADS131M02_print_error(rslt);
	if(rslt != ADS_OK || id_data[0] != ID)
	{
		PDEBUG("Test RREG - FAIL with ID = 0x%02X\r\n", id_data[0]);
		return 1;
	}

	/* WREG Test */
	uint8_t old_gain_data[2] = { 0 };
	ADS131M02_rreg(REG_GAIN, old_gain_data, 2);

	uint16_t gain_test_data = 0x0077;
	rslt = ADS131M02_wreg(REG_GAIN, &gain_test_data, 1);
	ADS131M02_print_error(rslt);
	if(rslt != ADS_OK)
	{
		PDEBUG("Test WREG - FAIL\r\n");
		return 1;
	}

	uint8_t new_gain_data[2] = { 0 };
	ADS131M02_rreg(REG_GAIN, new_gain_data, 2);

	if(new_gain_data[0] != (gain_test_data >> 8) || new_gain_data[1] != (gain_test_data & 0xFF))
	{
		PDEBUG("Test WREG - FAIL with gain data = 0x%02X%02X\r\n", new_gain_data[0], new_gain_data[1]);
		return 1;
	}

	PDEBUG("ALL TESTS PASSED - ADS131M02\r\n");

	return 0;
}


static int8_t ADS131M02_spi_read(uint8_t *response, uint16_t length)
{
	//HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET); // NSS low

	/* Receive response */
	HAL_SPI_Receive(&SPI_HANDLE, response, length, BUS_TIMEOUT);

	/* Wait for XMISSION to complete */
	while(SPI_HANDLE.State == HAL_SPI_STATE_BUSY);

	//HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET); // NSS high

	//return ADS131M02_OK;
	return 0;
}


static int8_t ADS131M02_spi_write(const uint8_t *payload, uint16_t length)
{
	/* Command length should be WORD_LENGTH_BYTES */

	/* Fill TX buffer with required number of words */
	//memcpy(GTXBuffer, command, WORD_LENGTH_BYTES);
	//memcpy(&GTXBuffer[WORD_LENGTH_BYTES], 0, FRAME_LENGTH_BYTES - WORD_LENGTH_BYTES);

	//HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET); // NSS low

	/* Transmit Payload */
	HAL_SPI_Transmit(&SPI_HANDLE, payload, length, BUS_TIMEOUT);

	/* Wait for XMISSION to complete */
	while(SPI_HANDLE.State == HAL_SPI_STATE_BUSY);

	//HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET); // NSS high

	//return ADS131M02_OK;
	return 0;
}
