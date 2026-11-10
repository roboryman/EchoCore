#include <stdio.h>
#include <string.h>
#include <math.h>
//#include "cmsis_os.h"
#include "stm32f0xx_hal.h"
#include "bmi323_porting.h"
#include "bmi323.h"
#include "bmi323_common.h"
#include "main.h"
#include "usbd_cdc_if.h"

#if defined(USE_BMI323)


volatile uint8_t imu_int1_flag = 0;
volatile uint8_t imu_int2_flag = 0;
volatile uint16_t bmi323_fifo_ready = 0;


struct bmi3_dev bmi323dev;
uint8_t bmi323_dev_addr;


#if defined(ACC_ONLY) || defined(GYRO_ONLY) || defined(ACC_GYRO)
struct bmi3_sensor_data sensor_data[2] = { { 0 } };
uint16_t temp_data;
#endif


#if defined(FIFO_POLL) || defined(FIFO_WM_INT) || defined(STEP_COUNTER)
/* Variable to index bytes */
uint16_t idx = 0;

/* Number of bytes of FIFO data (default 2048) */
uint8_t fifo_data[2048] = {0};

/* Array of accelerometer frames -> Total bytes =
150 * (6 axes bytes + 1 header byte) = 1050 bytes*/
struct bmi3_fifo_sens_axes_data fifo_accel_data[100] = { {0} };
struct bmi3_fifo_sens_axes_data fifo_gyr_data[100] = { {0} };
struct bmi3_fifo_temperature_data fifo_temp_data[100];


/* Initialize FIFO frame structure */
struct bmi3_fifo_frame fifoframe = {0};
#endif

/******************************************************************************/
/*!               User interface functions                                    */

/*!
 * @brief This API prints the execution status
 */
void bmi3_error_codes_print_result(const char api_name[], int8_t rslt)
{
    switch (rslt)
    {
        case BMI3_OK:

            /*! Do nothing */
            break;

        case BMI3_E_NULL_PTR:
        	PDEBUG("%s\t", api_name);
            PDEBUG(
                "Error [%d] : Null pointer error. It occurs when the user tries to assign value (not address) to a pointer," " which has been initialized to NULL.\r\n",
                rslt);
            break;

        case BMI3_E_COM_FAIL:
            PDEBUG("%s\t", api_name);
            PDEBUG(
                "Error [%d] : Communication failure error. It occurs due to read/write operation failure and also due " "to power failure during communication\r\n",
                rslt);
            break;

        case BMI3_E_DEV_NOT_FOUND:
            PDEBUG("%s\t", api_name);
            PDEBUG("Error [%d] : Device not found error. It occurs when the device chip id is incorrectly read\r\n",
                   rslt);
            break;

        case BMI3_E_INVALID_SENSOR:
            PDEBUG("%s\t", api_name);
            PDEBUG(
                "Error [%d] : Invalid sensor error. It occurs when there is a mismatch in the requested feature with the " "available one\r\n",
                rslt);
            break;

        case BMI3_E_INVALID_INT_PIN:
            PDEBUG("%s\t", api_name);
            PDEBUG(
                "Error [%d] : Invalid interrupt pin error. It occurs when the user tries to configure interrupt pins " "apart from INT1 and INT2\r\n",
                rslt);
            break;

        case BMI3_E_ACC_INVALID_CFG:
            PDEBUG("%s\t", api_name);
            PDEBUG(
                "Error [%d] : Invalid Accel configuration error. It occurs when there is an error in accel configuration" " register which could be one among range, BW or filter performance in reg address 0x20\r\n",
                rslt);
            break;

        case BMI3_E_GYRO_INVALID_CFG:
            PDEBUG("%s\t", api_name);
            PDEBUG(
                "Error [%d] : Invalid Gyro configuration error. It occurs when there is a error in gyro configuration" "register which could be one among range, BW or filter performance in reg address 0x21\r\n",
                rslt);
            break;

        case BMI3_E_INVALID_INPUT:
            PDEBUG("%s\t", api_name);
            PDEBUG("Error [%d] : Invalid input error. It occurs when the sensor input validity fails\r\n", rslt);
            break;

        case BMI3_E_INVALID_STATUS:
            PDEBUG("%s\t", api_name);
            PDEBUG("Error [%d] : Invalid status error. It occurs when the feature/sensor validity fails\r\n", rslt);
            break;

        case BMI3_E_DATA_RDY_INT_FAILED:
            PDEBUG("%s\t", api_name);
            PDEBUG(
                "Error [%d] : Data ready interrupt error. It occurs when the sample count exceeds the FOC sample limit " "and data ready status is not updated\r\n",
                rslt);
            break;

        case BMI3_E_INVALID_FOC_POSITION:
            PDEBUG("%s\t", api_name);
            PDEBUG(
                "Error [%d] : Invalid FOC position error. It occurs when average FOC data is obtained for the wrong" " axes\r\n",
                rslt);
            break;

        case BMI3_E_INVALID_ST_SELECTION:
            PDEBUG("%s\t", api_name);
            PDEBUG(
                "Error [%d] : Invalid self-test selection error. It occurs when there is an invalid precondition" "settings such as alternate accelerometer and gyroscope enable bits, accelerometer mode and output data rate\r\n",
                rslt);
            break;

        case BMI3_E_OUT_OF_RANGE:
            PDEBUG("%s\t", api_name);
            PDEBUG(
                "Error [%d] : Out of range error. It occurs when the range exceeds the maximum range for accel while performing FOC\r\n",
                rslt);
            break;

        case BMI3_E_FEATURE_ENGINE_STATUS:
            //PDEBUG("%s : ", api_name);
            //PDEBUG("Error [%d] : Feature engine status error. It occurs when the feature engine enable mask is not set\r\n", rslt);
            PDEBUG("Feature engine status error\r\n", rslt);
            break;

        default:
            PDEBUG("%s\t", api_name);
            PDEBUG("Error [%d] : Unknown error code\r\n", rslt);
            break;
    }
}

/*!
 *  @brief Function to select the interface between SPI and I2C.
 *  Also to initialize coines platform
 */
int8_t bmi3_interface_init(struct bmi3_dev *bmi, uint8_t intf)
{
	int8_t rslt = BMI3_OK;
	
	/* Bus configuration : SPI */
	if (intf == BMI3_SPI_INTF)
	{
		PDEBUG("Selecting SPI interface\r\n");
		bmi->intf = BMI3_SPI_INTF;
		bmi->read = (bmi3_read_fptr_t)SensorAPI_SPIx_Read;
		bmi->write = (bmi3_write_fptr_t)SensorAPI_SPIx_Write;
	}

	/* Assign device address to interface pointer */
	bmi->intf_ptr = &bmi323_dev_addr;

	/* Configure delay in microseconds */
	bmi->delay_us = bst_delay_us;

	/* Configure max read/write length (in bytes) ( Supported length depends on target machine) */
	bmi->read_write_len = READ_WRITE_LEN;

	return rslt;
}

/*!
 * @brief This function converts lsb to meter per second squared for 16 bit accelerometer at
 * range 2G, 4G, 8G or 16G.
 */
static float lsb_to_mps2(int16_t val, float g_range, uint8_t bit_width)
{
    float half_scale = ((float)(1 << bit_width) / 2.0f);

    return (GRAVITY_EARTH * val * g_range) / half_scale;
}

/*!
 * @brief This function converts lsb to degree per second for 16 bit gyro at
 * range 125, 250, 500, 1000 or 2000dps.
 */
static float lsb_to_dps(int16_t val, float dps, uint8_t bit_width)
{
    double power = 2;

    float half_scale = (float)((pow((double)power, (double)bit_width) / 2.0f));

    return (dps / (half_scale)) * (val);
}

int8_t Open_BMI323_ACC(struct bmi3_dev *dev)
{
	int8_t rslt = BMI3_OK;
	/* Sensor configuration structure */
	struct bmi3_sens_config config = { 0 };

	config.type = BMI3_ACCEL;
	
	/* Get the previous or default configuration settings */
	rslt = bmi323_get_sensor_config(&config, 1, dev);
	if (rslt == BMI3_OK) 
	{
		/* Update all or any of the accelerometer
		configurations */
		/* The Accel bandwidth coefficient defines the 3 dB cutoff frequency in relation to the ODR. */
		config.cfg.acc.bwp = BMI3_ACC_BW_ODR_HALF;

		/* Set number of average samples for accel. */
		config.cfg.acc.avg_num = BMI3_ACC_AVG4;

		/* Enable the accel mode where averaging of samples
		* will be done based on above set bandwidth and ODR.
		* Note : By default accel is disabled. The accel will get enable by selecting the mode.
		*/
		config.cfg.acc.acc_mode = BMI3_ACC_MODE_NORMAL;
		

		config.cfg.acc.odr      = BMI3_ACC_ODR_50HZ;

		/* Gravity range of the sensor (+/- 2G, 4G, 8G, 16G). */
		config.cfg.acc.range     = BMI3_ACC_RANGE_4G;
		
		/* Set the configurations */
		rslt = bmi323_set_sensor_config(&config, 1, dev);
		if (rslt != BMI3_OK) 
		{
			PDEBUG("Open ACC failed, rslt=%d\r\n", rslt);
		} 
		else
		{
			PDEBUG("Open ACC set successfully\r\n");

			/* Get the configuration settings for validation */
			rslt = bmi323_get_sensor_config(&config, 1, dev);
			if (rslt == BMI3_OK) 
			{
				PDEBUG("Get ACC configuration successful\r\n");
				PDEBUG("acc_mode = %d\r\n", config.cfg.acc.acc_mode);
				PDEBUG("bwp = %d\r\n", config.cfg.acc.bwp);
				PDEBUG("odr = %d\r\n", config.cfg.acc.odr);
				PDEBUG("Range = %d\r\n", config.cfg.acc.range);
				PDEBUG("avg_num = %d\r\n", config.cfg.acc.avg_num);
			}
		}
	}

	return rslt;
}

int8_t Close_BMI323_ACC(struct bmi3_dev *dev)
{
	int8_t rslt = BMI3_OK;
	/* Sensor configuration structure */
	struct bmi3_sens_config config = { 0 };

	config.type = BMI3_ACCEL;
	/* Get the previous or default configuration settings */
	rslt = bmi323_get_sensor_config(&config, 1, dev);
	if (rslt == BMI3_OK) 
	{
		config.cfg.acc.acc_mode = BMI3_ACC_MODE_DISABLE;
		
		/* Set the configurations */
		rslt = bmi323_set_sensor_config(&config, 1, dev);
		if (rslt != BMI3_OK) 
		{
			PDEBUG("Close ACC failed, rslt=%d\r\n", rslt);
		} 
		else
		{
			PDEBUG("Open ACC successfully\r\n");
		}
	}
	bmi3_error_codes_print_result("Close_BMI323_ACC",rslt);

	return rslt;
}

int8_t Open_BMI323_GYRO(struct bmi3_dev *dev)
{
	int8_t rslt = BMI3_OK;
	/* Sensor configuration structure */
	struct bmi3_sens_config config = { 0 };

	config.type = BMI3_GYRO;
	
	/* Get the previous or default configuration settings */
	rslt = bmi323_get_sensor_config(&config, 1, dev);
	if (rslt == BMI3_OK) 
	{
		config.cfg.gyr.odr = BMI3_GYR_ODR_50HZ;
		/* Gyroscope Angular Rate Measurement Range. By default the range is 2000dps. */
		config.cfg.gyr.range = BMI3_GYR_RANGE_2000DPS;

		/*	The Gyroscope bandwidth coefficient defines the 3 dB cutoff frequency in relation to the ODR
		*	Value	Name	  Description
		*	  0   odr_half	 BW = gyr_odr/2
		*	  1  odr_quarter BW = gyr_odr/4
		*/
		config.cfg.gyr.bwp = BMI3_GYR_BW_ODR_HALF;
		/* By default the gyro is disabled. Gyro is enabled by selecting the mode. */
		config.cfg.gyr.gyr_mode = BMI3_GYR_MODE_NORMAL;
		/* Value    Name    Description
		*  000     avg_1   No averaging; pass sample without filtering
		*  001     avg_2   Averaging of 2 samples
		*  010     avg_4   Averaging of 4 samples
		*  011     avg_8   Averaging of 8 samples
		*  100     avg_16  Averaging of 16 samples
		*  101     avg_32  Averaging of 32 samples
		*  110     avg_64  Averaging of 64 samples
		*/
		config.cfg.gyr.avg_num = BMI3_GYR_AVG4;
	
		/* Set the configurations */
		rslt = bmi323_set_sensor_config(&config, 1, dev);
		if (rslt != BMI3_OK) 
		{
			PDEBUG("Open GYRO failed\r\n");
		} 
		else
		{
			PDEBUG("Open GYRO successfully\r\n");

			/* Get the configuration settings for validation */
			rslt = bmi323_get_sensor_config(&config, 1, dev);
			if (rslt == BMI3_OK) 
			{
				PDEBUG("Get BMI2_GYRO Configuration successful\r\n");
				PDEBUG("gyr_mode = %d\r\n", config.cfg.gyr.gyr_mode);
				PDEBUG("ODR = %d\r\n", config.cfg.gyr.odr);
				PDEBUG("Range = %d\r\n", config.cfg.gyr.range);
				PDEBUG("bwp = %d\r\n", config.cfg.gyr.bwp);
				PDEBUG("avg_num = %d\r\n", config.cfg.gyr.avg_num);
			}
		}
	}

	return rslt;
}

int8_t Close_BMI323_GYRO(struct bmi3_dev *dev)
{
	int8_t rslt = BMI3_OK;

	/* Sensor configuration structure */
	struct bmi3_sens_config config = { 0 };

	config.type = BMI3_GYRO;
	/* Get the previous or default configuration settings */
	rslt = bmi323_get_sensor_config(&config, 1, dev);
	if (rslt == BMI3_OK) 
	{
		config.cfg.gyr.gyr_mode = BMI3_GYR_MODE_DISABLE;
		
		/* Set the configurations */
		rslt = bmi323_set_sensor_config(&config, 1, dev);
		if (rslt != BMI3_OK) 
		{
			PDEBUG("Close GYRO failed, rslt=%d\r\n", rslt);
		} 
		else
		{
			PDEBUG("Open GYRO successfully\r\n");
		}
	}
	bmi3_error_codes_print_result("Close_BMI323_GYRO",rslt);

	return rslt;
}


#if defined(FIFO_POLL) || defined(FIFO_WM_INT)
int8_t Open_BMI323_FIFO(struct bmi3_dev *dev)
{
	int8_t rslt = BMI3_OK;

	#if defined(FIFO_WM_INT)
	struct bmi3_int_pin_config int_cfg;
	struct bmi3_map_int map_int = { 0 };
	#endif

	/* Array to define set FIFO flush */
	uint8_t data[2] = { BMI323_ENABLE, 0 };

	/* Set the FIFO flush in FIFO control register to clear the FIFO data */
	rslt = bmi323_set_regs(BMI3_REG_FIFO_CTRL, data, 2, dev);
	bmi3_error_codes_print_result("bmi323_set_regs", rslt);

	/* Clear FIFO configuration register */
	rslt = bmi3_set_fifo_config(BMI3_FIFO_ALL_EN, BMI3_DISABLE, dev);
	bmi3_error_codes_print_result("bmi323_set_fifo_config", rslt);

	/* Example: 100Hz ODR, read data per second */
	/* Set FIFO configuration by enabling accelerometer and gyroscope*/
	PDEBUG("FIFO Header is disabled\r\n");

	#if defined(FIFO_POLL) || defined(FIFO_WM_INT)
	/* Set the water-mark level */
	rslt = bmi3_set_fifo_wm(300, dev);//12*50=600 bytes, 100HZ ODR, read data every second

	if (rslt != BMI3_OK)
	{
		PDEBUG("bmi3_set_fifo_wm error, error code: %d\r\n", rslt);
	}

	rslt = bmi3_set_fifo_config(BMI3_FIFO_ACC_EN | BMI3_FIFO_GYR_EN | BMI3_FIFO_TEMP_EN, BMI3_ENABLE, dev); // TODO - Also include temp reading in FIFO

	if (rslt != BMI3_OK)
	{
		PDEBUG("Set fifo config failed\r\n");
	}
	#endif

	#if defined(FIFO_WM_INT)
	bmi3_get_int_pin_config(&int_cfg, dev);
	
	int_cfg.pin_type = BMI3_INT1;
	int_cfg.pin_cfg[0].lvl = BMI3_INT_ACTIVE_LOW; // Active Low!
	int_cfg.pin_cfg[0].od = BMI3_INT_PUSH_PULL;
	int_cfg.pin_cfg[0].output_en = BMI3_INT_OUTPUT_ENABLE;

	bmi3_set_int_pin_config(&int_cfg, dev);

	/* Map the FIFO water-mark interrupt to INT1 */
	/* Note: User can map the interrupt to INT1 or INT2 */
	map_int.fifo_watermark_int = BMI3_INT1;

	/* Map the interrupt configuration */
	rslt = bmi323_map_interrupt(map_int, dev);
	bmi3_error_codes_print_result("bmi323_map_interrupt", rslt);
	#endif

	/* Update FIFO structure */
	fifoframe.data = fifo_data;
	
	return rslt;
}
#endif

int8_t Close_BMI323_FIFO(struct bmi3_dev *dev)
{
	int8_t rslt = BMI3_OK;
	
	/* Clear FIFO configuration register */
	rslt = bmi3_set_fifo_config(BMI3_FIFO_ALL_EN, BMI3_DISABLE, dev);

	if (rslt != BMI3_OK) 
	{
		PDEBUG("Clear FIFO configuration register error, error code: %d\r\n", rslt);
		return rslt;
	}
	
	return rslt;
}


void BMI323_Print_ALLRegs(void)
{
	struct bmi3_dev *dev;
	dev = &bmi323dev;

	uint8_t value[2];

	uint8_t reg_addr;

	for(reg_addr = 0; reg_addr < 0x80; reg_addr++)
	{
		bmi3_get_regs(reg_addr, value, 2, dev);
		PDEBUG("0x%02X, value=0x%02X%02X\r\n", reg_addr, value[1], value[0]);
	}
}

int8_t Init_BMI323(void)
{
	struct bmi3_dev *dev;
	dev = &bmi323dev;

	int8_t rslt = BMI3_OK;
	uint8_t chipid;

	/* Interface reference is given as a parameter
	* For I2C : BMI3_I2C_INTF
	* For SPI : BMI3_SPI_INTF
	*/
	rslt = bmi3_interface_init(dev, BMI3_SPI_INTF);
	bmi3_error_codes_print_result("bmi3_interface_init",rslt);

	//bmi3_soft_reset(dev);
	HAL_Delay(100);	

	/* Initialize bmi323. */
	PDEBUG("Initializing BMI323...\r\n");
	rslt = bmi323_init(dev);
	bmi3_error_codes_print_result("bmi323_init", rslt); // <-- breaks future debug statements ....?

	if (rslt != BMI3_OK)
	{
		PDEBUG("bmi323_init() failed, error code: %d\r\n", rslt);
		return rslt;
	}
	else
	{
		PDEBUG("BMI323 initialized successfully\r\n");
	}

	rslt = bmi3_get_regs(BMI3_REG_CHIP_ID, &chipid, 1, dev);
	if (rslt != BMI3_OK) 
	{
		PDEBUG("read chip ID failed, error code: %d\r\n", rslt);
		return rslt;
	}

	PDEBUG("Chip ID:0x%02x\r\n", chipid);

	//rslt = bmi323_get_config_version(&ver_major, &ver_minor, dev);
	//PDEBUG("The firmware version: v%d.%d\r\n", ver_major, ver_minor);

	/*Axis mapping according phisical structure*/
#if 0//For axis remap
	/* Strusture instance of axes remap */
	struct bmi3_axes_remap remap = { 0 };

	rslt = bmi323_get_remap_axes(&remap, &dev);
	bmi3_error_codes_print_result("get_remap_axes", rslt);

	/* @note: XYZ axis denotes x = x, y = y, z = z
	* Similarly,
	*    ZYX, x = z, y = y, z = x
	*/
	remap.axis_map = BMI3_MAP_ZYX_AXIS;

	/* Invert the x axis of accelerometer and gyroscope */
	remap.invert_x = BMI3_MAP_NEGATIVE;

	/* Invert the y axis of accelerometer and gyroscope */
	remap.invert_y = BMI3_MAP_NEGATIVE;

	/* Invert the z axis of accelerometer and gyroscope */
	remap.invert_z = BMI3_MAP_NEGATIVE;

	if (rslt == BMI323_OK)
	{
		rslt = bmi323_set_remap_axes(remap, &dev);
		bmi3_error_codes_print_result("set_remap_axes", rslt);

		if (rslt != BMI323_OK)
		{
			PDEBUG("bmi323_set_remap_axes() failed\r\n");
		}
	}
	else
	{
		PDEBUG("bmi323_get_remap_axes() failed\r\n");
	}

#endif
#if defined(FOC)
	//bmi3_perform_accel_foc();
	//bmi3_perform_gyro_foc(dev)
#endif
	struct bmi3_st_result st_result_status = { 0 };
#if defined(ACC_SELFTEST)
	rslt = bmi323_perform_self_test(BMI3_ST_ACCEL_ONLY, &st_result_status, dev);
	bmi3_error_codes_print_result("Perform_self_test", rslt);

	if ((rslt == BMI323_OK) && (st_result_status.self_test_rslt == BMI323_TRUE))
	{
		PDEBUG("ACC self-test is successfully completed \n");
	}
	if ((rslt == BMI323_OK) && (st_result_status.self_test_rslt == BMI323_FALSE))
	{
		PDEBUG("Self-test is not successfully completed\n");

		switch (st_result_status.self_test_err_rslt)
		{
			case BMI3_SC_ST_ABORTED_MASK:
				PDEBUG("SC_ST_ABORTED\n");
				break;
			case BMI3_ST_IGNORED_MASK:
				PDEBUG("BMI323_ST_IGNORED\n");
				break;
			case BMI3_SC_ST_PRECON_ERR_MASK:
				PDEBUG("BMI323_SC_ST_PRECON_ERR\n");
				break;
			case BMI3_MODE_CHANGE_WHILE_SC_ST_MASK:
				PDEBUG("BMI323_MODE_CHANGE_WHILE_SC_ST\n");
				break;
			default:
				break;
		}
	}

	PDEBUG("Result of acc_x_axis is %d\n", st_result_status.acc_sens_x_ok);
	PDEBUG("Result of acc_y_axis is %d\n", st_result_status.acc_sens_y_ok);
	PDEBUG("Result of acc_z_axis is %d\n", st_result_status.acc_sens_z_ok);
	PDEBUG("Result of self-test error is %d\n", st_result_status.self_test_err_rslt);
	PDEBUG("Result of ST_result is %d\n", st_result_status.self_test_rslt);

	for(;;)
	{
		;
	}
#elif defined(GYRO_SELFTEST)
	rslt = bmi323_perform_self_test(BMI3_ST_GYRO_ONLY, &st_result_status, dev);
	bmi3_error_codes_print_result("Perform_self_test", rslt);

	if ((rslt == BMI323_OK) && (st_result_status.self_test_rslt == BMI323_TRUE))
	{
		PDEBUG("ACC self-test is successfully completed \n");
	}
	if ((rslt == BMI323_OK) && (st_result_status.self_test_rslt == BMI323_FALSE))
	{
		PDEBUG("Self-test is not successfully completed\n");

		switch (st_result_status.self_test_err_rslt)
		{
			case BMI3_SC_ST_ABORTED_MASK:
				PDEBUG("SC_ST_ABORTED\n");
				break;
			case BMI3_ST_IGNORED_MASK:
				PDEBUG("BMI323_ST_IGNORED\n");
				break;
			case BMI3_SC_ST_PRECON_ERR_MASK:
				PDEBUG("BMI323_SC_ST_PRECON_ERR\n");
				break;
			case BMI3_MODE_CHANGE_WHILE_SC_ST_MASK:
				PDEBUG("BMI323_MODE_CHANGE_WHILE_SC_ST\n");
				break;
			default:
				break;
		}
	}

	PDEBUG("Result of gyr_x_axis is %d\n", st_result_status.gyr_sens_x_ok);
	PDEBUG("Result of gyr_y_axis is %d\n", st_result_status.gyr_sens_y_ok);
	PDEBUG("Result of gyr_z_axis is %d\n", st_result_status.gyr_sens_z_ok);
	PDEBUG("Result of gyr_drive_ok is %d\n", st_result_status.gyr_drive_ok);
	PDEBUG("Result of self-test error is %d\n", st_result_status.self_test_err_rslt);
	PDEBUG("Result of ST_result is %d\n", st_result_status.self_test_rslt);

	for(;;)
	{
		;
	}
#elif defined(ACC_GYRO_SELFTEST)
	rslt = bmi323_perform_self_test(BMI3_ST_BOTH_ACC_GYR, &st_result_status, dev);
	bmi3_error_codes_print_result("Perform_self_test", rslt);

	if ((rslt == BMI323_OK) && (st_result_status.self_test_rslt == BMI323_TRUE))
	{
		PDEBUG("ACC self-test is successfully completed \n");
	}
	if ((rslt == BMI323_OK) && (st_result_status.self_test_rslt == BMI323_FALSE))
	{
		PDEBUG("Self-test is not successfully completed\n");

		switch (st_result_status.self_test_err_rslt)
		{
			case BMI3_SC_ST_ABORTED_MASK:
				PDEBUG("SC_ST_ABORTED\n");
				break;
			case BMI3_ST_IGNORED_MASK:
				PDEBUG("BMI323_ST_IGNORED\n");
				break;
			case BMI3_SC_ST_PRECON_ERR_MASK:
				PDEBUG("BMI323_SC_ST_PRECON_ERR\n");
				break;
			case BMI3_MODE_CHANGE_WHILE_SC_ST_MASK:
				PDEBUG("BMI323_MODE_CHANGE_WHILE_SC_ST\n");
				break;
			default:
				break;
		}
	}

	PDEBUG("Result of acc_x_axis is %d\n", st_result_status.acc_sens_x_ok);
	PDEBUG("Result of acc_y_axis is %d\n", st_result_status.acc_sens_y_ok);
	PDEBUG("Result of acc_z_axis is %d\n", st_result_status.acc_sens_z_ok);
	PDEBUG("Result of gyr_x_axis is %d\n", st_result_status.gyr_sens_x_ok);
	PDEBUG("Result of gyr_y_axis is %d\n", st_result_status.gyr_sens_y_ok);
	PDEBUG("Result of gyr_z_axis is %d\n", st_result_status.gyr_sens_z_ok);
	PDEBUG("Result of gyr_drive_ok is %d\n", st_result_status.gyr_drive_ok);
	PDEBUG("Result of self-test error is %d\n", st_result_status.self_test_err_rslt);
	PDEBUG("Result of ST_result is %d\n", st_result_status.self_test_rslt);
	for(;;)
	{
		;
	}	
#elif defined(CRT)
	uint8_t sens_sel[2] = { BMI3_ACCEL, BMI3_GYRO };

	rslt = bmi2_sensor_enable(&sens_sel[0], 1, dev);    
	bmi2_perform_accel_self_test(rslt);    
	
	rslt = bmi2_sensor_disable(&sens_sel[1], 1, dev);    
	bmi2_perform_accel_self_test(rslt);    
	
	sensor_data.type = BMI2_ACCEL;    
	dev.delay_us(100000);    
	PDEBUG("before CRT Accel x,y,z values\r\n");

	/* read the accel data before CRT*/    
	rslt = bmi2_get_sensor_data(&sensor_data, 1, dev);    
	bmi2_perform_accel_self_test(rslt);    
	PDEBUG("X axes: %d, Y axes: %d, Z axes: %d\r\n", sensor_data.sens_data.acc.x, sensor_data.sens_data.acc.y, sensor_data.sens_data.acc.z );    

	/*brief This API is to run the CRT process*/    
	rslt = bmi2_do_crt(dev);    
	bmi2_perform_accel_self_test(rslt);    
	if(rslt == BMI3_OK)    
	{    	
		PDEBUG("After CRT Accel x,y,z values\r\n");    	
		/* read the accel data after CRT*/    	
		rslt = bmi2_get_sensor_data(&sensor_data, 1, dev);    	
		bmi2_perform_accel_self_test(rslt);    	
		PDEBUG("X axes: %d, Y axes: %d, Z axes: %d\r\n",sensor_data.sens_data.acc.x, sensor_data.sens_data.acc.y, sensor_data.sens_data.acc.z );    
	}
#endif

#if defined(ACC_ONLY)
	Open_BMI323_ACC(dev);
	Close_BMI323_GYRO(dev);
#elif defined(GYRO_ONLY)
	Close_BMI323_ACC(dev);
	Open_BMI323_GYRO(dev);
#elif defined(ACC_GYRO)
	Open_BMI323_ACC(dev);
	Open_BMI323_GYRO(dev);
#endif

#if defined(FIFO_POLL)
	Open_BMI323_FIFO(dev);
#elif defined(FIFO_WM_INT)
	Open_BMI323_FIFO(dev);
	Enable_MCU_IMU_INT1_Pin(); // Change to appropriate interrupt pin TODO TODO TODO
	//Enable_MCU_IMU_INT2_Pin();
#endif

#if !defined(FIFO_WM_INT)
	Disable_MCU_IMU_INT1_Pin(); // Change to appropriate interrupt pin TODO TODO TODO
	Disable_MCU_IMU_INT2_Pin();
#endif

	return rslt;
}


void BMI323Task(void)
{
//	if(bmi323_fifo_ready == 1)
//	{
		int8_t rslt = BMI3_OK;
		struct bmi3_dev *dev;
		dev = &bmi323dev;

		/* Retrieve raw data from accel and gyro registers */
		sensor_data[0].type = BMI3_ACCEL;
		sensor_data[1].type = BMI3_GYRO;
		rslt = bmi323_get_sensor_data(sensor_data, 2, dev);
		bmi3_error_codes_print_result("bmi323_get_sensor_data", rslt);

		if(rslt == BMI323_OK)
		{
			//PDEBUG("ACCEL X : %04d , Y : %04d , Z : %04d\r\n", sensor_data[0].sens_data.acc.x, sensor_data[0].sens_data.acc.y, sensor_data[0].sens_data.acc.z);
			//PDEBUG("GYRO X  : %04d , Y : %04d , Z : %04d\r\n", sensor_data[1].sens_data.gyr.x, sensor_data[1].sens_data.gyr.y, sensor_data[1].sens_data.gyr.z);

			/* Retrieve data from temp register */
			rslt = bmi323_get_temperature_data(&temp_data, dev);
			bmi3_error_codes_print_result("bmi323_get_temperature_data", rslt);

			if(rslt == BMI323_OK)
			{
				//PDEBUG("TEMP : %04d = %d C\r\n", temp_data, (temp_data >> 9) + 23); // Need to convert to int16_t
			}
		}

		/* Put sensor data at appropriate locations in UserTxBufferFS */
		// Accel X (int16_t data)
		int16_t accel_x = sensor_data[0].sens_data.acc.x;
		UserTxBufferFS[43] = (uint8_t)((accel_x >> 8) & 0xFF);
		UserTxBufferFS[44] = (uint8_t)(accel_x & 0xFF);

		// Accel Y (int16_t data)
		int16_t accel_y = sensor_data[0].sens_data.acc.y;
		UserTxBufferFS[45] = (uint8_t)((accel_y >> 8) & 0xFF);
		UserTxBufferFS[46] = (uint8_t)(accel_y & 0xFF);

		// Accel Z (int16_t data)
		int16_t accel_z = sensor_data[0].sens_data.acc.z;
		UserTxBufferFS[47] = (uint8_t)((accel_z >> 8) & 0xFF);
		UserTxBufferFS[48] = (uint8_t)(accel_z & 0xFF);

		// Gyro X (int16_t data)
		int16_t gyr_x = sensor_data[1].sens_data.gyr.x;
		UserTxBufferFS[49] = (uint8_t)((gyr_x >> 8) & 0xFF);
		UserTxBufferFS[50] = (uint8_t)(gyr_x & 0xFF);

		// Gyro Y (int16_t data)
		int16_t gyr_y = sensor_data[1].sens_data.gyr.y;
		UserTxBufferFS[51] = (uint8_t)((gyr_y >> 8) & 0xFF);
		UserTxBufferFS[52] = (uint8_t)(gyr_y & 0xFF);

		// Gyro Z (int16_t data)
		int16_t gyr_z = sensor_data[1].sens_data.gyr.z;
		UserTxBufferFS[53] = (uint8_t)((gyr_z >> 8) & 0xFF);
		UserTxBufferFS[54] = (uint8_t)(gyr_z & 0xFF);

		// Temp (uint16_t data)
		UserTxBufferFS[55] = (uint8_t)((temp_data >> 8) & 0xFF);
		UserTxBufferFS[56] = (uint8_t)(temp_data & 0xFF);

		/* Get data based on FIFO */
//		memset(fifo_data, 0, sizeof(fifo_data));
//		fifoframe.length = 0;
//
//		/* Update FIFO structure */
//		fifoframe.data = fifo_data;
//
//		rslt = bmi323_get_fifo_length(&fifoframe.available_fifo_len, dev);
//		bmi3_error_codes_print_result("bmi323_get_fifo_length", rslt);
//
//		//fifoframe.length = (uint16_t)(fifoframe.available_fifo_len * 2) + dev->dummy_byte;
//		fifoframe.length = (uint16_t)(fifoframe.available_fifo_len * 2) + 2;
//		PDEBUG("Fifo length=%d\r\n", fifoframe.length);
//		if(fifoframe.length > 0)
//		{
//			/* Read FIFO data */
//			rslt = bmi323_read_fifo_data(&fifoframe, dev);
//			bmi3_error_codes_print_result("bmi323_read_fifo_data", rslt);
//
//			if (rslt == BMI323_OK)
//			{
//				PDEBUG("Read %d bytes from fifo\r\n", fifoframe.length);
//
//				/* Parse the FIFO data to extract accelerometer data from the FIFO buffer */
//				rslt = bmi323_extract_accel(fifo_accel_data, &fifoframe, dev);
//				PDEBUG("Parsed accelerometer data frames: %d\r\n", fifoframe.avail_fifo_accel_frames);
//				if (rslt == BMI3_OK)
//				{
//					/* Print the parsed accelerometer data from the FIFO buffer */
//					//for(idx = 0 ; idx < fifoframe.avail_fifo_accel_frames ; idx++)
//					//{
//						//PDEBUG("ACCEL[%d] X : %d , Y : %d , Z : %d\r\n", idx , fifo_accel_data[idx].x, fifo_accel_data[idx].y, fifo_accel_data[idx].z);
//					//}
//
//					PDEBUG("ACCEL X : %d , Y : %d , Z : %d\r\n" , fifo_accel_data[0].x, fifo_accel_data[0].y, fifo_accel_data[0].z);
//				}
//
//				/* Parse the FIFO data to extract gyroscope data from the FIFO buffer */
//				rslt = bmi323_extract_gyro(fifo_gyr_data, &fifoframe, dev);
//				bmi3_error_codes_print_result("bmi323_extract_gyro", rslt);
//				PDEBUG("Parsed gyroscope data frames: %d\r\n", fifoframe.avail_fifo_gyro_frames);
//				if (rslt == BMI3_OK)
//				{
//					/* Print the parsed accelerometer data from the FIFO buffer */
//					//for(idx = 0 ; idx < fifoframe.avail_fifo_gyro_frames ; idx++)
//					//{
//						//PDEBUG("GYRO[%d] X : %d , Y : %d , Z : %d\r\n", idx , fifo_gyr_data[idx].x, fifo_gyr_data[idx].y, fifo_gyr_data[idx].z);
//					//}
//
//					PDEBUG("GYRO X : %d , Y : %d , Z : %d\r\n" , fifo_gyr_data[0].x, fifo_gyr_data[0].y, fifo_gyr_data[0].z);
//				}
//
//				/* Parse the FIFO data to extrac temperature data from the FIFO buffer */
//				rslt = bmi323_extract_temperature(fifo_temp_data, &fifoframe, dev);
//				bmi3_error_codes_print_result("bmi323_extract_temperature", rslt);
//				PDEBUG("Parsed temperature data frames: %d\r\n", fifoframe.avail_fifo_temp_frames);
//				if(rslt == BMI3_OK)
//				{
//					PDEBUG("TEMP : %d = %d C\r\n", fifo_temp_data->temp_data, (fifo_temp_data->temp_data >> 9) + 23);
//				}
//			}
//		}

//		bmi323_fifo_ready = 0;
//	}
}

#endif
