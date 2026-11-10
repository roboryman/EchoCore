#ifndef BMI323_H__
#define BMI323_H__

#include <stdint.h>
#include "user_define.h"
#include "bmi3_defs.h"


/*! Macro that defines read write length */
#define READ_WRITE_LEN     UINT8_C(32)
/*! Earth's gravity in m/s^2 */
#define GRAVITY_EARTH  (9.80665f)


extern volatile uint8_t imu_int1_flag;
extern volatile uint8_t imu_int2_flag;
extern volatile uint16_t bmi323_fifo_ready;


void bmi3_error_codes_print_result(const char api_name[], int8_t rslt);

int8_t Open_BMI323_ACC(struct bmi3_dev *dev);
int8_t Close_BMI323_ACC(struct bmi3_dev *dev);
int8_t Open_BMI323_GYRO(struct bmi3_dev *dev);
int8_t Close_BMI323_GYRO(struct bmi3_dev *dev);

#if defined(FIFO_POLL) || defined(FIFO_WM_INT)
int8_t Open_BMI323_FIFO(struct bmi3_dev *dev);
int8_t Close_BMI323_FIFO(struct bmi3_dev *dev);
#endif

void BMI323_Print_ALLRegs(void);
int8_t Init_BMI323(void);
void BMI323Task(void);


#endif
