#ifndef USER_DEFINE_H__
#define USER_DEFINE_H__


/* Begin ADC defines */

/* Begin IMU defines */
#define DEBUG_EN
#define USE_SHUTTLE30
#define USE_BMI323
//#define ACC_SELFTEST
//#define GYRO_SELFTEST
//#define ACC_GYRO_SELFTEST
//#define FOC
//#define CRT
//#define ACC_ONLY
//#define GYRO_ONLY
#define ACC_GYRO
#if defined(ACC_ONLY) || defined(ACC_GYRO)
#define STEP_COUNTER
#endif
/*Only support to select one*/
//#define FIFO_POLL
//#define FIFO_WM_INT // <== Uncomment to get FIFO data

#endif
