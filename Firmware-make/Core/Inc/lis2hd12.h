#ifndef LIS2HD12_H
#define LIS2HD12_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define LISS2HD12_WHO_AM_I 0x33

    typedef enum
    {
        LIS2HD12_ODR_PWR_DOWN,
        LIS2HD12_ODR_PWR_1HZ,            // 1Hz All Modes
        LIS2HD12_ODR_PWR_10HZ,           // 10Hz All Modes
        LIS2HD12_ODR_PWR_25HZ,           // 25Hz All Modes
        LIS2HD12_ODR_PWR_50HZ,           // 50Hz All Modes
        LIS2HD12_ODR_PWR_100HZ,          // 100Hz All Modes
        LIS2HD12_ODR_PWR_200HZ,          // 200Hz All Modes
        LIS2HD12_ODR_PWR_400HZ,          // 400Hz All Modes
        LIS2HD12_ODR_PWR_1_62KHZ,        // 1.62kHz All Modes
        LIS2HD12_ODR_PWR_MULTI = 0x9, // 1.344 kHz HR/Normal or 5.376 kHz Low Power Mode
    } lis2hd12_odr_t;

    typedef enum
    {
        LIS2HD12_NORMAL,
        LIS2HD12_REF_SIG_FILTERING,
        LIS2HD12_AUTO_RST_ON_INT = 0x3,
    } lis2hd12_hpf_mode_t;

    typedef struct
    {
        struct
        {
            lis2hd12_odr_t odr;
            bool lowPwrEn;
            bool zAxisEn, xAxisEn, yAxisEN;
        } ctrl1;
        struct
        {
            lis2hd12_hpf_mode_t hpfMode;
            uint8_t hpfCutoff;
            uint8_t fds; // Filtered data selection
            uint8_t hpClick;
            uint8_t hpIa2, hpIa1;
        } ctrl2;

        struct{
            uint8_t i1Click;
            uint8_t i1Ia1, i1Ia2;
            uint8_t i1Zyxda;
            uint8_t i1Wtm;
            uint8_t i1Overrun;
        }ctrl3;

        struct 
        {
            uint8_t blockDataUpdate;
            uint8_t bigOrLittleEndian;
            uint8_t operatingMode;
            uint8_t selfTestEnable;
            uint8_t sim;
        }ctrl4;
        
        struct {
            uint8_t boot;
            uint8_t fifoEn;
            uint8_t latchIntReq1;
            uint8_t D4DEn1;
            uint8_t latchIntReq2;
            uint8_t D4DEn2;
        }ctrl5;

        struct 
        {
            uint8_t clickInt2;
            uint8_t int1FnInt2En;
            uint8_t int2FnInt2En;
            uint8_t bootOnInt2En;
            uint8_t activityIntOnInt2En;
            uint8_t int12Polarity;
        }ctrl6;
        
    } lis2dh12_ctrl_t;
    typedef struct
    {
        uint16_t x, y, z;
    } lis2dh12_acc_xyz;

    // Place c code here
    void lis2hd12_init(void);
    uint8_t lis2hd12_who_am_i(uint8_t *who_am_i);
    uint8_t lis2hd12_get_ctrl(lis2dh12_ctrl_t *ctrl);

#ifdef __cplusplus
}
#endif

#endif