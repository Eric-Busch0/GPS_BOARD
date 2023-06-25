#ifndef LIS2HD12_H
#define LIS2HD12_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define LIS2HD12_WHO_AM_I 0x33

#define LIS2HD12_STATUS_Y_AVA (1 << 0)
#define LIS2HD12_STATUS_Z_AVA (1 << 1)
#define LIS2HD12_STATUS_XYZ_AVA (1 << 2)
#define LIS2HD12_STATUS_XYZ_OVR (1 << 3)
#define LIS2HD12_STATUS_X_OVR (1 << 4)
#define LIS2HD12_STATUS_Y_OVR (1 << 5)
#define LIS2HD12_STATUS_Z_OVR (1 << 6)
#define LIS2HD12_STATUS_XYZ_OVR (1 << 7)

    typedef enum
    {
        LIS2HD12_ODR_PWR_DOWN,
        LIS2HD12_ODR_PWR_1HZ,         // 1Hz All Modes
        LIS2HD12_ODR_PWR_10HZ,        // 10Hz All Modes
        LIS2HD12_ODR_PWR_25HZ,        // 25Hz All Modes
        LIS2HD12_ODR_PWR_50HZ,        // 50Hz All Modes
        LIS2HD12_ODR_PWR_100HZ,       // 100Hz All Modes
        LIS2HD12_ODR_PWR_200HZ,       // 200Hz All Modes
        LIS2HD12_ODR_PWR_400HZ,       // 400Hz All Modes
        LIS2HD12_ODR_PWR_1_62KHZ,     // 1.62kHz All Modes
        LIS2HD12_ODR_PWR_MULTI = 0x9, // 1.344 kHz HR/Normal or 5.376 kHz Low Power Mode
    } lis2hd12_odr_t;

    typedef enum
    {
        LIS2HD12_HPF_NORMAL,
        LIS2HD12_HPF_REF_SIG_FILTERING,
        LIS2HD12_HPF_AUTO_RST_ON_INT = 0x3,
    } lis2hd12_hpf_mode_t;
    typedef enum
    {
        LIS2HD12_FS_2G,
        LIS2HD12_FS_4G,
        LIS2HD12_FS_8G,
        LIS2HD12_FS_16G,
    } lis2hd12_fs_mode_t;
    typedef enum
    {
        LIS2HD12_MODE_HR,
        LIS2HD12_MODE_NORMAL,
        LIS2HD12_MODE_LOW_PWR,
    } lis2hd12_operating_mode_t;

    typedef enum
    {
        LI2HD12_ST_DISABLE,
        LI2HD12_ST_0,
        LI2HD12_ST_1,
    } lis2hd12_self_test_t;

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
            bool hpClick;
            bool hpIa2En, hpIa1En;
        } ctrl2;

        struct
        {
            bool i1Click;
            bool i1Ia1, i1Ia2;
            bool i1Zyxda;
            bool i1Wtm;
            bool i1Overrun;
        } ctrl3;

        struct
        {
            bool blockDataUpdate;
            uint8_t bigOrLittleEndian;
            lis2hd12_fs_mode_t fsMode;
            lis2hd12_operating_mode_t operatingMode;
            lis2hd12_self_test_t selfTestMode;
            uint8_t sim;
        } ctrl4;

        struct
        {
            uint8_t boot;
            uint8_t fifoEn;
            uint8_t latchIntReq1;
            uint8_t D4DEn1;
            uint8_t latchIntReq2;
            uint8_t D4DEn2;
        } ctrl5;

        struct
        {
            bool clickInt2;
            bool int1FnInt2En;
            bool int2FnInt2En;
            bool bootOnInt2En;
            bool activityIntOnInt2En;
            uint8_t int12Polarity;
        } ctrl6;

    } lis2hd12_ctrl_t;
    typedef struct
    {
        uint16_t x, y, z;
    } lis2dh12_acc_xyz;

    uint8_t lis2hd12_who_am_i(uint8_t *who_am_i);
    uint8_t lis2hd12_get_status_reg(uint8_t *status_reg);
    uint8_t lis2hd12_get_temp(uint16_t *temp);
    uint8_t lis2hd12_disconnect_sdo_pullup(bool disconnect);
    uint8_t lis2hd12_sdo_pullup_is_disconnected(bool *is_disconnected);
    uint8_t lis2hd12_get_ctrl(lis2hd12_ctrl_t *ctrl);
    uint8_t lis2hd12_set_ctrl(lis2hd12_ctrl_t *ctrl);
    uint8_t lis2hd12_enable_temp_sensor(void);
    uint8_t lis2hd12_disable_temp_sensor(void);
    uint8_t lis2hd12_temp_sensor_is_enabled(bool *is_enabled);
    uint8_t lis2hd12_get_interrupt_ref(uint8_t *interrupt_ref);
    uint8_t lis2hd12_set_interrupt_ref(uint8_t interrupt_ref);
    uint8_t lis2hd12_get_status(uint8_t *status);
    uint8_t lis2hd12_get_acc_xyz(lis2dh12_acc_xyz *xyz);
    uint8_t lis2hd12_get_fifo_ctrl(uint8_t *fifo_ctrl);
    uint8_t lis2hd12_set_fifo_ctrl(uint8_t fifo_ctrl);
    uint8_t lis2hd12_get_fifo_src(uint8_t *fifo_src);
    uint8_t lis2hd12_get_int1_cfg(uint8_t int1_cfg);
    uint8_t lis2hd12_set_int1_cfg(uint8_t *int1_cfg);
    uint8_t lis2hd12_get_int1_src(uint8_t *int1_src);
    uint8_t lis2hd12_get_int1_ths(uint8_t *int1_ths);
    uint8_t lis2hd12_set_int1_ths(uint8_t int1_ths);
    uint8_t lis2hd12_get_int1_duration(uint8_t *int1_duration);
    uint8_t lis2hd12_set_int1_duration(uint8_t int1_duration);
    uint8_t lis2hd12_get_int2_cfg(uint8_t *int2_cfg);
    uint8_t lis2hd12_set_int2_cfg(uint8_t int2_cfg);
    uint8_t lis2hd12_get_int2_src(uint8_t *int2_src);
    uint8_t lis2hd12_get_int2_ths(uint8_t *int2_ths);
    uint8_t lis2hd12_set_int2_ths(uint8_t int2_ths);
    uint8_t lis2hd12_get_int2_duration(uint8_t *int2_duration);
    uint8_t lis2hd12_set_int2_duration(uint8_t int2_duration);
    uint8_t lis2hd12_get_click_cfg(uint8_t *click_cfg);
    uint8_t lis2hd12_set_click_cfg(uint8_t click_cfg);
    uint8_t lis2hd12_get_click_src(uint8_t *click_src);
    uint8_t lis2hd12_get_click_ths(uint8_t *click_ths);
    uint8_t lis2hd12_set_click_ths(uint8_t *click_ths);
    uint8_t lis2hd12_get_time_limit(uint8_t *time_limit);
    uint8_t lis2hd12_set_time_limit(uint8_t time_limit);
    uint8_t lis2hd12_get_time_latency(uint8_t *time_latency);
    uint8_t lis2hd12_set_time_latency(uint8_t time_latency);
    uint8_t lis2hd12_get_time_window(uint8_t *time_window);
    uint8_t lis2hd12_set_time_window(uint8_t time_window);
    uint8_t lis2hd12_get_act_ths(uint8_t *act_ths);
    uint8_t lis2hd12_set_act_ths(uint8_t act_ths);
    uint8_t lis2hd12_get_act_dur(uint8_t *act_dur);
    uint8_t lis2hd12_set_act_dur(uint8_t act_dur);


#ifdef __cplusplus
}
#endif

#endif