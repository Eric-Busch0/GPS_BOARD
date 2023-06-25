#include <assert.h>
#include "i2c.h"
#include "lis2hd12.h"
#include "main.h"
#include "retcheck.h"

volatile uint8_t write_complete = 0;
volatile uint8_t read_complete = 0;

#define LIS2HD12_I2C_ADDR (0x19 << 1)
#define LIS2HD12_CHECK_NULL(x)  if(x == NULL) return HAL_ERROR

static inline uint8_t lis2hd12_write(uint8_t *buf, uint32_t len)
{
    return i2c_write(LIS2HD12_I2C_ADDR, buf, len);
}
static inline uint8_t lis2hd12_read_reg(uint8_t memaddr, uint8_t *buf, uint32_t len)
{
    return i2c_mem_read(LIS2HD12_I2C_ADDR, memaddr, sizeof(uint8_t), buf, len);
}
static inline uint8_t lis2hd12_write_reg(uint8_t reg_addr, uint8_t data)
{
    return 0;
}

void lis2hd12_init(void);

uint8_t lis2hd12_who_am_i(uint8_t *who_am_i)
{

    static const uint8_t REG_ADDR = 0x0f;

    return lis2hd12_read_reg(REG_ADDR, who_am_i, 1);
}
uint8_t lis2hd12_get_status_reg(uint8_t *status_reg)
{
    assert(status_reg);

    return lis2hd12_read_reg(0x07, status_reg, 1);
}
uint8_t lis2hd12_get_temp(uint16_t *temp)
{
    assert(temp);

    static const uint8_t TEMP_REG = 0x0C;

    return lis2hd12_read_reg(TEMP_REG, temp, sizeof(uint16_t));
}
uint8_t lis2hd12_disconnect_sdo_pullup(bool disconnect)
{
    static const uint8_t CTRL0_REG = 0x1E;
    static const uint8_t PU_DISCONNECT = 0x90;
    static const uint8_t PU_CONNECT = 0x10;

    const uint8_t reg_value = disconnect ? PU_DISCONNECT : PU_CONNECT;

    return lis2hd12_write_reg(CTRL0_REG, reg_value);

}
uint8_t lis2hd12_sdo_pullup_is_disconnected(bool *is_disconnected)
{
    static const uint8_t CTRL0_REG = 0x1E;
    static const uint8_t PU_DISCONNECT = 0x90;

    uint8_t reg_value;
    const uint8_t ret = lis2hd12_read_reg(CTRL0_REG, &reg_value, 1);

    *is_disconnected = (reg_value == PU_DISCONNECT);

    return ret;

}

uint8_t lis2hd12_get_ctrl(lis2dh12_ctrl_t *ctrl)
{
    return 0;
}


uint8_t lis2hd12_set_ctrl(lis2dh12_ctrl_t *ctrl)
{


    return 0;
}

uint8_t lis2hd12_enable_temp_sensor(void)
{
    static const uint8_t ENABLE = (0x3 << 6);
    static const uint8_t TEMP_CFG_REG = 0x1F;
    lis2hd12_write_reg(TEMP_CFG_REG, ENABLE);

}
uint8_t lis2hd12_disable_temp_sensor(void)
{
    static const uint8_t DISABLE = 0x00;
    static const uint8_t TEMP_CFG_REG = 0x1F;
    lis2hd12_write_reg(TEMP_CFG_REG, ENABLE);


}
uint8_t lis2hd12_temp_sensor_is_enabled(bool *is_enabled)
{
    static const uint8_t TEMP_CFG_REG = 0x1F;
    lis2hd12_read_reg(TEMP_CFG_REG, is_enabled, sizeof(uint8_t));
}
uint8_t lis2hd12_get_interrupt_ref(uint8_t *interrupt_ref)
{
    static const uint8_t REF_REG = 0x26;

    return lis2hd12_read_reg(REF_REG, interrupt_ref, sizeof(uint8_t));
}
uint8_t lis2hd12_set_interrupt_ref(uint8_t *interrupt_ref)
{
    static const uint8_t REF_REG = 0x26;

    return lis2hd12_write_reg(REF_REG, interrupt_ref);
}
uint8_t lis2hd12_get_status(uint8_t *status)
{
    static const uint8_t STATUS_REG = 0x26;

    return lis2hd12_read_reg(STATUS_REG, status, sizeof(uint8_t));
}

uint8_t lis2hd12_get_acc_xyz(lis2dh12_acc_xyz *xyz)
{
    static const uint8_t X_REG = 0x29;

    return lis2hd12_read_reg(X_REG, xyz, 3 * sizeof(uint16_t));
}
uint8_t lis2hd12_get_fifo_ctrl(uint8_t *fifo_ctrl)
{
    static const uint8_t FIFO_CTRL_REG = 0x2E;

    return lis2hd12_read_reg(FIFO_CTRL_REG, fifo_ctrl, sizeof(uint8_t));
}
uint8_t lis2hd12_set_fifo_ctrl(uint8_t *fifo_ctrl)
{
    static const uint8_t FIFO_CTRL_REG = 0x2E;

    return lis2hd12_write_reg(FIFO_CTRL_REG, fifo_ctrl);
}
uint8_t lis2hd12_get_fifo_src(uint8_t *fifo_src)
{
    static const uint8_t FIFO_SRC_REG = 0x2F;

    return lis2hd12_read_reg(FIFO_SRC_REG, fifo_src, sizeof(uint8_t));
}

uint8_t lis2hd12_get_int1_cfg(uint8_t *int1_cfg)
{
    static const uint8_t INT1_CFG_REG = 0x30;

    return lis2hd12_read_reg(INT1_CFG_REG, int1_cfg, sizeof(uint8_t));
}
uint8_t lis2hd12_set_int1_cfg(uint8_t *int1_cfg)
{
    static const uint8_t INT1_CFG_REG = 0x30;

    return lis2hd12_write_reg(INT1_CFG_REG, int1_cfg);
}
uint8_t lis2hd12_get_int1_src(uint8_t *int1_src)
{
    static const uint8_t INT1_SRC_REG = 0x31;

    return lis2hd12_read_reg(INT1_SRC_REG, int1_src, sizeof(uint8_t));
}


uint8_t lis2hd12_get_int1_ths(uint8_t *int1_ths)
{
    static const uint8_t INT1_THS_REG = 0x32;

    return lis2hd12_read_reg(INT1_THS_REG, int1_ths, sizeof(uint8_t));
}
uint8_t lis2hd12_set_int1_ths(uint8_t *int1_ths)
{
    static const uint8_t INT1_THS_REG = 0x32;

    return lis2hd12_write_reg(INT1_THS_REG, int1_ths);
}
uint8_t lis2hd12_get_int1_duration(uint8_t *int1_duration)
{
    static const uint8_t INT1_THS_DURATION = 0x33;

    return lis2hd12_read_reg(INT1_THS_DURATION, int1_duration, sizeof(uint8_t));
}
uint8_t lis2hd12_set_int1_duration(uint8_t *int1_duration)
{
    static const uint8_t INT1_THS_DURATION = 0x33;

    return lis2hd12_write_reg(INT1_THS_DURATION, int1_duration);
}


uint8_t lis2hd12_get_int2_cfg(uint8_t *int2_cfg)
{
    static const uint8_t INT2_CFG_REG = 0x34;

    return lis2hd12_read_reg(INT2_CFG_REG, int2_cfg, sizeof(uint8_t));
}
uint8_t lis2hd12_set_int2_cfg(uint8_t *int2_cfg)
{
    static const uint8_t INT2_CFG_REG = 0x34;

    return lis2hd12_write_reg(INT2_CFG_REG, int2_cfg);
}
uint8_t lis2hd12_get_int2_src(uint8_t *int2_src)
{
    static const uint8_t INT2_SRC_REG = 0x35;

    return lis2hd12_read_reg(INT2_SRC_REG, int2_src, sizeof(uint8_t));
}

uint8_t lis2hd12_get_int2_ths(uint8_t *int2_ths)
{
    static const uint8_t INT2_THS_REG = 0x36;

    return lis2hd12_read_reg(INT2_THS_REG, int2_ths, sizeof(uint8_t));
}
uint8_t lis2hd12_set_int2_ths(uint8_t *int2_ths)
{
    static const uint8_t INT2_THS_REG = 0x36;

    return lis2hd12_write_reg(INT2_THS_REG, int2_ths);
}

uint8_t lis2hd12_get_int2_duration(uint8_t *int2_duration)
{
    static const uint8_t INT2_THS_DURATION = 0x37;

    return lis2hd12_read_reg(INT2_THS_DURATION, int2_duration, sizeof(uint8_t));
}
uint8_t lis2hd12_set_int2_duration(uint8_t *int2_duration)
{
    static const uint8_t INT1_THS_DURATION = 0x37;

    return lis2hd12_write_reg(INT1_THS_DURATION, int2_duration);
}

uint8_t lis2hd12_get_click_cfg(uint8_t *click_cfg)
{
    static const uint8_t CLICK_CFG_REG = 0x38;

    return lis2hd12_read_reg(CLICK_CFG_REG, click_cfg, sizeof(uint8_t));
}
uint8_t lis2hd12_set_click_cfg(uint8_t *click_cfg)
{
    static const uint8_t CLICK_CFG_REG = 0x38;

    return lis2hd12_write_reg(CLICK_CFG_REG, click_cfg);
}

uint8_t lis2hd12_get_click_src(uint8_t *click_src)
{
    static const uint8_t CLICK_SRC_REG = 0x39;

    return lis2hd12_read_reg(CLICK_SRC_REG, click_src, sizeof(uint8_t));
}

uint8_t lis2hd12_get_click_ths(uint8_t *click_ths)
{
    static const uint8_t CLICK_THS_REG = 0x3A;

    return lis2hd12_read_reg(CLICK_THS_REG, click_ths, sizeof(uint8_t));
}
uint8_t lis2hd12_set_click_ths(uint8_t *click_ths)
{
    static const uint8_t CLICK_THS_REG = 0x3A;

    return lis2hd12_write_reg(CLICK_THS_REG, click_ths);
}
uint8_t lis2hd12_get_time_limit(uint8_t *time_limit)
{
    static const uint8_t TIME_LIMIT_REG = 0x3B;

    return lis2hd12_read_reg(TIME_LIMIT_REG, time_limit, sizeof(uint8_t));
}
uint8_t lis2hd12_set_time_limit(uint8_t *time_limit)
{
    static const uint8_t TIME_LIMIT_REG = 0x3B;

    return lis2hd12_write_reg(TIME_LIMIT_REG, time_limit);
}
uint8_t lis2hd12_get_time_latency(uint8_t *time_latency)
{
    static const uint8_t TIME_LATENCY_REG = 0x3C;

    return lis2hd12_read_reg(TIME_LATENCY_REG, time_latency, sizeof(uint8_t));
}
uint8_t lis2hd12_set_time_latency(uint8_t *time_latency)
{
    static const uint8_t TIME_LATENCY_REG = 0x3C;

    return lis2hd12_write_reg(TIME_LATENCY_REG, time_latency);
}
uint8_t lis2hd12_get_time_window(uint8_t *time_window)
{
    static const uint8_t TIME_WINDOW_REG = 0x3D;

    return lis2hd12_read_reg(TIME_WINDOW_REG, time_window, sizeof(uint8_t));
}
uint8_t lis2hd12_set_time_window(uint8_t *time_window)
{
    static const uint8_t TIME_WINDOW_REG = 0x3D;

    return lis2hd12_write_reg(TIME_WINDOW_REG, time_window);
}

uint8_t lis2hd12_get_act_ths(uint8_t *act_ths)
{
    static const uint8_t ACT_THS_REG = 0x3E;

    return lis2hd12_read_reg(ACT_THS_REG, act_ths, sizeof(uint8_t));
}
uint8_t lis2hd12_set_act_ths(uint8_t *act_ths)
{
    static const uint8_t ACT_THS_REG = 0x3E;

    return lis2hd12_write_reg(ACT_THS_REG, act_ths);
}
uint8_t lis2hd12_get_act_dur(uint8_t *act_dur)
{
    static const uint8_t ACT_DUR_REG = 0x3F;

    return lis2hd12_read_reg(ACT_DUR_REG, act_dur, sizeof(uint8_t));
}
uint8_t lis2hd12_set_act_dur(uint8_t *act_dur)
{
    static const uint8_t ACT_DUR_REG = 0x3F;

    return lis2hd12_write_reg(ACT_DUR_REG, act_dur);
}