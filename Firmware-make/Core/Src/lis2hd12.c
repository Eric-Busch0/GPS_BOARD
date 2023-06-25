#include <assert.h>
#include "i2c.h"
#include "lis2hd12.h"
#include "main.h"
#include "retcheck.h"

volatile uint8_t write_complete = 0;
volatile uint8_t read_complete = 0;
uint8_t wr_buf[8];

lis2hd12_ctrl_t ctrl_reg;
#define GETBIT(val, bit) ((val & (1 << bit)) >> bit)
#define GETBITMASK(val, mask, bit) ((val & (mask << bit)) >> bit)

#define LIS2HD12_I2C_ADDR (0x19 << 1)
#define LIS2HD12_CHECK_NULL(x) \
    if (x == NULL)             \
    return HAL_ERROR

static inline uint8_t lis2hd12_write(uint8_t *buf, uint32_t len)
{
    return i2c_write(LIS2HD12_I2C_ADDR, buf, len);
}
static inline uint8_t lis2hd12_read_reg(uint8_t memaddr, uint8_t *buf, uint32_t len)
{
    return i2c_mem_read(LIS2HD12_I2C_ADDR, memaddr, sizeof(uint8_t), buf, len);
}

static inline uint8_t lis2hd12_write_reg(uint8_t memaddr, uint8_t *data, uint32_t len)
{
    return i2c_mem_write(LIS2HD12_I2C_ADDR, memaddr, sizeof(uint8_t), data, len);
}
static inline uint8_t lis2hd12_write_reg_dma(uint8_t memaddr, uint8_t *data, uint32_t len)
{
    return i2c_mem_write_dma(LIS2HD12_I2C_ADDR, memaddr, sizeof(uint8_t), data, len);
}

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

    return lis2hd12_write_reg(CTRL0_REG, &reg_value, 1);
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
static void ctrl_regs_to_struct(uint8_t *regs, lis2hd12_ctrl_t *ctrl)
{

    /*
        Ctrl 1
    */
    ctrl->ctrl1.odr = (lis2hd12_odr_t)((regs[0] & (0x4 << 4)) >> 4);
    ctrl->ctrl1.lowPwrEn = regs[0] & (1 << 3);
    ctrl->ctrl1.zAxisEn = regs[0] & (1 << 2);
    ctrl->ctrl1.yAxisEN = regs[0] & (1 << 1);
    ctrl->ctrl1.xAxisEn = regs[0] & (1 << 0);

    /*
        Ctrl 2
    */
    ctrl->ctrl2.hpfMode = (lis2hd12_hpf_mode_t)((regs[1] & (0x2 << 6)) >> 6);
    ctrl->ctrl2.hpfCutoff = (regs[1] & (0x2 << 4)) >> 4;
    ctrl->ctrl2.fds = (regs[1] & (1 << 3)) >> 3;
    ctrl->ctrl2.hpClick = (regs[1] & (1 << 2)) >> 2;
    ctrl->ctrl2.hpIa2En = (regs[1] & (1 << 1)) >> 1;
    ctrl->ctrl2.hpIa1En = (regs[1] & (1 << 0));

    /*
        Ctrl 3
    */
    ctrl->ctrl3.i1Click = (regs[2] & (1 << 7)) >> 7;
    ctrl->ctrl3.i1Ia1 = (regs[2] & (1 << 6)) >> 6;
    ctrl->ctrl3.i1Ia2 = (regs[2] & (1 << 5)) >> 5;
    ctrl->ctrl3.i1Zyxda = (regs[2] & (1 << 4)) >> 4;
    ctrl->ctrl3.i1Wtm = (regs[2] & (1 << 2)) >> 2;
    ctrl->ctrl3.i1Overrun = (regs[2] & (1 << 1)) >> 1;
    /*
        Ctrl 4
    */
    ctrl->ctrl4.blockDataUpdate = GETBIT(regs[3], 7);
    ctrl->ctrl4.bigOrLittleEndian = GETBIT(regs[3], 6);
    ctrl->ctrl4.fsMode = GETBITMASK(regs[3], 0x2, 4);
    ctrl->ctrl4.operatingMode = GETBIT(regs[3], 3);
    ctrl->ctrl4.selfTestMode = GETBITMASK(regs[3], 0x2, 1);
    ctrl->ctrl4.sim = GETBIT(regs[3], 1);

    /*
        Ctrl 5
    */
    ctrl->ctrl5.boot = GETBIT(regs[4], 7);
    ctrl->ctrl5.fifoEn = GETBIT(regs[4], 6);
    ctrl->ctrl5.latchIntReq1 = GETBIT(regs[4], 3);
    ctrl->ctrl5.D4DEn1 = GETBIT(regs[4], 2);
    ctrl->ctrl5.latchIntReq2 = GETBIT(regs[4], 1);
    ctrl->ctrl5.D4DEn2 = GETBIT(regs[4], 0);

    /*
        Ctrl 6
    */

    ctrl->ctrl6.clickInt2 = GETBIT(regs[5], 7);
    ctrl->ctrl6.int1FnInt2En = GETBIT(regs[5], 6);
    ctrl->ctrl6.int2FnInt2En = GETBIT(regs[5], 5);
    ctrl->ctrl6.bootOnInt2En = GETBIT(regs[5], 4);
    ctrl->ctrl6.activityIntOnInt2En = GETBIT(regs[5], 3);
    ctrl->ctrl6.int12Polarity = GETBIT(regs[5], 1);
}
uint8_t lis2hd12_get_ctrl(lis2hd12_ctrl_t *ctrl)
{
    assert(ctrl);

    static const uint8_t CTRL_REG1 = 0x20;
    static const uint8_t NUM_CTRL_REGS = 6u;

    uint8_t regs[6] = {0};

    uint8_t ret = lis2hd12_read_reg(CTRL_REG1, regs, NUM_CTRL_REGS);

    RETCHECK(ret);

    ctrl_regs_to_struct(regs, ctrl);

    return ret;
}
static void ctrl_struct_to_regs(uint8_t *regs, lis2hd12_ctrl_t *ctrl)
{
    /*
        Ctrl 1
    */
    regs[0] = (ctrl->ctrl1.odr << 4) |
              (ctrl->ctrl1.lowPwrEn << 3) |
              (ctrl->ctrl1.zAxisEn << 2) |
              (ctrl->ctrl1.yAxisEN << 1) |
              (ctrl->ctrl1.yAxisEN << 1);

    /*
        Ctrl 2
    */
    regs[1] = (ctrl->ctrl2.hpfMode << 6) |
              (ctrl->ctrl2.hpfCutoff << 4) |
              (ctrl->ctrl2.fds << 3) |
              (ctrl->ctrl2.hpClick << 2) |
              (ctrl->ctrl2.hpIa2En << 1) |
              (ctrl->ctrl2.hpIa1En << 0);
    /*
        Ctrl 3
    */
    regs[2] = (ctrl->ctrl3.i1Click << 7) |
              (ctrl->ctrl3.i1Ia1 << 6) |
              (ctrl->ctrl3.i1Ia2 << 5) |
              (ctrl->ctrl3.i1Zyxda << 4) |
              (ctrl->ctrl3.i1Wtm << 2) |
              (ctrl->ctrl3.i1Overrun << 1);
    /*
        Ctrl 4
    */
    regs[3] = (ctrl->ctrl4.blockDataUpdate << 7) |
              (ctrl->ctrl4.bigOrLittleEndian << 6) |
              (ctrl->ctrl4.fsMode << 4) |
              (ctrl->ctrl4.operatingMode << 3) |
              (ctrl->ctrl4.selfTestMode << 1) |
              (ctrl->ctrl4.sim << 0);
    /*
        Ctrl 5
    */
    regs[4] = (ctrl->ctrl5.boot << 7) |
              (ctrl->ctrl5.fifoEn << 6) |
              (ctrl->ctrl5.latchIntReq1 << 3) |
              (ctrl->ctrl5.D4DEn1 << 2) |
              (ctrl->ctrl5.latchIntReq2 << 1) |
              (ctrl->ctrl5.D4DEn2 << 0);

    /*
        Ctrl 6
    */
    regs[5] = (ctrl->ctrl6.clickInt2 << 7) |
              (ctrl->ctrl6.int1FnInt2En << 6) |
              (ctrl->ctrl6.int2FnInt2En << 5) |
              (ctrl->ctrl6.bootOnInt2En << 4) |
              (ctrl->ctrl6.activityIntOnInt2En << 3) |
              (ctrl->ctrl6.int12Polarity << 1);
}
uint8_t lis2hd12_set_ctrl(lis2hd12_ctrl_t *ctrl)
{

    static const uint8_t CTRL_REG1 = 0x20;
    static const uint8_t NUM_CTRL_REGS = 6u;

    ctrl_struct_to_regs(wr_buf, ctrl);

    return lis2hd12_write_reg_dma(CTRL_REG1, wr_buf, NUM_CTRL_REGS);
}

uint8_t lis2hd12_enable_temp_sensor(void)
{
    static const uint8_t ENABLE = (0x3 << 6);
    static const uint8_t TEMP_CFG_REG = 0x1F;
    return lis2hd12_write_reg(TEMP_CFG_REG, &ENABLE, 1);
}
uint8_t lis2hd12_disable_temp_sensor(void)
{
    static const uint8_t DISABLE = 0x00;
    static const uint8_t TEMP_CFG_REG = 0x1F;
    return lis2hd12_write_reg(TEMP_CFG_REG, (uint8_t)&DISABLE, 1);
}
uint8_t lis2hd12_temp_sensor_is_enabled(bool *is_enabled)
{
    static const uint8_t TEMP_CFG_REG = 0x1F;
    
    return lis2hd12_read_reg(TEMP_CFG_REG, (uint8_t*)is_enabled, sizeof(uint8_t));
}
uint8_t lis2hd12_get_interrupt_ref(uint8_t *interrupt_ref)
{
    static const uint8_t REF_REG = 0x26;

    return lis2hd12_read_reg(REF_REG, interrupt_ref, sizeof(uint8_t));
}
uint8_t lis2hd12_set_interrupt_ref(uint8_t interrupt_ref)
{
    static const uint8_t REF_REG = 0x26;

    return lis2hd12_write_reg(REF_REG, &interrupt_ref, 1);
}
uint8_t lis2hd12_get_status(uint8_t *status)
{
    static const uint8_t STATUS_REG = 0x26;

    return lis2hd12_read_reg(STATUS_REG, status, sizeof(uint8_t));
}

uint8_t lis2hd12_get_acc_xyz(lis2dh12_acc_xyz *xyz)
{
    static const uint8_t X_REG = 0x29;
    uint8_t xyz_regs[6];
    uint8_t ret = lis2hd12_read_reg(X_REG, xyz_regs, 3 * sizeof(uint16_t));

    xyz->x = xyz_regs[1] << 8 | xyz_regs[0];
    xyz->y = xyz_regs[3] << 8 | xyz_regs[2];
    xyz->z = xyz_regs[5] << 8 | xyz_regs[4];
    
    return ret;
}
uint8_t lis2hd12_get_fifo_ctrl(uint8_t *fifo_ctrl)
{
    static const uint8_t FIFO_CTRL_REG = 0x2E;

    return lis2hd12_read_reg(FIFO_CTRL_REG, fifo_ctrl, sizeof(uint8_t));
}
uint8_t lis2hd12_set_fifo_ctrl(uint8_t fifo_ctrl)
{
    static const uint8_t FIFO_CTRL_REG = 0x2E;

    return lis2hd12_write_reg(FIFO_CTRL_REG, &fifo_ctrl, 1);
}
uint8_t lis2hd12_get_fifo_src(uint8_t *fifo_src)
{
    static const uint8_t FIFO_SRC_REG = 0x2F;

    return lis2hd12_read_reg(FIFO_SRC_REG, fifo_src, sizeof(uint8_t));
}

uint8_t lis2hd12_get_int1_cfg(uint8_t int1_cfg)
{
    static const uint8_t INT1_CFG_REG = 0x30;

    return lis2hd12_read_reg(INT1_CFG_REG, &int1_cfg, sizeof(uint8_t));
}
uint8_t lis2hd12_set_int1_cfg(uint8_t *int1_cfg)
{
    static const uint8_t INT1_CFG_REG = 0x30;

    return lis2hd12_write_reg(INT1_CFG_REG, int1_cfg, 1);
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
uint8_t lis2hd12_set_int1_ths(uint8_t int1_ths)
{
    static const uint8_t INT1_THS_REG = 0x32;

    return lis2hd12_write_reg(INT1_THS_REG, &int1_ths, 1);
}
uint8_t lis2hd12_get_int1_duration(uint8_t *int1_duration)
{
    static const uint8_t INT1_THS_DURATION = 0x33;

    return lis2hd12_read_reg(INT1_THS_DURATION, int1_duration, sizeof(uint8_t));
}
uint8_t lis2hd12_set_int1_duration(uint8_t int1_duration)
{
    static const uint8_t INT1_THS_DURATION = 0x33;

    return lis2hd12_write_reg(INT1_THS_DURATION, &int1_duration, 1);
}

uint8_t lis2hd12_get_int2_cfg(uint8_t *int2_cfg)
{
    static const uint8_t INT2_CFG_REG = 0x34;

    return lis2hd12_read_reg(INT2_CFG_REG, int2_cfg, sizeof(uint8_t));
}
uint8_t lis2hd12_set_int2_cfg(uint8_t int2_cfg)
{
    static const uint8_t INT2_CFG_REG = 0x34;

    return lis2hd12_write_reg(INT2_CFG_REG, &int2_cfg, 1);
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
uint8_t lis2hd12_set_int2_ths(uint8_t int2_ths)
{
    static const uint8_t INT2_THS_REG = 0x36;

    return lis2hd12_write_reg(INT2_THS_REG, &int2_ths, 1);
}

uint8_t lis2hd12_get_int2_duration(uint8_t *int2_duration)
{
    static const uint8_t INT2_THS_DURATION = 0x37;

    return lis2hd12_read_reg(INT2_THS_DURATION, int2_duration, sizeof(uint8_t));
}
uint8_t lis2hd12_set_int2_duration(uint8_t int2_duration)
{
    static const uint8_t INT1_THS_DURATION = 0x37;

    return lis2hd12_write_reg(INT1_THS_DURATION, &int2_duration, 1);
}

uint8_t lis2hd12_get_click_cfg(uint8_t *click_cfg)
{
    static const uint8_t CLICK_CFG_REG = 0x38;

    return lis2hd12_read_reg(CLICK_CFG_REG, click_cfg, sizeof(uint8_t));
}
uint8_t lis2hd12_set_click_cfg(uint8_t click_cfg)
{
    static const uint8_t CLICK_CFG_REG = 0x38;

    return lis2hd12_write_reg(CLICK_CFG_REG, &click_cfg, 1);
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

    return lis2hd12_write_reg(CLICK_THS_REG, click_ths, 1);
}
uint8_t lis2hd12_get_time_limit(uint8_t *time_limit)
{
    static const uint8_t TIME_LIMIT_REG = 0x3B;

    return lis2hd12_read_reg(TIME_LIMIT_REG, time_limit, sizeof(uint8_t));
}
uint8_t lis2hd12_set_time_limit(uint8_t time_limit)
{
    static const uint8_t TIME_LIMIT_REG = 0x3B;

    return lis2hd12_write_reg(TIME_LIMIT_REG, &time_limit, 1);
}
uint8_t lis2hd12_get_time_latency(uint8_t *time_latency)
{
    static const uint8_t TIME_LATENCY_REG = 0x3C;

    return lis2hd12_read_reg(TIME_LATENCY_REG, time_latency, sizeof(uint8_t));
}
uint8_t lis2hd12_set_time_latency(uint8_t time_latency)
{
    static const uint8_t TIME_LATENCY_REG = 0x3C;

    return lis2hd12_write_reg(TIME_LATENCY_REG, &time_latency, 1);
}
uint8_t lis2hd12_get_time_window(uint8_t *time_window)
{
    static const uint8_t TIME_WINDOW_REG = 0x3D;

    return lis2hd12_read_reg(TIME_WINDOW_REG, time_window, sizeof(uint8_t));
}
uint8_t lis2hd12_set_time_window(uint8_t time_window)
{
    static const uint8_t TIME_WINDOW_REG = 0x3D;

    return lis2hd12_write_reg(TIME_WINDOW_REG, &time_window, 1);
}

uint8_t lis2hd12_get_act_ths(uint8_t *act_ths)
{
    static const uint8_t ACT_THS_REG = 0x3E;

    return lis2hd12_read_reg(ACT_THS_REG, act_ths, sizeof(uint8_t));
}
uint8_t lis2hd12_set_act_ths(uint8_t act_ths)
{
    static const uint8_t ACT_THS_REG = 0x3E;

    return lis2hd12_write_reg(ACT_THS_REG, &act_ths, 1);
}
uint8_t lis2hd12_get_act_dur(uint8_t *act_dur)
{
    static const uint8_t ACT_DUR_REG = 0x3F;

    return lis2hd12_read_reg(ACT_DUR_REG, act_dur, sizeof(uint8_t));
}
uint8_t lis2hd12_set_act_dur(uint8_t act_dur)
{
    static const uint8_t ACT_DUR_REG = 0x3F;

    return lis2hd12_write_reg(ACT_DUR_REG, &act_dur, 1);
}