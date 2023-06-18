#include "i2c.h"
#include "lis2hd12.h"
#include "main.h"

volatile uint8_t write_complete = 0;
volatile uint8_t read_complete = 0;

#define LIS2HD12_I2C_ADDR (0x19 << 1)

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