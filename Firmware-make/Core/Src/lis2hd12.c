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
static inline uint8_t lis2hd12_read(uint8_t *buf, uint32_t len)
{
    return i2c_read(LIS2HD12_I2C_ADDR, buf, len);
}
static inline uint8_t lis2hd12_write_reg(uint8_t reg_addr, uint8_t data)
{
    return 0;
}
static inline uint8_t lis2hd12_read_reg(uint8_t reg_addr, uint8_t *data)
{
    HAL_StatusTypeDef ret =  lis2hd12_write(&reg_addr, 1);

    if(ret != HAL_OK)
    {
        return ret;
    }


    return lis2hd12_read(data, 1);

}
void lis2hd12_init(void);
uint8_t lis2hd12_who_am_i(uint8_t *who_am_i)
{
    static const uint8_t reg_addr = 0x0f;

    return lis2hd12_read_reg(reg_addr, who_am_i);
}