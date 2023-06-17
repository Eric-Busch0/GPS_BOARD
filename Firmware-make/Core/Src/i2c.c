#include "i2c.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c3;
extern DMA_HandleTypeDef hdma_i2c3_rx;
extern DMA_HandleTypeDef hdma_i2c3_tx;

uint8_t i2c_write(uint8_t addr, uint8_t* buf, uint32_t len)
{
    return HAL_I2C_Master_Transmit(&hi2c3, addr,buf, len, HAL_MAX_DELAY);
}
uint8_t i2c_read(uint8_t addr, uint8_t* buf, uint32_t len)
{
    return HAL_I2C_Master_Receive(&hi2c3, addr, buf, len, HAL_MAX_DELAY);
}

