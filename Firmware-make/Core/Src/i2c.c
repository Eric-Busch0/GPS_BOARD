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
uint8_t i2c_write_dma(uint8_t addr, uint8_t* buf, uint32_t len)
{
    return HAL_I2C_Master_Transmit_DMA(&hi2c3, addr, buf, len);
}
uint8_t i2c_read_dma(uint8_t addr, uint8_t* buf, uint32_t len)
{
    return HAL_I2C_Master_Receive_DMA(&hi2c3, addr, buf, len);
}

uint8_t i2c_device_acks(uint8_t addr, uint32_t attempts)
{
    return HAL_I2C_IsDeviceReady(&hi2c3, addr, attempts, HAL_MAX_DELAY);
}
void HAL_I2C_MasterTxCpltCallback (I2C_HandleTypeDef * hi2c)
{
  while(1);
}