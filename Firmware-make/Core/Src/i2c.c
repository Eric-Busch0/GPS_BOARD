#include <string.h>
#include "i2c.h"
#include "uart_printf.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c3;
extern DMA_HandleTypeDef hdma_i2c3_rx;
extern DMA_HandleTypeDef hdma_i2c3_tx;

volatile struct{
    uint8_t txCplt;
    uint8_t rxCplt;
    uint8_t txMemCplt;
    uint8_t rxMemCplt;
}dmaStatusCb;

uint8_t i2c_write(uint8_t addr, uint8_t *buf, uint32_t len)
{
    return HAL_I2C_Master_Transmit(&hi2c3, addr, buf, len, HAL_MAX_DELAY);
}

uint8_t i2c_read(uint8_t addr, uint8_t *buf, uint32_t len)
{
    return HAL_I2C_Master_Receive(&hi2c3, addr, buf, len, HAL_MAX_DELAY);
}
uint8_t i2c_mem_write(uint8_t devaddr, uint8_t memaddr, uint16_t memaddr_size, uint8_t *buf, uint32_t len)
{
    dmaStatusCb.txMemCplt = 0;

    return HAL_I2C_Mem_Write(&hi2c3, devaddr, memaddr, memaddr_size, buf, len, HAL_MAX_DELAY);
}
uint8_t i2c_mem_read(uint8_t devaddr, uint8_t memaddr, uint16_t memaddr_size, uint8_t *buf, uint32_t len)
{
    dmaStatusCb.rxMemCplt = 0;

    return HAL_I2C_Mem_Read(&hi2c3, devaddr, memaddr, memaddr_size, buf, len, HAL_MAX_DELAY);
}
uint8_t i2c_mem_write_dma(uint8_t devaddr, uint8_t memaddr, uint16_t memaddr_size, uint8_t *buf, uint32_t len)
{
    dmaStatusCb.txMemCplt = 0;

    return HAL_I2C_Mem_Write_DMA(&hi2c3, devaddr, memaddr, memaddr_size, buf, len);
}
uint8_t i2c_mem_read_dma(uint8_t devaddr, uint8_t memaddr, uint16_t memaddr_size, uint8_t *buf, uint32_t len)
{
    dmaStatusCb.rxMemCplt = 0;

    return HAL_I2C_Mem_Read_DMA(&hi2c3, devaddr, memaddr, memaddr_size, buf, len);
}

uint8_t i2c_device_acks(uint8_t addr, uint32_t attempts)
{
    return HAL_I2C_IsDeviceReady(&hi2c3, addr, attempts, HAL_MAX_DELAY);
}
uint8_t i2c_dma_mem_read_cplt(void)
{
    return dmaStatusCb.rxMemCplt;

}
uint8_t i2c_dma_mem_write_cplt(void)
{
    return dmaStatusCb.txMemCplt;

}
uint8_t i2c_dma_read_cplt(void)
{
    return dmaStatusCb.rxCplt;

}
uint8_t i2c_dma_write_cplt(void)
{
    return dmaStatusCb.txCplt;
}
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
        dmaStatusCb.rxCplt = 1;
}
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    dmaStatusCb.rxCplt = 1;
}
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
        dmaStatusCb.txMemCplt = 1;
    
}
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
        dmaStatusCb.rxMemCplt = 1;
}
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
        memset((void*)&dmaStatusCb, 1, sizeof(dmaStatusCb));
}
void HAL_I2C_AbortCpltCallback(I2C_HandleTypeDef *hi2c)
{
        memset((void*)&dmaStatusCb, 1, sizeof(dmaStatusCb));

}