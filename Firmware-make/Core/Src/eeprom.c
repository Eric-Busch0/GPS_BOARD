#include <string.h>
#include "eeprom.h"
#include "i2c.h"
#include "main.h"

#define EEPROM_I2C_ADDR (0x50 << 1)
#define EEPROM_PAGE_SIZE 64

#define ADDR_LEN sizeof(uint16_t)
static volatile uint8_t write_buf[EEPROM_PAGE_SIZE + ADDR_LEN] = {0};

static inline uint32_t get_next_page(uint32_t addr)
{
    return addr + (EEPROM_PAGE_SIZE - addr % EEPROM_PAGE_SIZE);
}

void enable_wp(void)
{
    HAL_GPIO_WritePin(EEPROM_WP_GPIO_Port, EEPROM_WP_Pin, GPIO_PIN_SET);
}
void disable_wp(void)
{
    HAL_GPIO_WritePin(EEPROM_WP_GPIO_Port, EEPROM_WP_Pin, GPIO_PIN_RESET);
}
uint8_t eeprom_write(uint32_t addr, void *data, uint32_t len)
{
    /*
        If the write does not write over a page boundry it can be done using DMA
        Otherwise, it needs to be chunked and sent synchronously
    */
    uint32_t next_page = get_next_page(addr);

    if (addr + len < next_page)
    {
        write_buf[0] = (addr & 0xff00) >> 8;
        write_buf[1] = addr & 0xff;

        memcpy((uint8_t*)write_buf + ADDR_LEN, data, len % EEPROM_PAGE_SIZE);

        disable_wp();
        uint8_t ret = i2c_write(EEPROM_I2C_ADDR, (uint8_t*) write_buf, len + 1);
        enable_wp();

        if (ret != HAL_OK)
        {
            return ret;
        }
        static const uint32_t WAIT_ATTEMPTS = 100;

        return i2c_device_acks(EEPROM_I2C_ADDR, WAIT_ATTEMPTS);

        // return i2c_mem_write_dma(EEPROM_I2C_ADDR, write_buf, len + 1);
    }
    return HAL_OK;
}
uint8_t eeprom_read(uint32_t addr, void *data, uint32_t len)
{

    write_buf[0] = (addr & 0xff00) >> 8;
    write_buf[1] = addr & 0xff;
    HAL_StatusTypeDef ret = i2c_write(EEPROM_I2C_ADDR, (uint8_t*)write_buf, ADDR_LEN);

    if (ret != HAL_OK)
    {
        return ret;
    }

    return i2c_read(EEPROM_I2C_ADDR, data, len);
}
