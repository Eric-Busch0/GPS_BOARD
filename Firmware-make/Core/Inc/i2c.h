#ifndef I2C_H
#define I2C_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif


    typedef struct{
        uint8_t devaddr;
        uint8_t memaddr;
        uint8_t memsize;
        uint8_t *buf;
        uint32_t len;
    }i2c_mem_op_cfg_t;

    // Place c code here

    uint8_t i2c_write(uint8_t addr, uint8_t *buf, uint32_t len);
    uint8_t i2c_read(uint8_t addr, uint8_t *buf, uint32_t len);
    uint8_t i2c_mem_write(uint8_t devaddr, uint8_t memaddr,
                          uint16_t memaddr_size, uint8_t *buf, uint32_t len);
    uint8_t i2c_mem_read(uint8_t devaddr, uint8_t memaddr,
                         uint16_t memaddr_size, uint8_t *buf, uint32_t len);
    uint8_t i2c_mem_write_dma(uint8_t devaddr, uint8_t memaddr,
                              uint16_t memaddr_size, uint8_t *buf, uint32_t len);
    uint8_t i2c_mem_read_dma(uint8_t devaddr, uint8_t memaddr,
                             uint16_t memaddr_size, uint8_t *buf, uint32_t len);

    uint8_t i2c_device_acks(uint8_t addr, uint32_t attempts);
    uint8_t i2c_dma_mem_read_cplt(void);
    uint8_t i2c_dma_mem_write_cplt(void);
    uint8_t i2c_dma_read_cplt(void);
    uint8_t i2c_dma_write_cplt(void);

#ifdef __cplusplus
}
#endif

#endif