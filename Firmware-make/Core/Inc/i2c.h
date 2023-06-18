#ifndef I2C_H
#define I2C_H


#include <stdint.h>

#ifdef __cplusplus
extern "C"{
#endif

//Place c code here




uint8_t i2c_write(uint8_t addr, uint8_t* buf, uint32_t len);
uint8_t i2c_read(uint8_t addr, uint8_t* buf, uint32_t len);
uint8_t i2c_write_dma(uint8_t addr, uint8_t* buf, uint32_t len);
uint8_t i2c_read_dma(uint8_t addr, uint8_t* buf, uint32_t len);
uint8_t i2c_device_acks(uint8_t addr, uint32_t attempts);


#ifdef __cplusplus
}
#endif

#endif