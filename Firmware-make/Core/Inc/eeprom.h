#ifndef EEPROM_H
#define EEPROM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"{
#endif

//Place c code here
uint8_t eeprom_write(uint32_t addr, void * data, uint32_t len);
uint8_t eeprom_read(uint32_t addr, void * data, uint32_t len);


void enable_wp(void);
void disable_wp(void);


#ifdef __cplusplus
}
#endif

#endif