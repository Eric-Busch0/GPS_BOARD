#ifndef LIS2HD12_H
#define LIS2HD12_H


#ifdef __cplusplus
extern "C"{
#endif

#define LISS2HD12_WHO_AM_I 0x33

//Place c code here
void lis2hd12_init(void);
uint8_t lis2hd12_who_am_i(uint8_t *who_am_i);


#ifdef __cplusplus
}
#endif

#endif