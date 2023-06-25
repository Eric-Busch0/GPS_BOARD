#ifndef RETCHECK_H
#define RETCHECK_H

#include "main.h"
#include <stdint.h>


#ifdef __cplusplus
extern "C"{
#endif

//Place c code here


#define RETCHECK(x) if((x) != HAL_OK) return x

#ifdef __cplusplus
}
#endif

#endif