#include "sam_m10q.h"
#include "stm32f401xc.h"
#include "main.h"
#include "i2c.h"

#define SAM_M10Q_I2CA_ADDR (0x42 << 1)

void sam_m10q_init(void);
void sam_m10q_reset(void)
{
    HAL_GPIO_WritePin(GPS_INT_GPIO_Port, GPS_INT_Pin, 0);
    HAL_GPIO_WritePin(GPS_INT_GPIO_Port, GPS_INT_Pin, 1);
}
