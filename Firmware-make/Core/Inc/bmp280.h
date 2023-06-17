/*
 * bmp280.h
 *
 *  Created on: Jun 9, 2023
 *      Author: ebusc
 */

#ifndef INC_BMP280_H_
#define INC_BMP280_H_
#include <stdint.h>

#define BMP280_DEV_ID 0x58
typedef void (*bmp_cplt_callback_t)(uint8_t *data, uint32_t len);

typedef enum {
	BMP280_PRESS_OS_SKIPPED = 0u,
	BMP280_PRESS_OS_1X,
	BMP280_PRESS_OS_2X,
	BMP280_PRESS_OS_4X,
	BMP280_PRESS_OS_8X,
	BMP280_PRESS_OS_16X,
} bmp280_press_oversampling_t;

typedef enum {
	BMP280_TEMP_OS_SKIPPED = 0u,
	BMP280_TEMP_OS_1X,
	BMP280_TEMP_OS_2X,
	BMP280_TEMP_OS_4X,
	BMP280_TEMP_OS_8X,
	BMP280_TEMP_OS_16X,
} bmp280_temp_oversampling_t;
typedef enum {
	BMP280_MODE_SLEEP, BMP280_MODE_FORCED, BMP280_MODE_NORMAL
} bmp280_mode_t;

typedef struct {
	bmp280_press_oversampling_t pressOversapling;
	bmp280_temp_oversampling_t tempOversampling;
	bmp280_mode_t mode;
} bmp280_ctrl_cfg_t;
typedef struct {
	uint8_t standbyTime;
	uint8_t iirFilterTc;
	uint8_t spi3wEn;
} bmp280_config_t;

uint8_t bmp280_init(bmp_cplt_callback_t rxCb, bmp_cplt_callback_t txCb);
uint8_t bmp280_getid(uint8_t *id);
void bmp280_reset(void);
void bmp280_set_ctrl_meas(bmp280_ctrl_cfg_t *ctrlCfg);
void bmp280_set_config(bmp280_config_t *config);
uint8_t bmp280_start_press_read(void);
uint8_t bmp280_start_temp_read(void);
uint8_t bmp280_read_pressure_and_temperature(void);
uint32_t bmp280_get_pressure(void);
uint32_t bmp280_get_temp(void);
void bmp280_get_pressure_and_temp(uint32_t *pressure , uint32_t *temp);
#endif /* INC_BMP280_H_ */
