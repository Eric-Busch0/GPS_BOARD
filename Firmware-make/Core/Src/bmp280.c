/*
 * bmp280.c
 *
 *  Created on: Jun 9, 2023
 *      Author: ebusc
 */

#include "bmp280.h"

#include "main.h"
#include <string.h>

extern SPI_HandleTypeDef hspi1;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;

#define BMP280_ID_ADDR 0xD0
#define BMP280_RESET_ADDR 0xE0
#define BMP280_CTRL_MEAS_ADDR 0xF4
#define BMP280_CONFIG_ADDR 0xF5
#define BMP280_PRESS_ADDR_MSB 0xF7
#define BMP280_PRESS_ADDR_LSB 0xF8
#define BMP280_PRESS_ADDR_XLSB 0xF9
#define BMP280_TEMP_ADDR_MSB 0xFA
#define BMP280_TEMP_ADDR_LSB 0xFB
#define BMP280_TEMP_ADDR_XLSB 0xFC

#define BUF_SIZE 32
static uint8_t wrbuf[BUF_SIZE] = {0};
static uint8_t rdbuf[BUF_SIZE] = {0};
static uint32_t rdLen = 0, wrLen = 0;
bmp_cplt_callback_t userRxCb = NULL, userTxCb = NULL;

typedef enum
{
	BMP280_IDLE,
	BMP280_GET_ID,
	BMP280_RESET,
	BMP280_SET_CTRL,
	BMP280_SET_CONFIG,
	BMP280_GET_TEMP,
	BMP280_GET_PRESS,
	BMP280_GET_PRESS_TEMP
} bpm280_op_t;

struct
{
	uint16_t digT1;
	int16_t digT2;
	int16_t digT3;
	uint16_t digP1;
	int16_t digP2;
	int16_t digP3;
	int16_t digP4;
	int16_t digP5;
	int16_t digP6;
	int16_t digP7;
	int16_t digP8;
	int16_t digP9;

	int32_t var1;
	int32_t var2;

} trim_values;

bpm280_op_t currentOp = BMP280_IDLE;
uint32_t currentTemp = 0;
uint32_t currentPress = 0;

static uint8_t bmp280_write_async(bpm280_op_t op, uint8_t addr, uint8_t *data, size_t len)
{

	if (len + 1 > BUF_SIZE)
	{
		return HAL_ERROR;
	}

	while (currentOp != BMP280_IDLE)
		;
	currentOp = op;

	wrbuf[0] = addr;
	memcpy(wrbuf + 1, data, len);

	wrLen = len + 1;

	return HAL_SPI_Transmit_DMA(&hspi1, wrbuf, wrLen);
}
static uint8_t bmp280_read_async(bpm280_op_t op, uint8_t addr, uint8_t *data, size_t len)
{
	if (len + 1 > BUF_SIZE)
	{
		return HAL_ERROR;
	}

	while (currentOp != BMP280_IDLE)
		;
	currentOp = op;

	memset(wrbuf, 0, BUF_SIZE);

	wrbuf[0] = addr;

	rdLen = len;
	return HAL_SPI_TransmitReceive_DMA(&hspi1, wrbuf, rdbuf, rdLen);
}
static uint8_t bmp280_write_sync(uint8_t addr, uint8_t *data, size_t len,
								 uint32_t timeout)
{

	wrLen = len + 1;
	// This may be faster to just combine the two?
	HAL_SPI_Transmit(&hspi1, &addr, 1, timeout);
	return HAL_SPI_Transmit(&hspi1, data, len, timeout);
}
static uint8_t bmp280_read_sync(uint8_t addr, uint8_t *data, size_t len, uint32_t timeout)
{

	HAL_GPIO_WritePin(BMP_CS_GPIO_Port, BMP_CS_Pin, 0);

	rdLen = len;
	HAL_SPI_Transmit(&hspi1, &addr, 1, timeout);
	uint8_t ret = HAL_SPI_Receive(&hspi1, data, len, timeout);
	HAL_GPIO_WritePin(BMP_CS_GPIO_Port, BMP_CS_Pin, 1);
	return ret;
}
static void bmp280_get_trim_values(void)
{
}
uint8_t bmp280_init(bmp_cplt_callback_t rxCb, bmp_cplt_callback_t txCb)
{

	memset((void *)&trim_values, 0, sizeof(trim_values));

	userRxCb = rxCb;
	userTxCb = txCb;

	/*
	 * Verify we can communicate to the device
	 */
	uint8_t id = 0;
	if (bmp280_getid(&id) != HAL_OK)
	{
		return HAL_ERROR;
	}
	else if (id != BMP280_DEV_ID)
	{
		return HAL_ERROR;
	}

	return HAL_OK;
}

uint8_t bmp280_getid(uint8_t *id)
{

	uint8_t ret = bmp280_read_sync(BMP280_ID_ADDR, id, 1, HAL_MAX_DELAY);

	//	*id &= ~(1 << 7);
	//	*id <<=1;
	return ret;
}

void bmp280_reset(void)
{
	static const uint8_t RESET_CMD = 0xB6;

	bmp280_write_async(BMP280_RESET, BMP280_RESET_ADDR, (uint8_t *)&RESET_CMD, 1);
}

void bmp280_set_ctrl_meas(bmp280_ctrl_cfg_t *ctrlCfg)
{
	static uint8_t ctrlVal = 0x00;
	ctrlVal = (ctrlCfg->tempOversampling << 5) | (ctrlCfg->pressOversapling << 2) | ctrlCfg->mode;
	bmp280_write_async(BMP280_SET_CTRL, BMP280_CTRL_MEAS_ADDR, &ctrlVal, 1);
}
uint8_t bmp280_set_config(bmp280_config_t *config)
{

	static uint8_t configVal = 0;

	configVal = (config->standbyTime << 5) | (config->iirFilterTc << 2) | config->spi3wEn;

	return bmp280_write_async(BMP280_SET_CONFIG, BMP280_CONFIG_ADDR, &configVal, 1);
}

uint8_t bmp280_start_press_read(void)
{
	static const uint32_t PRESS_SIZE = 3;
	return bmp280_read_async(BMP280_GET_PRESS, BMP280_PRESS_ADDR_MSB, rdbuf, PRESS_SIZE);
}
uint8_t bmp280_start_temp_read(void)
{
	static const uint32_t TEMP_SIZE = 3;

	return bmp280_read_async(BMP280_GET_TEMP, BMP280_TEMP_ADDR_MSB, rdbuf, TEMP_SIZE);
}
uint8_t bmp280_read_pressure_and_temperature(void)
{

	static const uint32_t PRESS_TEMP_SIZE = 6;
	return bmp280_read_async(BMP280_GET_PRESS_TEMP, BMP280_PRESS_ADDR_MSB, rdbuf, PRESS_TEMP_SIZE);
}
uint32_t bmp280_get_pressure(void)
{

	return currentPress;
}
uint8_t bmp280_get_temp(uint32_t *temperature)
{

	static const uint32_t TEMP_SIZE = 3;
	uint8_t temp[3] = {0};

	static const uint8_t addr = BMP280_TEMP_ADDR_MSB;
	uint8_t ret = bmp280_read_sync(addr, temp, TEMP_SIZE, HAL_MAX_DELAY);

	*temperature = temp[2] << 16 | temp[1] << 8 | temp[0];

	return ret;
}
void bmp280_get_pressure_and_temp(uint32_t *pressure, uint32_t *temp)
{
	*pressure = currentPress;
	*temp = currentTemp;
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{

	switch (currentOp)
	{

	case BMP280_GET_PRESS:

		currentPress = rdbuf[0] << 11 | rdbuf[1] << 3 | rdbuf[2];

		break;
	case BMP280_GET_TEMP:
		currentTemp = rdbuf[0] << 11 | rdbuf[1] << 3 | rdbuf[2];
		break;
	case BMP280_GET_PRESS_TEMP:
		currentPress = rdbuf[0] << 11 | rdbuf[1] << 3 | rdbuf[2];
		currentTemp = rdbuf[3] << 11 | rdbuf[4] << 3 | rdbuf[5];
		break;
	default:
		break;
	}

	currentOp = BMP280_IDLE;
}
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	currentOp = BMP280_IDLE;
}
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	currentOp = BMP280_IDLE;
}
