#ifndef _BMP280_SPI_H_
#define _BMP280_SPI_H_

#include "stm32f1xx_hal.h"          /* ← cambia f1xx si tu MCU es otro */
#include <stdint.h>
#include <math.h>

/* ─────────────────  registros ───────────────── */
#define BMP280_REG_ID            0xD0
#define BMP280_REG_RESET         0xE0
#define BMP280_RESET_VALUE       0xB6
#define BMP280_REG_STATUS        0xF3
#define BMP280_REG_CTRL_MEAS     0xF4
#define BMP280_REG_CONFIG        0xF5
#define BMP280_REG_PRESS_MSB     0xF7
#define BMP280_REG_TEMP_MSB      0xFA

/* oversampling + modo normal */
#define BMP280_OSRS_T_x1         (1 << 5)
#define BMP280_OSRS_P_x1         (1 << 2)
#define BMP280_MODE_NORMAL       0x03

typedef struct {
  uint16_t dig_T1; int16_t dig_T2; int16_t dig_T3;
  uint16_t dig_P1; int16_t dig_P2; int16_t dig_P3;
  int16_t dig_P4;  int16_t dig_P5;  int16_t dig_P6;
  int16_t dig_P7;  int16_t dig_P8;  int16_t dig_P9;
} bmp280_calib_t;

typedef struct {
  SPI_HandleTypeDef *hspi;
  GPIO_TypeDef      *cs_port;
  uint16_t           cs_pin;
  bmp280_calib_t     calib;
  int32_t            t_fine;
} bmp280_t;

/* API */
HAL_StatusTypeDef bmp280_init     (bmp280_t *dev);
float             bmp280_read_temp(bmp280_t *dev);
float             bmp280_read_pres(bmp280_t *dev);
float             bmp280_read_alt (bmp280_t *dev, float sea_hPa);

#endif /* _BMP280_SPI_H_ */
