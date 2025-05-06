#include "bmp280_spi.h"

/* helpers CS */
#define CS_LOW(d)   HAL_GPIO_WritePin((d)->cs_port, (d)->cs_pin, GPIO_PIN_RESET)
#define CS_HIGH(d)  HAL_GPIO_WritePin((d)->cs_port, (d)->cs_pin, GPIO_PIN_SET)

/* write 8 bits */
static HAL_StatusTypeDef bmp280_write8(bmp280_t *d, uint8_t reg, uint8_t val)
{
  uint8_t buf[2] = { reg & 0x7F, val };
  CS_LOW(d);
  HAL_StatusTypeDef st = HAL_SPI_Transmit(d->hspi, buf, 2, HAL_MAX_DELAY);
  CS_HIGH(d);
  return st;
}

/* read N bytes */
static HAL_StatusTypeDef bmp280_read(bmp280_t *d, uint8_t reg,
                                     uint8_t *dst, uint16_t len)
{
  uint8_t addr = reg | 0x80;
  CS_LOW(d);
  HAL_StatusTypeDef st = HAL_SPI_Transmit(d->hspi, &addr, 1, HAL_MAX_DELAY);
  if (st == HAL_OK)
      st = HAL_SPI_Receive(d->hspi, dst, len, HAL_MAX_DELAY);
  CS_HIGH(d);
  return st;
}

/* little-endian helpers */
static uint16_t read16_LE(bmp280_t *d, uint8_t reg)
{
  uint8_t b[2]; bmp280_read(d, reg, b, 2);
  return (uint16_t)b[0] | ((uint16_t)b[1] << 8);
}
static int16_t readS16_LE(bmp280_t *d, uint8_t reg)
{
  return (int16_t)read16_LE(d, reg);
}

/* ---------- init ---------- */
HAL_StatusTypeDef bmp280_init(bmp280_t *dev)
{
  /* reset */
  bmp280_write8(dev, BMP280_REG_RESET, BMP280_RESET_VALUE);
  HAL_Delay(2);

  /* ID */
  uint8_t id;
  if (bmp280_read(dev, BMP280_REG_ID, &id, 1) != HAL_OK || id != 0x58)
      return HAL_ERROR;

  /* coeficientes */
  bmp280_calib_t *c = &dev->calib;
  c->dig_T1 = read16_LE (dev, 0x88);
  c->dig_T2 = readS16_LE(dev, 0x8A);
  c->dig_T3 = readS16_LE(dev, 0x8C);
  c->dig_P1 = read16_LE (dev, 0x8E);
  c->dig_P2 = readS16_LE(dev, 0x90);
  c->dig_P3 = readS16_LE(dev, 0x92);
  c->dig_P4 = readS16_LE(dev, 0x94);
  c->dig_P5 = readS16_LE(dev, 0x96);
  c->dig_P6 = readS16_LE(dev, 0x98);
  c->dig_P7 = readS16_LE(dev, 0x9A);
  c->dig_P8 = readS16_LE(dev, 0x9C);
  c->dig_P9 = readS16_LE(dev, 0x9E);

  /* configuración */
  bmp280_write8(dev, BMP280_REG_CONFIG, 0xA0); /* t_sb=1000 ms, filtro 0 */
  bmp280_write8(dev, BMP280_REG_CTRL_MEAS,
                BMP280_OSRS_T_x1 | BMP280_OSRS_P_x1 | BMP280_MODE_NORMAL);

  return HAL_OK;
}

/* ---------- lectura cruda 24 bits ---------- */
static uint32_t read24(bmp280_t *d, uint8_t reg)
{
  uint8_t b[3]; bmp280_read(d, reg, b, 3);
  return (uint32_t)b[0] << 16 | (uint32_t)b[1] << 8 | b[2];
}

/* ---------- temperatura ---------- */
float bmp280_read_temp(bmp280_t *dev)
{
  int32_t adc_T = (int32_t)(read24(dev, BMP280_REG_TEMP_MSB) >> 4);

  int32_t var1 = ((((adc_T >> 3) - ((int32_t)dev->calib.dig_T1 << 1)) *
                  dev->calib.dig_T2) >> 11);
  int32_t var2 = (((((adc_T >> 4) - dev->calib.dig_T1) *
                   ((adc_T >> 4) - dev->calib.dig_T1)) >> 12) *
                   dev->calib.dig_T3) >> 14;
  dev->t_fine = var1 + var2;

  return ((dev->t_fine * 5 + 128) >> 8) / 100.0f;
}

/* ---------- presión ---------- */
float bmp280_read_pres(bmp280_t *dev)
{
  bmp280_read_temp(dev);  /* actualiza t_fine */

  int32_t adc_P = (int32_t)(read24(dev, BMP280_REG_PRESS_MSB) >> 4);
  int64_t var1 = (int64_t)dev->t_fine - 128000;
  int64_t var2 = var1 * var1 * dev->calib.dig_P6;
  var2 += (var1 * dev->calib.dig_P5) << 17;
  var2 += ((int64_t)dev->calib.dig_P4) << 35;
  var1 = ((var1 * var1 * dev->calib.dig_P3) >> 8) +
         ((var1 * dev->calib.dig_P2) << 12);
  var1 = (((((int64_t)1) << 47) + var1) * dev->calib.dig_P1) >> 33;
  if (var1 == 0) return 0;

  int64_t p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (dev->calib.dig_P9 * (p >> 13) * (p >> 13)) >> 25;
  var2 = (dev->calib.dig_P8 * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (dev->calib.dig_P7 << 4);

  return (float)p / 256.0f;      /* Pascales */
}

/* ---------- altitud ---------- */
float bmp280_read_alt(bmp280_t *dev, float sea_hPa)
{
  float pres_hPa = bmp280_read_pres(dev) / 100.0f;
  return 44330.0f * (1.0f - powf(pres_hPa / sea_hPa, 0.1903f));
}
