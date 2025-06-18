#ifndef BMP280_SPI_H_
#define BMP280_SPI_H_

#include "stm32f1xx_hal.h"

// Registros BMP280
#define BMP280_REG_ID         0xD0
#define BMP280_REG_RESET      0xE0
#define BMP280_REG_CTRL_MEAS  0xF4
#define BMP280_REG_CONFIG     0xF5
#define BMP280_REG_PRESS_MSB  0xF7
#define BMP280_REG_TEMP_MSB   0xFA
#define BMP280_REG_CALIB_START 0x88

// Estructura para guardar la calibración
typedef struct {
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;

    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
} BMP280_CalibData;

// Funciones públicas
void BMP280_SPI_Init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin);
float BMP280_ReadTemperature(void);
float BMP280_ReadPressure(void);

#endif /* BMP280_SPI_H_ */
