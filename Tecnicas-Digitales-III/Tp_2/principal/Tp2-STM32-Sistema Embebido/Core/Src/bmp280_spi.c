#include "stm32f1xx_hal.h"
#include "bmp280_spi.h"

static SPI_HandleTypeDef *bmp_spi;
static GPIO_TypeDef *bmp_cs_port;
static uint16_t bmp_cs_pin;
static BMP280_CalibData bmp_calib;
static int32_t t_fine;

static void CS_Select(void) {
    HAL_GPIO_WritePin(bmp_cs_port, bmp_cs_pin, GPIO_PIN_RESET);
}

static void CS_Unselect(void) {
    HAL_GPIO_WritePin(bmp_cs_port, bmp_cs_pin, GPIO_PIN_SET);
}

static void BMP280_Write8(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg & 0x7F, value};
    CS_Select();
    HAL_SPI_Transmit(bmp_spi, data, 2, HAL_MAX_DELAY);
    CS_Unselect();
}

static void BMP280_ReadBytes(uint8_t reg, uint8_t *buffer, uint16_t length) {
    reg |= 0x80; // Set read bit
    CS_Select();
    HAL_SPI_Transmit(bmp_spi, &reg, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(bmp_spi, buffer, length, HAL_MAX_DELAY);
    CS_Unselect();
}

static void BMP280_ReadCalibration(void) {
    uint8_t calib[24];
    BMP280_ReadBytes(BMP280_REG_CALIB_START, calib, 24);

    bmp_calib.dig_T1 = (uint16_t)(calib[1] << 8 | calib[0]);
    bmp_calib.dig_T2 = (int16_t)(calib[3] << 8 | calib[2]);
    bmp_calib.dig_T3 = (int16_t)(calib[5] << 8 | calib[4]);

    bmp_calib.dig_P1 = (uint16_t)(calib[7] << 8 | calib[6]);
    bmp_calib.dig_P2 = (int16_t)(calib[9] << 8 | calib[8]);
    bmp_calib.dig_P3 = (int16_t)(calib[11] << 8 | calib[10]);
    bmp_calib.dig_P4 = (int16_t)(calib[13] << 8 | calib[12]);
    bmp_calib.dig_P5 = (int16_t)(calib[15] << 8 | calib[14]);
    bmp_calib.dig_P6 = (int16_t)(calib[17] << 8 | calib[16]);
    bmp_calib.dig_P7 = (int16_t)(calib[19] << 8 | calib[18]);
    bmp_calib.dig_P8 = (int16_t)(calib[21] << 8 | calib[20]);
    bmp_calib.dig_P9 = (int16_t)(calib[23] << 8 | calib[22]);
}

void BMP280_SPI_Init(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin) {
    bmp_spi = hspi;
    bmp_cs_port = cs_port;
    bmp_cs_pin = cs_pin;

    CS_Unselect();
    HAL_Delay(100);

    // Reset sensor
    BMP280_Write8(BMP280_REG_RESET, 0xB6);
    HAL_Delay(100);

    uint8_t id = 0;
    BMP280_ReadBytes(BMP280_REG_ID, &id, 1);
    if (id != 0x58) {
        // ID incorrecto, colgarse
        while(1);
    }

    BMP280_ReadCalibration();

    // Configurar oversampling x1 para temperatura y presion, modo normal
    BMP280_Write8(BMP280_REG_CTRL_MEAS, 0x27);
    // Configurar tiempo de espera y filtro (tiempo standby 0.5ms, filtro off)
    BMP280_Write8(BMP280_REG_CONFIG, 0x00);
}

float BMP280_ReadTemperature(void) {
    uint8_t data[3];
    BMP280_ReadBytes(BMP280_REG_TEMP_MSB, data, 3);
    int32_t adc_T = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | (data[2] >> 4);

    int32_t var1 = ((((adc_T >> 3) - ((int32_t)bmp_calib.dig_T1 << 1))) * ((int32_t)bmp_calib.dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)bmp_calib.dig_T1)) * ((adc_T >> 4) - ((int32_t)bmp_calib.dig_T1))) >> 12) * ((int32_t)bmp_calib.dig_T3)) >> 14;

    t_fine = var1 + var2;

    float T = (t_fine * 5 + 128) >> 8;
    return T / 100.0f;
}

float BMP280_ReadPressure(void) {
    uint8_t data[3];
    BMP280_ReadBytes(BMP280_REG_PRESS_MSB, data, 3);
    int32_t adc_P = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | (data[2] >> 4);

    int64_t var1 = ((int64_t)t_fine) - 128000;
    int64_t var2 = var1 * var1 * (int64_t)bmp_calib.dig_P6;
    var2 = var2 + ((var1 * (int64_t)bmp_calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)bmp_calib.dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)bmp_calib.dig_P3) >> 8) + ((var1 * (int64_t)bmp_calib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)bmp_calib.dig_P1) >> 33;

    if (var1 == 0) return 0;

    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)bmp_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)bmp_calib.dig_P8) * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)bmp_calib.dig_P7) << 4);
    return (float)p / 25600.0f; // hPa
}
