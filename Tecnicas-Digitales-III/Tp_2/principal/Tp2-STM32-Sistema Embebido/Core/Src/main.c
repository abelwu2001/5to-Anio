#include "main.h"
#include "cmsis_os.h"
#include <string.h>
#include "bmp280_spi.h"
#include "stm32f1xx_hal_dma.h"  // Incluir el encabezado DMA

#define RS485_DE_PORT GPIOB
#define RS485_DE_PIN  GPIO_PIN_9

ADC_HandleTypeDef hadc1;
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
TIM_HandleTypeDef htim1;
DMA_HandleTypeDef hdma_adc1;  // Declaración completa del manejador DMA

osThreadId defaultTaskHandle;
osThreadId entradasTaskHandle;
osThreadId uartTaskHandle;

// Buffers y variables compartidas
uint8_t transmision[12] = {0};
uint8_t recepcion[5] = {0x02, 0, 0, 0, 0};
volatile uint8_t rx_ready = 0;
uint16_t adc_buffer[3];  // Buffer para DMA ADC

// Mutex para protección de recursos compartidos
osMutexId transmisionMutex;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_DMA_Init(void);  // Prototipo agregado
void StartDefaultTask(void const * argument);
void StartEntradasTask(void const * argument);
void StartUartTask(void const * argument);

// Función para calcular CRC-8
uint8_t calcular_crc(uint8_t* data, uint8_t length) {
    const uint8_t poly = 0x07;
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            crc = (crc & 0x80) ? (crc << 1) ^ poly : (crc << 1);
        }
    }
    return crc;
}

// Función segura para transmisión RS485
void RS485_Transmit(uint8_t* data, uint16_t size) {
    HAL_GPIO_WritePin(RS485_DE_PORT, RS485_DE_PIN, GPIO_PIN_SET);
    HAL_UART_Transmit(&huart2, data, size, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(RS485_DE_PORT, RS485_DE_PIN, GPIO_PIN_RESET);
}

// Callback de recepción UART
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        // Verificar CRC de los primeros 4 bytes
        uint8_t crc_calculado = calcular_crc(recepcion, 4);
        if (crc_calculado == recepcion[4]) {
            rx_ready = 1;
        }
        // Reactivar recepción
        HAL_UART_Receive_IT(&huart2, recepcion, sizeof(recepcion));
    }
}

int main(void) {
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();  // Inicializar DMA primero
    MX_USART1_UART_Init();
    MX_USART2_UART_Init();
    MX_SPI1_Init();
    MX_ADC1_Init();
    MX_TIM1_Init();

    // Inicialización de periféricos
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // PA8
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3); // PA10
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, 3); // Iniciar ADC con DMA
    BMP280_SPI_Init(&hspi1, GPIOA, GPIO_PIN_4);

    // Crear mutex para protección de buffer
    osMutexDef(transmisionMutex);
    transmisionMutex = osMutexCreate(osMutex(transmisionMutex));

    // Iniciar recepción por interrupción para RS485
    HAL_UART_Receive_IT(&huart2, recepcion, sizeof(recepcion));

    // Creación de tareas
    osThreadDef(defaultTask, StartDefaultTask, osPriorityHigh, 0, 128);
    defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

    osThreadDef(entradasTask, StartEntradasTask, osPriorityAboveNormal, 0, 256);
    entradasTaskHandle = osThreadCreate(osThread(entradasTask), NULL);

    osThreadDef(uartTask, StartUartTask, osPriorityNormal, 0, 128);
    uartTaskHandle = osThreadCreate(osThread(uartTask), NULL);

    // Iniciar planificador
    osKernelStart();

    while (1) {}
}

void StartDefaultTask(void const * argument) {
    for (;;) {
        // Transmitir por RS485 con protección de mutex
        osMutexWait(transmisionMutex, osWaitForever);
        RS485_Transmit(transmision, sizeof(transmision));
        osMutexRelease(transmisionMutex);

        osDelay(1000);
    }
}

void StartEntradasTask(void const * argument) {
    float temp, press;
    uint32_t press_entero;

    for (;;) {
        // Copiar valores ADC de forma segura usando sección crítica
        uint16_t local_adc[3];
        taskENTER_CRITICAL();
        local_adc[0] = adc_buffer[0];
        local_adc[1] = adc_buffer[1];
        local_adc[2] = adc_buffer[2];
        taskEXIT_CRITICAL();

        // Leer entradas digitales
        uint8_t entradas_digitales = 0;
        entradas_digitales |= HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);
        entradas_digitales |= HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) << 1;
        entradas_digitales |= HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11) << 2;

        // Leer sensor BMP280
        temp = BMP280_ReadTemperature();
        press = BMP280_ReadPressure();
        uint16_t press_entero = (uint16_t)(press * 10);

        // Actualizar buffer de transmisión con protección
        osMutexWait(transmisionMutex, osWaitForever);

        transmision[0] = 0x02;  // Byte de inicio
        transmision[1] = local_adc[0] & 0xFF;         // ADC0 LSB
        transmision[2] = (local_adc[0] >> 8) & 0xFF;  // ADC0 MSB
        transmision[3] = local_adc[1] & 0xFF;         // ADC1 LSB
        transmision[4] = (local_adc[1] >> 8) & 0xFF;  // ADC1 MSB
        transmision[5] = local_adc[2] & 0xFF;         // ADC2 LSB
        transmision[6] = (local_adc[2] >> 8) & 0xFF;  // ADC2 MSB
        transmision[7] = entradas_digitales;
        transmision[8] = (uint8_t)temp;
        // Corrección de endianness para presión
        transmision[9] = press_entero & 0xFF;          // LSB
        transmision[10] = (press_entero >> 8) & 0xFF;  // MSB
        transmision[11] = calcular_crc(transmision, 11);

        osMutexRelease(transmisionMutex);

        osDelay(100);
    }
}

void StartUartTask(void const * argument) {
    uint8_t local_recepcion[5];
    uint8_t procesar_datos = 0;

    for (;;) {
        // Transmitir por RS232 (monitoreo)
        osMutexWait(transmisionMutex, osWaitForever);
        HAL_UART_Transmit(&huart1, transmision, sizeof(transmision), HAL_MAX_DELAY);
        osMutexRelease(transmisionMutex);

        // Procesar comandos recibidos
        if (rx_ready) {
            // Copiar datos a buffer local
            taskENTER_CRITICAL();
            memcpy(local_recepcion, recepcion, sizeof(recepcion));
            procesar_datos = 1;
            rx_ready = 0;
            taskEXIT_CRITICAL();
        }

        if (procesar_datos) {
            // Controlar salidas digitales
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, (local_recepcion[1] & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, (local_recepcion[1] & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, (local_recepcion[1] & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);

            // Controlar PWM con mayor precisión
            uint16_t duty1 = (uint16_t)((local_recepcion[2] * 4095) / 255);
            uint16_t duty2 = (uint16_t)((local_recepcion[3] * 4095) / 255);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty1);
            __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, duty2);

            procesar_datos = 0;
        }

        osDelay(500);
    }
}

void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
}

static void MX_GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    // PA4 - CS del BMP280
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

    // PB1, PB10, PB11 - Entradas digitales
    GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_10 | GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // PB9 - Control DE para RS485
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);

    // PB12, PB13, PB14 - Salidas digitales
    GPIO_InitStruct.Pin = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
}

static void MX_USART1_UART_Init(void) {
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart1);
}

static void MX_USART2_UART_Init(void) {
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_HalfDuplex_Init(&huart2);
}

static void MX_SPI1_Init(void) {
    hspi1.Instance = SPI1;
    hspi1.Init.Mode = SPI_MODE_MASTER;
    hspi1.Init.Direction = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    hspi1.Init.CRCPolynomial = 10;
    HAL_SPI_Init(&hspi1);
}

static void MX_ADC1_Init(void) {
    ADC_ChannelConfTypeDef sConfig = {0};

    hadc1.Instance = ADC1;
    hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
    hadc1.Init.ContinuousConvMode = ENABLE;
    // Eliminado: hadc1.Init.DMAContinuousRequests = ENABLE; (no existe en F1)
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 3;
    HAL_ADC_Init(&hadc1);

    sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;

    // Canal 0 (PA0)
    sConfig.Channel = ADC_CHANNEL_0;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    // Canal 1 (PA1)
    sConfig.Channel = ADC_CHANNEL_1;
    sConfig.Rank = ADC_REGULAR_RANK_2;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    // Canal 8 (PB0)
    sConfig.Channel = ADC_CHANNEL_8;
    sConfig.Rank = ADC_REGULAR_RANK_3;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    HAL_ADCEx_Calibration_Start(&hadc1);  // Calibración ADC
}

static void MX_TIM1_Init(void) {
    __HAL_RCC_TIM1_CLK_ENABLE();

    TIM_OC_InitTypeDef sConfigOC = {0};
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;
    htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim1.Init.Period = 4095;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(&htim1);

    // PWM init
    HAL_TIM_PWM_Init(&htim1);

    // OC config (canal 1 y 3)
    sConfigOC.OCMode = TIM_OCMODE_PWM1;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3);

    // Configuración dead-time
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = 0;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig);
}

static void MX_DMA_Init(void) {
    __HAL_RCC_DMA1_CLK_ENABLE();

    hdma_adc1.Instance = DMA1_Channel1;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;

    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK) {
        Error_Handler();
    }

    __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);
}

void Error_Handler(void) {
    while(1) {
        // Manejo de error (parpadeo LED, etc.)
    }
}




