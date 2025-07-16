/**
 * @file hardware_init.c
 * @brief Hardware initialization and low-level setup
 */

#include "hardware_init.h"
#include "system_config.h"

/* Private function prototypes */
static void system_clock_config(void);
static void gpio_init(void);
static void timer_init(void);
static void dma_init(void);
static void adc_init(void);
static void uart_init(void);
static void spi_init(void);
static void usb_init(void);
static void watchdog_init(void);

/**
 * @brief Initialize all hardware peripherals
 * @return HAL_OK if successful, HAL_ERROR otherwise
 */
HAL_StatusTypeDef init_hardware(void)
{
    /* Reset of all peripherals, Initializes the Flash interface and Systick */
    HAL_Init();

    /* Configure the system clock to 168 MHz */
    system_clock_config();

    /* Initialize GPIO pins */
    gpio_init();

    /* Initialize DMA controllers */
    dma_init();

    /* Initialize timers for motor control and system timing */
    timer_init();

    /* Initialize communication interfaces */
    uart_init();
    spi_init();

    /* Initialize ADC for battery monitoring */
    adc_init();

    /* Initialize USB for configuration/telemetry */
    usb_init();

    /* Initialize watchdog timer for safety */
    watchdog_init();

    /* Enable cycle counter for precise timing */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    return HAL_OK;
}

/**
 * @brief System Clock Configuration
 * @details Configures system clock to 168MHz using external 8MHz crystal
 */
static void system_clock_config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 8;
    RCC_OscInitStruct.PLL.PLLN = 336;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = 7;

    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief Initialize GPIO pins
 */
static void gpio_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    /* Configure motor output pins */
    GPIO_InitStruct.Pin = MOTOR1_PIN | MOTOR2_PIN | MOTOR3_PIN | MOTOR4_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
    HAL_GPIO_Init(MOTOR_GPIO_PORT, &GPIO_InitStruct);

    /* Configure LED pins */
    GPIO_InitStruct.Pin = LED_RED_PIN | LED_GREEN_PIN | LED_BLUE_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_GPIO_PORT, &GPIO_InitStruct);

    /* Configure buzzer pin */
    GPIO_InitStruct.Pin = BUZZER_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(BUZZER_GPIO_PORT, &GPIO_InitStruct);

    /* Configure SPI chip select pins */
    GPIO_InitStruct.Pin = IMU_CS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(IMU_GPIO_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(IMU_GPIO_PORT, IMU_CS_PIN, GPIO_PIN_SET);

#ifdef USE_OSD
    GPIO_InitStruct.Pin = OSD_CS_PIN;
    HAL_GPIO_Init(OSD_GPIO_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(OSD_GPIO_PORT, OSD_CS_PIN, GPIO_PIN_SET);
#endif

    /* Configure IMU interrupt pin */
    GPIO_InitStruct.Pin = IMU_INT_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    HAL_GPIO_Init(IMU_INT_GPIO_PORT, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(EXTI4_IRQn);

    HAL_GPIO_WritePin(LED_GPIO_PORT, LED_RED_PIN | LED_GREEN_PIN | LED_BLUE_PIN, GPIO_PIN_RESET);
}

/**
 * @brief Initialize timers
 */
static void timer_init(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();

    TIM2->PSC = 0;
    TIM2->ARR = 139;
    TIM2->CR1 = 0;

    __HAL_RCC_TIM6_CLK_ENABLE();
    TIM6->PSC = 83;
    TIM6->ARR = 0xFFFF;
    TIM6->CR1 = TIM_CR1_CEN;

    __HAL_RCC_TIM7_CLK_ENABLE();
    TIM7->PSC = 41999;
    TIM7->ARR = 0xFFFF;
    TIM7->DIER = TIM_DIER_UIE;
    TIM7->CR1 = TIM_CR1_CEN;

    HAL_NVIC_SetPriority(TIM7_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(TIM7_IRQn);
}

/**
 * @brief Initialize DMA controllers
 */
static void dma_init(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 2, 0);
    HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 1, 0);
    HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 1, 0);
    HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 1, 0);
    HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);

    HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
    HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
    HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);
    HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
    HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
}

/**
 * @brief Initialize UART peripherals
 */
static void uart_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = RC_TX_PIN | RC_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(RC_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = TELEM_TX_PIN | TELEM_RX_PIN;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(TELEM_GPIO_PORT, &GPIO_InitStruct);
}

/**
 * @brief Initialize SPI peripherals
 */
static void spi_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = IMU_SCK_PIN | IMU_MISO_PIN | IMU_MOSI_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(IMU_GPIO_PORT, &GPIO_InitStruct);

#ifdef USE_OSD
    GPIO_InitStruct.Pin = OSD_SCK_PIN | OSD_MISO_PIN | OSD_MOSI_PIN;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(OSD_GPIO_PORT, &GPIO_InitStruct);
#endif
}

/**
 * @brief Initialize ADC for battery monitoring
 */
static void adc_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    GPIO_InitStruct.Pin = VBAT_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(VBAT_GPIO_PORT, &GPIO_InitStruct);
}

/**
 * @brief Initialize USB peripheral
 */
static void usb_init(void)
{
    /* USB pins are fixed on STM32F405 (PA11/PA12) */
}

/**
 * @brief Initialize watchdog timer
 */
static void watchdog_init(void)
{
#ifdef USE_WATCHDOG
    IWDG->KR = 0x5555;
    IWDG->PR = IWDG_PR_PR_2;
    IWDG->RLR = 1250;
    IWDG->KR = 0xAAAA;
    IWDG->KR = 0xCCCC;
#endif
}

/**
 * @brief Get system time in microseconds
 */
uint32_t get_microseconds(void)
{
    return TIM6->CNT;
}

/**
 * @brief Get system time in milliseconds
 */
uint32_t get_milliseconds(void)
{
    return TIM7->CNT;
}

/**
 * @brief Delay in milliseconds
 */
void delay_ms(uint32_t ms)
{
    uint32_t start = get_milliseconds();
    while ((get_milliseconds() - start) < ms) {
#ifdef USE_WATCHDOG
        IWDG->KR = 0xAAAA;
#endif
    }
}

/**
 * @brief System initialization
 */
void system_init(void)
{
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));
#endif

    FLASH->ACR |= FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_PRFTEN;

    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
}

/**
 * @brief Error handler
 */
void Error_Handler(void)
{
    __disable_irq();
    HAL_GPIO_WritePin(LED_GPIO_PORT, LED_RED_PIN, GPIO_PIN_SET);
    while (1) {
    }
}

/**
 * @brief TIM7 interrupt handler (system tick)
 */
void TIM7_IRQHandler(void)
{
    if (TIM7->SR & TIM_SR_UIF) {
        TIM7->SR = ~TIM_SR_UIF;
    }
}

