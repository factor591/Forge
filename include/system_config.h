/**
 * @file system_config.h
 * @brief System-wide configuration and definitions
 */

#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

/* Firmware version */
#define FIRMWARE_VERSION        "1.0.0"
#define FIRMWARE_NAME          "FPV-FC"
#define TARGET_NAME            "STM32F405"

/* System frequencies */
#define SYSTEM_CLOCK_HZ        168000000UL  // 168 MHz
#define APB1_CLOCK_HZ          42000000UL   // 42 MHz
#define APB2_CLOCK_HZ          84000000UL   // 84 MHz

/* Control loop rates */
#define CONTROL_LOOP_HZ        8000         // Main PID loop frequency
#define IMU_SAMPLE_RATE_HZ     8000         // Gyro/Accel sampling
#define ATTITUDE_UPDATE_HZ     1000         // Sensor fusion rate
#define BARO_UPDATE_HZ         50           // Barometer update rate
#define GPS_UPDATE_HZ          10           // GPS update rate

/* Hardware pin definitions */
/* Motor outputs - Timer 2 channels */
#define MOTOR1_PIN             GPIO_PIN_0   // PA0 - TIM2_CH1
#define MOTOR2_PIN             GPIO_PIN_1   // PA1 - TIM2_CH2
#define MOTOR3_PIN             GPIO_PIN_2   // PA2 - TIM2_CH3
#define MOTOR4_PIN             GPIO_PIN_3   // PA3 - TIM2_CH4
#define MOTOR_GPIO_PORT        GPIOA
#define MOTOR_TIMER            TIM2

/* IMU SPI interface */
#define IMU_SPI                SPI1
#define IMU_SCK_PIN            GPIO_PIN_5   // PA5
#define IMU_MISO_PIN           GPIO_PIN_6   // PA6
#define IMU_MOSI_PIN           GPIO_PIN_7   // PA7
#define IMU_CS_PIN             GPIO_PIN_4   // PA4
#define IMU_GPIO_PORT          GPIOA
#define IMU_INT_PIN            GPIO_PIN_4   // PC4 - EXTI4
#define IMU_INT_GPIO_PORT      GPIOC

/* RC Receiver UART */
#define RC_UART                USART1
#define RC_TX_PIN              GPIO_PIN_9   // PA9
#define RC_RX_PIN              GPIO_PIN_10  // PA10
#define RC_GPIO_PORT           GPIOA

/* Telemetry/USB UART */
#define TELEM_UART             USART3
#define TELEM_TX_PIN           GPIO_PIN_10  // PB10
#define TELEM_RX_PIN           GPIO_PIN_11  // PB11
#define TELEM_GPIO_PORT        GPIOB

/* OSD SPI interface (if analog OSD) */
#define OSD_SPI                SPI2
#define OSD_SCK_PIN            GPIO_PIN_13  // PB13
#define OSD_MISO_PIN           GPIO_PIN_14  // PB14
#define OSD_MOSI_PIN           GPIO_PIN_15  // PB15
#define OSD_CS_PIN             GPIO_PIN_12  // PB12
#define OSD_GPIO_PORT          GPIOB

/* Status LEDs */
#define LED_RED_PIN            GPIO_PIN_0   // PC0
#define LED_GREEN_PIN          GPIO_PIN_1   // PC1
#define LED_BLUE_PIN           GPIO_PIN_2   // PC2
#define LED_GPIO_PORT          GPIOC

/* Buzzer */
#define BUZZER_PIN             GPIO_PIN_3   // PC3
#define BUZZER_GPIO_PORT       GPIOC

/* Battery voltage ADC */
#define VBAT_ADC               ADC1
#define VBAT_ADC_CHANNEL       ADC_CHANNEL_8
#define VBAT_PIN               GPIO_PIN_0   // PB0
#define VBAT_GPIO_PORT         GPIOB
#define VBAT_SCALE             11.0f        // Voltage divider ratio

/* Flash memory layout */
#define FLASH_SECTOR_CONFIG    FLASH_SECTOR_11  // Last sector for config
#define FLASH_CONFIG_ADDRESS   0x080E0000       // Sector 11 address
#define FLASH_CONFIG_SIZE      16384            // 16KB for config

/* DMA assignments */
#define MOTOR_DMA              DMA1
#define MOTOR_DMA_STREAM_M1    DMA1_Stream5    // TIM2_CH1
#define MOTOR_DMA_STREAM_M2    DMA1_Stream6    // TIM2_CH2
#define MOTOR_DMA_STREAM_M3    DMA1_Stream1    // TIM2_CH3
#define MOTOR_DMA_STREAM_M4    DMA1_Stream7    // TIM2_CH4

#define UART_RX_DMA            DMA2
#define RC_UART_DMA_STREAM     DMA2_Stream2    // USART1_RX
#define TELEM_UART_DMA_STREAM  DMA1_Stream1    // USART3_RX

/* Safety limits */
#define MIN_THROTTLE           1000    // Minimum PWM value
#define MAX_THROTTLE           2000    // Maximum PWM value
#define MOTOR_MIN_COMMAND      1000    // Minimum motor command
#define MOTOR_MAX_COMMAND      2000    // Maximum motor command
#define ARM_MIN_THROTTLE       1100    // Max throttle to allow arming

/* Failsafe settings */
#define FAILSAFE_DELAY_MS      500     // Time before failsafe triggers
#define FAILSAFE_THROTTLE      1000    // Failsafe throttle value (minimum)
#define RX_TIMEOUT_MS          50      // Max time between valid RC frames

/* Flight modes */
typedef enum {
    FLIGHT_MODE_ACRO = 0,      // Rate mode - direct rate control
    FLIGHT_MODE_ANGLE,         // Self-leveling mode
    FLIGHT_MODE_HORIZON,       // Mixed mode
    FLIGHT_MODE_ALT_HOLD,      // Altitude hold using barometer
    FLIGHT_MODE_GPS_HOLD,      // GPS position hold
    FLIGHT_MODE_GPS_RESCUE,    // Return to home
    FLIGHT_MODE_FAILSAFE       // Failsafe active
} flight_mode_t;

/* System state structure */
typedef struct {
    bool armed;
    bool failsafe_active;
    flight_mode_t flight_mode;
    float battery_voltage;
    uint32_t loop_time_us;
    uint8_t cpu_load_percent;
} system_state_t;

/* Global system state */
extern system_state_t g_system_state;

/* Utility macros */
#define CONSTRAIN(x, min, max)  ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
#define ABS(x)                  ((x) < 0 ? -(x) : (x))
#define MIN(a, b)               ((a) < (b) ? (a) : (b))
#define MAX(a, b)               ((a) > (b) ? (a) : (b))
#define DEG_TO_RAD              0.017453292519943295f
#define RAD_TO_DEG              57.29577951308232f

/* Debug/development features */
#ifdef DEBUG
    #define DEBUG_PRINT(fmt, ...) printf(fmt, ##__VA_ARGS__)
#else
    #define DEBUG_PRINT(fmt, ...)
#endif

#endif /* SYSTEM_CONFIG_H */

