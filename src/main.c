/*
 * @file main.c
 * @brief Main entry point for FPV drone flight controller firmware
 * @details STM32F405 @ 168MHz, bare-metal implementation
 *
 * Safety-critical system: Motors will not spin unless armed
 * Default failsafe behavior: Cut throttle and disarm
 */

#include "main.h"
#include "system_config.h"
#include "hardware_init.h"
#include "motor_control.h"
#include "rc_receiver.h"
#include "imu_sensor.h"
#include "sensor_fusion.h"
#include "flight_control.h"
#include "telemetry.h"
#include "config_storage.h"
#include "safety.h"
#include "blackbox.h"
#include "osd.h"

/* Global system state */
system_state_t g_system_state = {
    .armed = false,
    .failsafe_active = false,
    .flight_mode = FLIGHT_MODE_ACRO,
    .battery_voltage = 0.0f,
    .loop_time_us = 0,
    .cpu_load_percent = 0
};

/* Main control loop timing */
#define CONTROL_LOOP_PERIOD_US  125     // 8kHz control loop
#define TELEMETRY_PERIOD_US     20000   // 50Hz telemetry
#define OSD_UPDATE_PERIOD_US    33333   // 30Hz OSD update

/* Timing variables */
static uint32_t last_control_loop_us = 0;
static uint32_t last_telemetry_us = 0;
static uint32_t last_osd_update_us = 0;

int main(void)
{
    /* Disable interrupts during initialization */
    __disable_irq();

    /* Initialize system hardware */
    system_init();

    /* Initialize all peripherals and modules */
    if (init_hardware() != HAL_OK) {
        /* Critical initialization failure - enter safe mode */
        enter_safe_mode();
    }

    /* Load configuration from flash */
    config_load();

    /* Initialize flight control modules */
    motor_control_init();
    rc_receiver_init();
    imu_sensor_init();
    sensor_fusion_init();
    flight_control_init();
    telemetry_init();
    osd_init();
    blackbox_init();
    safety_init();

    /* Calibrate sensors (drone must be stationary) */
    imu_calibrate_gyro(1000);  // 1000 samples for gyro bias

    /* Enable interrupts - system is ready */
    __enable_irq();

    /* Signal successful initialization */
    led_pattern(LED_PATTERN_READY);
    buzzer_pattern(BUZZER_PATTERN_STARTUP);

    /* Main control loop */
    while (1) {
        uint32_t current_time_us = get_microseconds();

        /* High-priority control loop - 8kHz (125us) */
        if ((current_time_us - last_control_loop_us) >= CONTROL_LOOP_PERIOD_US) {
            uint32_t loop_start_us = current_time_us;
            last_control_loop_us = current_time_us;

            /* Read IMU data (gyro + accelerometer) */
            imu_data_t imu_data;
            if (imu_read_sensors(&imu_data) == HAL_OK) {

                /* Update sensor fusion for attitude estimation */
                attitude_data_t attitude;
                sensor_fusion_update(&imu_data, &attitude);

                /* Get latest RC commands */
                rc_channels_t rc_channels;
                rc_receiver_update(&rc_channels);

                /* Check failsafe conditions */
                g_system_state.failsafe_active = safety_check_failsafe(&rc_channels);

                /* Run flight control PID loop */
                motor_commands_t motor_commands;
                flight_control_update(&rc_channels, &attitude, &imu_data, &motor_commands);

                /* Update motor outputs (only if armed and no failsafe) */
                if (g_system_state.armed && !g_system_state.failsafe_active) {
                    motor_control_set_outputs(&motor_commands);
                } else {
                    /* Safety: Ensure motors are stopped */
                    motor_control_stop_all();
                }

                /* Log flight data if armed */
                if (g_system_state.armed) {
                    blackbox_log_frame(&imu_data, &rc_channels, &motor_commands, &attitude);
                }
            }

            /* Calculate loop execution time for monitoring */
            g_system_state.loop_time_us = get_microseconds() - loop_start_us;

            /* Check for control loop overrun */
            if (g_system_state.loop_time_us > CONTROL_LOOP_PERIOD_US) {
                /* Loop took too long - this is a critical error */
                g_system_state.cpu_load_percent = 100;
                led_pattern(LED_PATTERN_ERROR);
            } else {
                g_system_state.cpu_load_percent = (g_system_state.loop_time_us * 100) / CONTROL_LOOP_PERIOD_US;
            }
        }

        /* Lower priority tasks */

        /* Telemetry update - 50Hz */
        if ((current_time_us - last_telemetry_us) >= TELEMETRY_PERIOD_US) {
            last_telemetry_us = current_time_us;
            telemetry_send_status();
            telemetry_process_commands();
        }

        /* OSD update - 30Hz */
        if ((current_time_us - last_osd_update_us) >= OSD_UPDATE_PERIOD_US) {
            last_osd_update_us = current_time_us;
            osd_update();
        }

        /* Handle USB/configuration if connected */
        if (usb_is_connected()) {
            usb_process_commands();
        }

        /* Monitor battery voltage */
        g_system_state.battery_voltage = read_battery_voltage();
        safety_check_battery(g_system_state.battery_voltage);

        /* Background tasks - only run if we have spare time */
        if (g_system_state.cpu_load_percent < 80) {
            /* Save configuration if pending */
            config_save_if_needed();

            /* GPS processing if available */
            #ifdef USE_GPS
            gps_process();
            #endif

            /* Barometer processing if available */
            #ifdef USE_BARO
            baro_process();
            #endif
        }
    }
}

/**
 * @brief Enter safe mode on critical failure
 * @details Disables all motors and enters infinite loop with error indication
 */
void enter_safe_mode(void)
{
    /* Ensure motors cannot spin */
    motor_control_emergency_stop();

    /* Disable interrupts except systick for timing */
    __disable_irq();
    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
    __enable_irq();

    /* Error indication */
    while (1) {
        led_toggle(LED_RED);
        buzzer_pattern(BUZZER_PATTERN_ERROR);
        delay_ms(500);
    }
}

/**
 * @brief Hard fault handler
 * @details Called on unrecoverable hardware fault
 */
void HardFault_Handler(void)
{
    /* Log fault information if possible */
    #ifdef DEBUG
    volatile uint32_t *sp = (uint32_t *)__get_MSP();
    volatile uint32_t pc = sp[6];  // Program counter at fault
    volatile uint32_t lr = sp[5];  // Link register
    #endif

    /* Enter safe mode */
    enter_safe_mode();
}

