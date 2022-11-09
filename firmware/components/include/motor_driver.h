#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_err.h"

#define MOTOR_BACKWARD 1
#define MOTOR_FORWARD -1
#define MOTOR_STOP 0

#define MOTOR_A_0 0
#define MOTOR_A_1 1

/////// motor driver ////////
#define PWM_A 12 //MCPWM_UNIT_1 MCPWM0A
#define MDA_NORMAL_IN_1 32 //MCPWM_UNIT_1 MCPWM0B
#define MDA_NORMAL_IN_2 33 //MCPWM_UNIT_1 MCPWM1A
#define PWM_B 33 //MCPWM_UNIT_1 MCPWM1B
#define MDA_NORMAL_IN_3 25 //MCPWM_UNIT_1 MCPWM2A bin1
#define MDA_NORMAL_IN_4 26 //MCPWM_UNIT_1 MCPWM2B bin2

#define CHECK(x)                \
    do                          \
    {                           \
        esp_err_t __;           \
        if ((__ = x) != ESP_OK) \
            return __;          \
    } while (0)
#define CHECK_LOGE(err, x, tag, msg, ...)      \
    do                                         \
    {                                          \
        if ((err = x) != ESP_OK)               \
        {                                      \
            ESP_LOGE(tag, msg, ##__VA_ARGS__); \
            return err;                        \
        }                                      \
    } while (0) 

esp_err_t enable_motor_driver();
esp_err_t set_motor_speed(int motor_id, int direction, float duty_cycle);

#endif