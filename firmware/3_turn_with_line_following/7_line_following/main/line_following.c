#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sra_board.h"
#include "tuning_http_server.h"

#define MODE NORMAL_MODE
#define BLACK_MARGIN 400
#define WHITE_MARGIN 2000
#define bound_LSA_LOW 0
#define bound_LSA_HIGH 1000

/*
 * weights given to respective line sensor
 */
int weights[4] = {3, 1, -1, -3};

/*
 * Motor value boundts
 */
int optimum_duty_cycle = 55;
int lower_duty_cycle = 45;
int higher_duty_cycle = 65;
float left_duty_cycle = 0, right_duty_cycle = 0;

/*
 * Line Following PID Variables
 */
float error = 0, prev_error = 0, difference, cumulative_error, correction;

/*
 * Union containing line sensor readings
 */
line_sensor_array line_sensor_readings;

void calculate_correction()
{
    error = error * 10; // we need the error correction in range 0-100 so that we can send it directly as duty cycle paramete
    difference = error - prev_error;
    cumulative_error += error;

    cumulative_error = bound(cumulative_error, -30, 30);

    correction = read_pid_const().kp * error + read_pid_const().ki * cumulative_error + read_pid_const().kd * difference;
    prev_error = error;
}

void calculate_error()
{
    int all_black_flag = 1; // assuming initially all black condition
    float weighted_sum = 0, sum = 0;
    float pos = 0;

    for (int i = 0; i < 4; i++)
    {
        if (line_sensor_readings.adc_reading[i] > BLACK_MARGIN)
        {
            all_black_flag = 0;
        }
        weighted_sum += (float)(weights[i]) * (line_sensor_readings.adc_reading[i]);
        sum = sum + line_sensor_readings.adc_reading[i];
    }

    if (sum != 0) // sum can never be 0 but just for safety purposes
    {
        pos = weighted_sum / sum; // This will give us the position wrt line. if +ve then bot is facing left and if -ve the bot is facing to right.
    }
    if (all_black_flag == 1) // If all black then we check for previous error to assign current error.
    {
        if (prev_error > 0)
        {
            error = 2.5;
        }
        else
        {
            error = -2.5;
        }
    }
    else
    {
        error = pos;
    }
}

void line_follow_task(void *arg)
{
    ESP_ERROR_CHECK(enable_motor_driver(a, NORMAL_MODE));
    ESP_ERROR_CHECK(enable_line_sensor());
    int node = 0;
    int left = 0;
    int right = 0;
    int black = 0;
    while (true)
    {

        line_sensor_readings = read_line_sensor();
        calculate_error();
        calculate_correction();

        left_duty_cycle = bound((optimum_duty_cycle - correction), lower_duty_cycle, higher_duty_cycle);
        right_duty_cycle = bound((optimum_duty_cycle + correction), lower_duty_cycle, higher_duty_cycle);

        /*The case of inversion*/
        if (line_sensor_readings.adc_reading[0] > 2000 && line_sensor_readings.adc_reading[1] < 2000 && line_sensor_readings.adc_reading[2] < 2000 && line_sensor_readings.adc_reading[3] > 2000)
        {
            printf("Inversion detected\n") ;
            while (!(line_sensor_readings.adc_reading[0] < 2000 && line_sensor_readings.adc_reading[1] > 2000 && line_sensor_readings.adc_reading[2] > 2000 && line_sensor_readings.adc_reading[3] < 2000))
            {
                printf("INVERSION\n") ;
                ESP_LOGI("debug", "%d %d %d %d", line_sensor_readings.adc_reading[0] , line_sensor_readings.adc_reading[1] , line_sensor_readings.adc_reading[2] , line_sensor_readings.adc_reading[3]);
                ESP_LOGI("debug", "%d %d %d %d", weights[0] , weights[1] , weights[2] , weights[3]);
                /*getting sensor readings*/
                line_sensor_readings = read_line_sensor();

                /*inverting the weights*/
                weights[0] = -1;
                weights[1] = -3;
                weights[2] = 3;
                weights[3] = 1;

                // float weighted_sum = 0, sum = 0;
                // float pos = 0;

                // for (int i = 0; i < 4; i++)
                // {
                //     weighted_sum += (float)(weights[i]) * (line_sensor_readings.adc_reading[i]);
                //     sum = sum + line_sensor_readings.adc_reading[i];
                // }

                // if (sum != 0) // sum can never be 0 but just for safety purposes
                // {
                //     pos = weighted_sum / sum; // This will give us the position wrt line. if +ve then bot is facing left and if -ve the bot is facing to right.
                // }
                // else
                // {
                //     error = pos;
                // }

                // error = error * 10; // we need the error correction in range 0-100 so that we can send it directly as duty cycle paramete
                // difference = error - prev_error;
                // cumulative_error += error;

                // cumulative_error = bound(cumulative_error, -30, 30);

                // correction = read_pid_const().kp * error + read_pid_const().ki * cumulative_error + read_pid_const().kd * difference;
                // prev_error = error;

                // left_duty_cycle = bound((optimum_duty_cycle - correction), lower_duty_cycle, higher_duty_cycle);
                // right_duty_cycle = bound((optimum_duty_cycle + correction), lower_duty_cycle, higher_duty_cycle);

                vTaskDelay(100 / portTICK_PERIOD_MS);
            }
            printf("Inversion ended\n") ;
            /*inverting the weights*/
                
        }
        weights[0] = 3;
        weights[1] = 1;
        weights[2] = -1;
        weights[3] = -3;
        /*normal line follow*/
        if (line_sensor_readings.adc_reading[0] > 2000 && line_sensor_readings.adc_reading[1] > 2000 && line_sensor_readings.adc_reading[2] > 2000 && line_sensor_readings.adc_reading[3] > 2000)
        {
            node = 1;
            left = 0;
            right = 0;
            black = 0;
        }
        else if (line_sensor_readings.adc_reading[0] > 2000 && line_sensor_readings.adc_reading[1] > 2000 && line_sensor_readings.adc_reading[3] < 2000)
        {
            node = 0;
            left = 1;
            right = 0;
            black = 0;
        }
        else if (line_sensor_readings.adc_reading[0] < 2000 && line_sensor_readings.adc_reading[3] > 2000)
        {
            node = 0;
            left = 0;
            right = 1;
            black = 0;
        }
        else if (line_sensor_readings.adc_reading[0] < 2000 && line_sensor_readings.adc_reading[1] < 2000 && line_sensor_readings.adc_reading[2] < 2000 && line_sensor_readings.adc_reading[3] < 2000)
        {
            node = 0;
            left = 0;
            right = 0;
            black = 1;
        }
        else
        {
            node = 0;
            left = 0;
            right = 0;
            black = 0;
        }

        if (node == 1)
        {
            printf("Node + Left");
            set_motor_speed(MOTOR_A_0, MOTOR_BACKWARD, left_duty_cycle * 1.5);
            set_motor_speed(MOTOR_A_1, MOTOR_FORWARD, right_duty_cycle * 1.5);
        }
        else if (left == 1)
        {
            printf("Left");
            set_motor_speed(MOTOR_A_0, MOTOR_BACKWARD, left_duty_cycle);
            set_motor_speed(MOTOR_A_1, MOTOR_FORWARD, right_duty_cycle);
        }
        else if (right == 1)
        {
            printf("Right");
            set_motor_speed(MOTOR_A_0, MOTOR_FORWARD, left_duty_cycle);
            set_motor_speed(MOTOR_A_1, MOTOR_BACKWARD, right_duty_cycle);
        }
        else if (black == 1)
        {
            printf("Black");
            set_motor_speed(MOTOR_A_0, MOTOR_FORWARD, left_duty_cycle);
            set_motor_speed(MOTOR_A_1, MOTOR_BACKWARD, right_duty_cycle);
        }
        else
        {
            printf("Straight");
            set_motor_speed(MOTOR_A_0, MOTOR_FORWARD, left_duty_cycle);
            set_motor_speed(MOTOR_A_1, MOTOR_FORWARD, right_duty_cycle);
        }
        // ESP_LOGI("debug","left_duty_cycle:  %f    ::  right_duty_cycle :  %f  :: error :  %f  correction  :  %f  \n",left_duty_cycle, right_duty_cycle, error, correction);
        ESP_LOGI("debug", "%d %d %d %d", line_sensor_readings.adc_reading[0] , line_sensor_readings.adc_reading[1] , line_sensor_readings.adc_reading[2] , line_sensor_readings.adc_reading[3]);
        ESP_LOGI("debug", "%d %d %d %d", weights[0] , weights[1] , weights[2] , weights[3]);

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}

void app_main()
{

    xTaskCreate(&line_follow_task, "line_follow_task", 4096, NULL, 1, NULL);
    start_tuning_http_server();
}