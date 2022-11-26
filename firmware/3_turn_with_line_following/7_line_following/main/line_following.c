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
int optimum_duty_cycle = 56;
int lower_duty_cycle = 46;
int higher_duty_cycle = 66;
float left_duty_cycle = 0, right_duty_cycle = 0;

/*
 * Line Following PID Variables
 */
float error = 0, prev_error = 0, difference, cumulative_error, correction;

/*
 * Union containing line sensor readings
 */
line_sensor_array line_sensor_readings;

/*
north is 1
east is 2
south is 3
west is 4
*/
int actual_path[] = {1, 2, 3, 2, 2, 1, 2, 3, 4, 1, 4, 1, 1, 2};
int k = 1; // traversing variable

int node = 0;
int left = 0;
int right = 0;

int prev_left = 0;
int prev_right = 0;
int prev_node = 0;

int left_count = 0;
int right_count = 0;
int node_count = 0;

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
    while (true)
    {
        line_sensor_readings = read_line_sensor();

        /*case of inversion*/
        if (line_sensor_readings.adc_reading[0] > 2000 && line_sensor_readings.adc_reading[1] < 2000 && line_sensor_readings.adc_reading[2] < 2000 && line_sensor_readings.adc_reading[3] > 2000)
        {
            weights[0] = -2;
            weights[1] = -5;
            weights[2] = 5;
            weights[3] = 2;
            line_sensor_readings = read_line_sensor();
            for (int i = 0; i < 4; i++)
            {
                line_sensor_readings.adc_reading[i] = bound(line_sensor_readings.adc_reading[i], BLACK_MARGIN, WHITE_MARGIN);
                line_sensor_readings.adc_reading[i] = map(line_sensor_readings.adc_reading[i], BLACK_MARGIN, WHITE_MARGIN, bound_LSA_LOW, bound_LSA_HIGH);
            }

            calculate_error();
            calculate_correction();

            left_duty_cycle = bound((optimum_duty_cycle - correction), lower_duty_cycle, higher_duty_cycle);
            right_duty_cycle = bound((optimum_duty_cycle + correction), lower_duty_cycle, higher_duty_cycle);

            set_motor_speed(MOTOR_A_0, MOTOR_FORWARD, left_duty_cycle);
            set_motor_speed(MOTOR_A_1, MOTOR_FORWARD, right_duty_cycle);
        }
        else
        {
            weights[0] = 3;
            weights[1] = 1;
            weights[2] = -1;
            weights[3] = -3;
            line_sensor_readings = read_line_sensor();
            calculate_error();
            calculate_correction();

            left_duty_cycle = bound((optimum_duty_cycle - correction), lower_duty_cycle, higher_duty_cycle);
            right_duty_cycle = bound((optimum_duty_cycle + correction), lower_duty_cycle, higher_duty_cycle);

            set_motor_speed(MOTOR_A_0, MOTOR_FORWARD, left_duty_cycle);
            set_motor_speed(MOTOR_A_1, MOTOR_FORWARD, right_duty_cycle);

            if (line_sensor_readings.adc_reading[0] > 2000 && line_sensor_readings.adc_reading[1] > 2000 && line_sensor_readings.adc_reading[2] > 2000 && line_sensor_readings.adc_reading[3] > 2000)
            {

                node = 1; // nodal case
                node_count++;
            }
            else if (line_sensor_readings.adc_reading[0] > 2000 && line_sensor_readings.adc_reading[3] < 2000)
            {
                node = 1; // left case
                left_count++;
            }
            else if (line_sensor_readings.adc_reading[0] < 2000 && line_sensor_readings.adc_reading[3] > 2000)
            {
                node = 1; // right case
                right_count++;
            }
            else if (line_sensor_readings.adc_reading[0] < 2000 && line_sensor_readings.adc_reading[1] > 2000 && line_sensor_readings.adc_reading[2] > 2000 && line_sensor_readings.adc_reading[3] < 2000)
            {
                node = 0;      // normal case
                prev_node = 0; // reseting prev_node flag since we have taken turn
                //reseting all counters
                left_count = 0;
                right_count = 0;
                node_count = 0;

            }
            //prev_node also becomes 1 instead of being 0 due to inversion
            if (node == 1 && prev_node == 1 && (left_count > 4 || right_count > 4 || node_count > 4))
            {
                if (actual_path[k] - actual_path[k - 1] == -1 || actual_path[k] - actual_path[k - 1] == 3) // for left , we used to subtract 1 from the prev_value of path
                {
                    printf("Left\n");
                    set_motor_speed(MOTOR_A_0, MOTOR_BACKWARD, left_duty_cycle * 1.4);
                    set_motor_speed(MOTOR_A_1, MOTOR_FORWARD, right_duty_cycle * 1.4);
                }
                else if (actual_path[k] - actual_path[k - 1] == 0) // for straight
                {
                    printf("straight\n");
                    set_motor_speed(MOTOR_A_0, MOTOR_FORWARD, left_duty_cycle);
                    set_motor_speed(MOTOR_A_1, MOTOR_FORWARD, right_duty_cycle);
                }
                else if (actual_path[k] - actual_path[k - 1] == 1 || actual_path[k] - actual_path[k - 1] == -3) // for right , we used to add 1 from the prev_value of path
                {
                    printf("right\n");
                    set_motor_speed(MOTOR_A_0, MOTOR_FORWARD, left_duty_cycle * 1.4);
                    set_motor_speed(MOTOR_A_1, MOTOR_BACKWARD, right_duty_cycle * 1.4);
                }
                else
                {
                    continue;
                }

                k++;

                //reseting all counters
                left_count = 0;
                right_count = 0;
                node_count = 0;

                vTaskDelay(480 / portTICK_PERIOD_MS);
            }
            prev_node = node;

            ESP_LOGI("debug", "node : %d left : %d right : %d", node_count, left_count , right_count);
        }
        // ESP_LOGI("debug", "%d %d %d %d", line_sensor_readings.adc_reading[0], line_sensor_readings.adc_reading[1], line_sensor_readings.adc_reading[2], line_sensor_readings.adc_reading[3]);

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}

void app_main()
{
    xTaskCreate(&line_follow_task, "line_follow_task", 4096, NULL, 1, NULL);
    start_tuning_http_server();
}
