#include "jaguar.h"

/*This method involves tuning kp , ki ,kd physically*/
#define GOOD_DUTY_CYCLE 40
#define MIN_DUTY_CYCLE 30
#define MAX_DUTY_CYCLE 50

int actiavate_left_counter = 0;
int actiavate_right_counter = 0;
int Turn;
float error = 0, prev_error = 0, difference, cumulative_error, correction;
float left_duty_cycle = 0, right_duty_cycle = 0;

void calculate_correction();
void calculate_error();
float bound(float val, float min, float max);

void line_follow_task(void *arg)
{

    while (1)
    {
        get_raw_lsa();

        calculate_error();
        calculate_correction();

        left_duty_cycle = bound((GOOD_DUTY_CYCLE - correction), MIN_DUTY_CYCLE, MAX_DUTY_CYCLE);
        right_duty_cycle = bound((GOOD_DUTY_CYCLE + correction), MIN_DUTY_CYCLE, MAX_DUTY_CYCLE);

        set_motor_speed(MOTOR_A_0, MOTOR_FORWARD, left_duty_cycle);
        set_motor_speed(MOTOR_A_1, MOTOR_FORWARD, right_duty_cycle);

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}
// end of task

void app_main()
{
    ESP_ERROR_CHECK(enable_lsa());
    ESP_ERROR_CHECK(enable_motor_driver());

    xTaskCreate(&line_follow_task, "line_follow_task", 4096, NULL, 1, NULL); // creating a task to start line following
}
// end of main

void calculate_error()
{
    /*
    possible cases of errors :-
    1) All the front sensor read black - this means the bot is entirely out of path
    2) if lsa_reading [1] != lsa_reading [2] - this means bot is slightly out of path

    error to right of line is +ve while left of line is -ve
    */

    int error_arr[4] = {-4, -1, +1, +4};

    error = (error_arr[0] * lsa_reading[0] + error_arr[1] * lsa_reading[1] + error_arr[2] * lsa_reading[2] + error_arr[3] * lsa_reading[3]) / (lsa_reading[0] + lsa_reading[1] + lsa_reading[2] + lsa_reading[3]);

    if (lsa_reading[0] == 0 && lsa_reading[1] == 0 && lsa_reading[2] == 0 && lsa_reading[3] == 0)
    {
        set_motor_speed(MOTOR_A_0, MOTOR_STOP, 100);
        set_motor_speed(MOTOR_A_1, MOTOR_STOP, 100);
        vTaskDelay(100 / portTICK_PERIOD_MS);
        /*All sensors read black*/
        // if (prev_error > 0) // we use prev_error to extract the sign to give to error
        // {
        //     error = 2.5; // chnage
        // }
        // else
        // {
        //     error = -2.5; // change
        // }
    }

    if (lsa_reading[0] == 1000 && lsa_reading[1] == 1000 && lsa_reading[2] == 1000)
    {
        set_motor_speed(MOTOR_A_0, MOTOR_FORWARD, 70);
        set_motor_speed(MOTOR_A_1, MOTOR_BACKWARD, 70);

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
// end of function

void calculate_correction()
{
    error = error * 10;              // we need the error correction in range 0-100 so that we can send it directly as duty cycle paramete
    difference = error - prev_error; // used for calcuating kd
    cumulative_error += error;       // used for calculating ki

    cumulative_error = bound(cumulative_error, -30, 30); // bounding cumulative_error to avoid the issue of cumulative_error being very large

    float kp, ki, kd;
    kp = 5;
    kd = 2;
    ki = 0;

    correction = kp * error + ki * cumulative_error + kd * difference; // defined in http_server.c

    prev_error = error; // update error
}
// end of function

float bound(float val, float min, float max) // To bound a certain value in range MAX to MIN
{
    if (val > max)
        val = max;
    else if (val < min)
        val = min;
    return val;
}
// end of function
