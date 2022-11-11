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
#define GPIO_BIT_MASK  (1ULL<<IR)
// #define TAG IR

/*
 * weights given to respective line sensor
 */
gpio_config_t io_conf;
	io_conf.intr_type = GPIO_INTR_DISABLE;
	io_conf.mode = GPIO_MODE_INPUT;
	io_conf.pin_bit_mask = GPIO_BIT_MASK;
	io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
	io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
	gpio_config(&io_conf);
int weights[4] = {3,1,-1,-3};

/*
 * Motor value boundts
 */
int optimum_duty_cycle = 49;
int lower_duty_cycle = 39;
int higher_duty_cycle = 59;
float left_duty_cycle = 0, right_duty_cycle = 0;

/*
 * Line Following PID Variables
 */
float error=0, prev_error=0, difference, cumulative_error, correction;

/*
 * Union containing line sensor readings
 */
line_sensor_array line_sensor_readings;


void calculate_correction()
{
    error = error*10;  // we need the error correction in range 0-100 so that we can send it directly as duty cycle paramete
    difference = error - prev_error;
    cumulative_error += error;

    cumulative_error = bound(cumulative_error, -30, 30);

    correction = read_pid_const().kp*error + read_pid_const().ki*cumulative_error + read_pid_const().kd*difference;
    prev_error = error;
}

void calculate_error()
{
    int all_black_flag = 1; // assuming initially all black condition
    float weighted_sum = 0, sum = 0; 
    float pos = 0;
    
    
    

    for(int i = 0; i < 4; i++)
    {
        if(line_sensor_readings.adc_reading[i] > BLACK_MARGIN)
        {
            all_black_flag = 0;
        }
        weighted_sum += (float)(weights[i]) * (line_sensor_readings.adc_reading[i]);
        sum = sum + line_sensor_readings.adc_reading[i];
    }

    if(sum != 0) // sum can never be 0 but just for safety purposes
    {
        pos = weighted_sum / sum; // This will give us the position wrt line. if +ve then bot is facing left and if -ve the bot is facing to right.
    }

    if(all_black_flag == 1)  // If all black then we check for previous error to assign current error.
    {
        if(prev_error > 0)
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


void line_follow_task(void* arg)
{
    ESP_ERROR_CHECK(enable_motor_driver(a, NORMAL_MODE));
    ESP_ERROR_CHECK(enable_line_sensor());
    int node = 0;
    int left = 0;
    int right = 0;
    int black = 0;
    int prev_turn = 0;
    int inversion = 0;
    gpio_set_direction(IR, GPIO_MODE_INPUT);
    int ir = gpio_get_level(IR);
    while(true)
    {
        weights[0] = 3;
        weights[1] = 1;
        weights[2] = -1;
        weights[3] = -3;

        line_sensor_readings = read_line_sensor();
        calculate_error();
        calculate_correction();
        printf("the value of ir is %d",ir);
        
        
        left_duty_cycle = bound((optimum_duty_cycle - correction), lower_duty_cycle, higher_duty_cycle);
        right_duty_cycle = bound((optimum_duty_cycle + correction), lower_duty_cycle, higher_duty_cycle);

        

        
        if(line_sensor_readings.adc_reading[0] > 2000  &&  line_sensor_readings.adc_reading[1] > 2000 && line_sensor_readings.adc_reading[2] > 2000  &&  line_sensor_readings.adc_reading[3] > 2000 ){
            node = 1;
            left = 0;
            right = 0;
            black = 0;
            inversion = 0;
        }
        else if(line_sensor_readings.adc_reading[0] > 2000  &&   line_sensor_readings.adc_reading[3] < 2000 ){
            node = 0;
            left = 1;
            right = 0;
            black = 0;
            prev_turn = -1;
            inversion = 0;
        }
        else if(line_sensor_readings.adc_reading[0] < 2000  &&  line_sensor_readings.adc_reading[3] > 2000  ){
            node = 0;
            left = 0;
            right = 1;
            black = 0;
            prev_turn = 1;
            inversion = 0;
        }
        else if(line_sensor_readings.adc_reading[0] < 2000  &&  line_sensor_readings.adc_reading[1] < 2000 && line_sensor_readings.adc_reading[2] < 2000  &&  line_sensor_readings.adc_reading[3] < 2000){
            node = 0;
            left = 0;
            right = 0;
            black = 1;
            inversion = 0;
        }
        else if(line_sensor_readings.adc_reading[1] < 2000 && line_sensor_readings.adc_reading[2] < 2000 ){
            node = 0;
            left = 0;
            right = 0;
            black = 0;
            printf("Blind");
            {   
                while (!(line_sensor_readings.adc_reading[0] < 2000  &&  line_sensor_readings.adc_reading[1] > 2000 && line_sensor_readings.adc_reading[2] > 2000  &&  line_sensor_readings.adc_reading[3] < 2000))
                {
                    printf("Inversion loop is activated\n") ;
                }
            }
        }
        else{
            node = 0;
            left = 0;
            right = 0;
            black = 0;
            inversion = 0;
        }

        if(node == 1){
            
            printf("Node + Left");
            set_motor_speed(MOTOR_A_0, MOTOR_BACKWARD, left_duty_cycle * 1.5);
            set_motor_speed(MOTOR_A_1, MOTOR_FORWARD, right_duty_cycle * 1.5);
        }
        else if(left == 1){
            
            printf("Left");
            set_motor_speed(MOTOR_A_0, MOTOR_BACKWARD, left_duty_cycle );
            set_motor_speed(MOTOR_A_1, MOTOR_FORWARD, right_duty_cycle );
        }
        else if(right == 1){
            
            printf("Right");
            set_motor_speed(MOTOR_A_0, MOTOR_FORWARD, left_duty_cycle );
            set_motor_speed(MOTOR_A_1, MOTOR_BACKWARD, right_duty_cycle);
        }
        else if(black == 1){
            printf("Black");
            set_motor_speed(MOTOR_A_0, MOTOR_FORWARD, left_duty_cycle);
            set_motor_speed(MOTOR_A_1, MOTOR_BACKWARD, right_duty_cycle);
        }
        else{
            printf("Straight");
            set_motor_speed(MOTOR_A_0, MOTOR_FORWARD, left_duty_cycle);
            set_motor_speed(MOTOR_A_1, MOTOR_FORWARD, right_duty_cycle);
        }
        //ESP_LOGI("debug","left_duty_cycle:  %f    ::  right_duty_cycle :  %f  :: error :  %f  correction  :  %f  \n",left_duty_cycle, right_duty_cycle, error, correction);
        ESP_LOGI("debug", "KP: %f ::  KI: %f  :: KD: %f", read_pid_const().kp, read_pid_const().ki, read_pid_const().kd);

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}

void app_main()
{  
    

    xTaskCreate(&line_follow_task, "line_follow_task", 4096, NULL, 1, NULL);
    start_tuning_http_server();
}
