#include "jaguar.h"
//this code will print the readings of LSA on terminal

void app_main(void)
{
    ESP_ERROR_CHECK(enable_lsa()) ; //check LSA 

    while (1)
    {
        get_raw_lsa() ; //prints the value of readings 
        vTaskDelay(100 / portTICK_PERIOD_MS); 
    }
}