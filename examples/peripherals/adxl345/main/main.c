/* This is acc_serve.c */

#include <stdio.h>
#include "acc_serve.h"

xSemaphoreHandle print_mux;
xQueueHandle acc_queue;

typedef struct acc_data {
	int16_t x;
	int16_t y;
} acc_data_t;

void i2c_adxl345_task(void* arg)
{
    int ret;
    uint32_t task_idx = (uint32_t) arg;
    
    int16_t x, y, z;
    acc_data_t acc;

    printf("*******************\n");
    printf("TASK[%d]  MASTER READ SENSOR( ADXL345 )\n", task_idx);
    printf("*******************\n");

    if (ESP_OK != adxl345_init()) {
        xSemaphoreTake(print_mux, portMAX_DELAY);
        printf("Failed to initialize ADXL345, task failed");
    }

    while (1) {
        ret = adxl345_read_xyz(&x, &y, &z);
        // take semaphore for print is instrude.
        xSemaphoreTake(print_mux, portMAX_DELAY);
        if (ret == ESP_OK) {
            printf("x, y, z: %6d, %6d, %6d\n", x, y, z);
            acc.x = x;
            acc.y = y;
            xQueueSendToBack(acc_queue, &acc, 0);
        } else {
            printf("No ack, sensor not connected...skip...\n");
        }
        xSemaphoreGive(print_mux);
        
        vTaskDelay(( DELAY_TIME_BETWEEN_ITEMS_MS * ( task_idx + 1 ) ) / portTICK_RATE_MS);
    }
}

void i2c_pca9685_task(void* arg)
{
    uint32_t task_idx = (uint32_t) arg;
    acc_data_t acc;

    printf("*******************\n");
    printf("TASK[%d]  Servo running\n", task_idx);
    printf("*******************\n");

    if (ESP_OK != pca9685_init()) {
        xSemaphoreTake(print_mux, portMAX_DELAY);
        printf("Failed to initialize PCA9685, task failed");
    }

    while (1) {
    	if (pdTRUE == xQueueReceive(acc_queue, &acc, 0)) {
    		pca9685_set_pwm(0, 2*acc.x + 1500);
    		pca9685_set_pwm(1, 2*acc.y + 1500);
    		// take semaphore for print is instrude.
	        xSemaphoreTake(print_mux, portMAX_DELAY);
	        printf("Servo is running, with x: %4d, y: %4d\n", 2*acc.x + 1500, 2*acc.y + 1500);
	        xSemaphoreGive(print_mux);
    	}
        vTaskDelay(( DELAY_TIME_BETWEEN_ITEMS_MS * ( task_idx + 1 ) ) / portTICK_RATE_MS);
    }	
}

void app_main()
{
    print_mux = xSemaphoreCreateMutex();
    acc_queue = xQueueCreate(5, sizeof(acc_data_t));
    i2c_master_init();

    xTaskCreate(i2c_adxl345_task, "i2c_adxl345", 1024 * 2, (void* ) 0, 10, NULL);
    xTaskCreate(i2c_pca9685_task, "i2c_pca9685", 1024 * 2, (void*) 0, 10, NULL);
}