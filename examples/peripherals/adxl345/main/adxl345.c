/* i2c - Example

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "driver/i2c.h"
#include "acc_serve.h"

/**
 * TEST CODE BRIEF
 *
 * This example will show you how to use I2C module by running two tasks on i2c bus:
 *
 * - read external i2c sensor, here we use a BH1750 light sensor(GY-30 module) for instance.
 * - Use one I2C port(master mode) to read or write the other I2C port(slave mode) on one ESP32 chip.
 *
 * Pin assignment:
 *
 * - master:
 *    GPIO18 is assigned as the data signal of i2c master port
 *    GPIO19 is assigned as the clock signal of i2c master port
 *
 * Connection:
 *
 * - connect sda/scl of sensor with GPIO18/GPIO19
 * - no need to add external pull-up resistors, driver will enable internal pull-up resistors.
 *
 * Test items:
 *
 * - read the sensor data, if connected.
 */


#define ADXL345_SENSOR_ADDR  0x1D    /*!< slave address for BH1750 sensor */
#define ADXL_READ           (ADXL345_SENSOR_ADDR << 1) | 1
#define ADXL_WRITE          (ADXL345_SENSOR_ADDR << 1)
 
// register map
#define ADXL_DEVICE_ID 0X00 
#define ADXL_THRESH_TAP 0X1D   
#define ADXL_OFSX 0X1E
#define ADXL_OFSY 0X1F
#define ADXL_OFSZ 0X20
#define ADXL_DUR 0X21
#define ADXL_Latent 0X22
#define ADXL_Window   0X23 
#define ADXL_THRESH_ACK 0X24
#define ADXL_THRESH_INACT        0X25 
#define ADXL_TIME_INACT 0X26
#define ADXL_ACT_INACT_CTL        0X27
#define ADXL_THRESH_FF 0X28
#define ADXL_TIME_FF 0X29 
#define ADXL_TAP_AXES 0X2A  
#define ADXL_ACT_TAP_STATUS          0X2B 
#define ADXL_BW_RATE 0X2C 
#define ADXL_POWER_CTL 0X2D 
#define ADXL_INT_ENABLE 0X2E
#define ADXL_INT_MAP 0X2F
#define ADXL_INT_SOURCE          0X30
#define ADXL_DATA_FORMAT        0X31
#define ADXL_DATA_X0 0X32
#define ADXL_DATA_X1 0X33
#define ADXL_DATA_Y0 0X34
#define ADXL_DATA_Y1 0X35
#define ADXL_DATA_Z0 0X36
#define ADXL_DATA_Z1 0X37
#define ADXL_FIFO_CTL 0X38
#define ADXL_FIFO_STATUS 0X39


static esp_err_t __adxl345_read_or_write_reg(uint8_t reg_addr, uint8_t* buffer, uint8_t op)
{
    esp_err_t ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADXL_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);

    if (op == ADXL_WRITE) {
        i2c_master_write_byte(cmd, *buffer, ACK_CHECK_EN);
    } else {
        // must send start signal before read
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, ADXL_READ, ACK_CHECK_EN);
        i2c_master_read_byte(cmd, buffer, NACK_VAL);      
    }
    i2c_master_stop(cmd);

    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

static inline esp_err_t adxl345_write_reg(uint8_t reg_addr, uint8_t data)
{
    esp_err_t ret = __adxl345_read_or_write_reg(reg_addr, &data, ADXL_WRITE);
    printf("write data %d\n", data); 
    return ret; 
}

static inline esp_err_t adxl345_read_reg(uint8_t reg_addr, uint8_t* buffer)
{
    esp_err_t ret =  __adxl345_read_or_write_reg(reg_addr, buffer, ADXL_READ);
    printf("Read data %d\n", *buffer);
    return ret;
}

esp_err_t adxl345_read_xyz(int16_t* x, int16_t* y, int16_t* z)
{
    uint8_t buffer[6];
    int i;
    esp_err_t ret;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADXL_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, ADXL_DATA_X0, ACK_CHECK_EN);
    // must send start signal before read
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADXL_READ, ACK_CHECK_EN);
    for (i = 0; i < 5; i++) {
        i2c_master_read_byte(cmd, &buffer[i], ACK_VAL); 
    }
    i2c_master_read_byte(cmd, &buffer[i], NACK_VAL);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);

    if (ret == ESP_OK) {
        *x = (int16_t)(buffer[1] << 8) + buffer[0];
        *y = (int16_t)(buffer[3] << 8) + buffer[2];
        *z = (int16_t)(buffer[5] << 8) + buffer[4];
    }

    return ret;
}

/**
 * @brief adxl345 initialization
 */
esp_err_t adxl345_init(void)
{
    uint8_t buffer;
    if (adxl345_read_reg(ADXL_DEVICE_ID, &buffer) == ESP_OK && buffer == 0xE5) {
        adxl345_write_reg(ADXL_BW_RATE,0x0A);     //数据输出速度为100Hz  
        adxl345_write_reg(ADXL_POWER_CTL,0x28);       //链接使能,测量模式  
        adxl345_write_reg(ADXL_INT_ENABLE,0x00);  //不使用中断        
        adxl345_write_reg(ADXL_OFSX,0x00);  
        adxl345_write_reg(ADXL_OFSY,0x00);  
        adxl345_write_reg(ADXL_OFSZ,0x00);
        printf("initialization adxl345 oK!\n");
        return ESP_OK; 
    } else {
        printf("initialization failed\n");
        return ESP_FAIL;
    }

}




