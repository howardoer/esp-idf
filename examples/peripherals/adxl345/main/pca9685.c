#include <stdio.h>
#include "driver/i2c.h"
#include "acc_serve.h"

#define PCA9685_ADDR 0x80//  1+A5+A4+A3+A2+A1+A0+w/r 
#define PCA9685_WRITE PCA9685_ADDR
#define PCA9685_READ PCA9685_ADDR | 1

#define PCA9685_SUBADR1 0x2
#define PCA9685_SUBADR2 0x3
#define PCA9685_SUBADR3 0x4


#define PCA9685_MODE1 0x0
#define PCA9685_PRESCALE 0xFE

#define PCA9685_LED0_ON_L 0x6
#define PCA9685_LED0_ON_H 0x7
#define PCA9685_LED0_OFF_L 0x8
#define PCA9685_LED0_OFF_H 0x9

#define PCA9685_LEDn_ON_L(n) PCA9685_LED0_ON_L + 4*(n)
#define PCA9685_LEDn_ON_H(n) PCA9685_LED0_ON_H + 4*(n)
#define PCA9685_LEDn_OFF_L(n) PCA9685_LED0_OFF_L + 4*(n)
#define PCA9685_LEDn_OFF_H(n) PCA9685_LED0_OFF_H + 4*(n)


#define PCA9685_ALLLED_ON_L 0xFA
#define PCA9685_ALLLED_ON_H 0xFB
#define PCA9685_ALLLED_OFF_L 0xFC
#define PCA9685_ALLLED_OFF_H 0xFD


#define SERVO_MIN  1000 // this is the 'minimum' pulse length count (out of 4096)
#define SERVO_MAX  2000 // this is the 'maximum' pulse length count (out of 4096)


#define SERVO_DELAY ((4096-SERVO_MAX)/MAX_SERVOS - 2)

static esp_err_t __pca9685_read_or_write_reg(uint8_t reg_addr, uint8_t* buffer, uint8_t op)
{
    esp_err_t ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, PCA9685_WRITE, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);

    if (op == PCA9685_WRITE) {
        i2c_master_write_byte(cmd, *buffer, ACK_CHECK_EN);
    } else {
        // must send start signal before read
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, PCA9685_READ, ACK_CHECK_EN);
        i2c_master_read_byte(cmd, buffer, NACK_VAL);      
    }
    i2c_master_stop(cmd);

    ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    
    return ret;
}

static inline esp_err_t pca9685_write_reg(uint8_t reg_addr, uint8_t data)
{
    esp_err_t ret = __pca9685_read_or_write_reg(reg_addr, &data, PCA9685_WRITE);
    printf("write data %d\n", data); 
    if (ret != ESP_OK)
        printf("Write pca9685 reg failed\n");
    return ret; 
}

static inline esp_err_t pca9685_read_reg(uint8_t reg_addr, uint8_t* buffer)
{
    esp_err_t ret =  __pca9685_read_or_write_reg(reg_addr, buffer, PCA9685_READ);
    printf("Read data %d\n", *buffer);
    if (ret != ESP_OK)
        printf("Read pca9685 reg failed\n");
    return ret;
}

/**
 * @brief pca9685 initialization
 */
esp_err_t pca9685_init(void)
{
	uint8_t temp;

	if(ESP_OK != pca9685_read_reg(PCA9685_MODE1, &temp)) {
		goto __failed;
	}
	pca9685_write_reg(PCA9685_MODE1, temp|0x10); //go to sleep
	pca9685_write_reg(PCA9685_PRESCALE, 0x18); //244.14Hz, then one tick = 1us
	pca9685_write_reg(PCA9685_MODE1, 0xa1); //restart and go to normal mode
	delay_ms(2);

	return ESP_OK;

__failed:
	printf("initialization failed\n");
	return ESP_FAIL;
}

void pca9685_set_pwm(uint8_t pin_num, uint16_t width_us)
{
	uint16_t on = SERVO_DELAY*pin_num;
	uint16_t off = SERVO_DELAY*pin_num + width_us;

	pca9685_write_reg(PCA9685_LEDn_ON_L(pin_num), on);
	pca9685_write_reg(PCA9685_LEDn_ON_H(pin_num), on >> 8);
	pca9685_write_reg(PCA9685_LEDn_OFF_L(pin_num), off);
	pca9685_write_reg(PCA9685_LEDn_OFF_H(pin_num), off >> 8);
	delay_ms(2); //for safty, untest
}
