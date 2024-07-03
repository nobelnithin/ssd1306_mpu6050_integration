#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "ssd1306.h"
#include "font8x8_basic.h"


#define WINDOW_SIZE 10
#define THRESH_LOW -1.5
#define THRESH_HIGH 0

int16_t accel_x, accel_y, accel_z;
int16_t gyro_x, gyro_y, gyro_z;
/*
 You have to set this config value with menuconfig
 CONFIG_INTERFACE

 for i2c
 CONFIG_MODEL
 CONFIG_SDA_GPIO
 CONFIG_SCL_GPIO
 CONFIG_RESET_GPIO

 for SPI
 CONFIG_CS_GPIO
 CONFIG_DC_GPIO
 CONFIG_RESET_GPIO
*/

#define TAG "SSD1306"
SSD1306_t dev;

float moving_average_filter(float *window, int window_size) {
    float sum = 0;
    for (int i = 0; i < window_size; i++) {
        sum += window[i];
    }
    return sum / window_size;
}

bool peak_detected(float curr_val, float prev_val)
{
	if(curr_val < prev_val)
		return true;
	return false;
}
bool trough_detected(float curr_val, float prev_val)
{
	if(curr_val > prev_val)
		return true;
	return false;
}


void mpu6050_get_data(void *params)
{
    while (1) {
        mpu6050_read_data(&accel_x, &accel_y, &accel_z, &gyro_x, &gyro_y, &gyro_z);
        printf("2000 %d %d %d -2000\n",gyro_x,gyro_y,gyro_z);

        vTaskDelay(100 / portTICK_PERIOD_MS); // Delay for 1 second
    }


}

void detect_gait(void *params)
{
	uint8_t data[6];
	float GyroX, GyroY, GyroZ;
	float gyroAngleX, gyroAngleY;
	gyroAngleX = 0;
	gyroAngleY = 0;
	unsigned long oldmillis;
	oldmillis = esp_timer_get_time();
	float dt;
	float window[WINDOW_SIZE] = {0};
    int window_index = 0;
	float prev_value = 0.0;
	float curr_value = 0.0;
	bool p_detect_flag = true;
	bool t_detect_flag = true;
	while(1) {

        // if(animation_running)
        //     {
            GyroX = (float)gyro_x/131.0;
		    GyroY =  (float)gyro_y/131.0;
		    GyroZ =  (float)gyro_z/131.0;
		    
    
		    dt = (esp_timer_get_time() - oldmillis)/1000; 
		    oldmillis = esp_timer_get_time();
		    
		    gyroAngleX = GyroX * (dt/1000);
  		    gyroAngleY = GyroY * (dt/1000);
		    window[window_index] = gyroAngleY;
		    float filtered_gyroAngleY = moving_average_filter(window, WINDOW_SIZE);
		    window_index = (window_index + 1) % WINDOW_SIZE;
		    prev_value = curr_value;
		    curr_value = filtered_gyroAngleY;
		    //printf("%f\n",filtered_gyroAngleY);
    
		    if(curr_value > 3)
		    {
		    	if(trough_detected(curr_value, prev_value) && t_detect_flag)
		    	{
    
		    			printf("->->->->->->->->Fire ON<-<-<-<-<-<-<-<-\n");
						ssd1306_clear_screen(&dev, true);
		    			t_detect_flag = false;
		    		    p_detect_flag = true;
		    
		    		
		    
    
		    	}
		    	//gpio_set_level(green_led, 0);
		    }
		    if(curr_value < THRESH_HIGH)
		    {
		    	if(peak_detected(curr_value, prev_value) && p_detect_flag)
		    	{
		    		//stim off
		    		printf("Fire OFF\n");
		    		ssd1306_clear_screen(&dev, false);
		    		p_detect_flag = false;
		    		t_detect_flag = true;
		    	}
		    	//gpio_set_level(blue_led, 0);
		    }

            // }

		vTaskDelay(100/portTICK_PERIOD_MS);
	}
}





void app_main(void)
{
    	

#if CONFIG_I2C_INTERFACE
	ESP_LOGI(TAG, "INTERFACE is i2c");
	ESP_LOGI(TAG, "CONFIG_SDA_GPIO=%d",CONFIG_SDA_GPIO);
	ESP_LOGI(TAG, "CONFIG_SCL_GPIO=%d",CONFIG_SCL_GPIO);
	ESP_LOGI(TAG, "CONFIG_RESET_GPIO=%d",CONFIG_RESET_GPIO);
	i2c_master_init(&dev, CONFIG_SDA_GPIO, CONFIG_SCL_GPIO, CONFIG_RESET_GPIO);
#endif // CONFIG_I2C_INTERFACE

#if CONFIG_FLIP
	dev._flip = true;
	ESP_LOGW(TAG, "Flip upside down");
#endif

#if CONFIG_SSD1306_128x64
	ESP_LOGI(TAG, "Panel is 128x64");
	ssd1306_init(&dev, 128, 64);
    mpu6050_init();
#endif // CONFIG_SSD1306_128x64
#if CONFIG_SSD1306_128x32
	ESP_LOGI(TAG, "Panel is 128x32");
	ssd1306_init(&dev, 128, 32);
#endif // CONFIG_SSD1306_128x32

	ssd1306_contrast(&dev, 0xff);
    ssd1306_clear_screen(&dev, false);
    // while(1)
    // {
    //     mpu6050_read_data(&accel_x, &accel_y, &accel_z, &gyro_x, &gyro_y, &gyro_z);
    //     printf("2000 %d %d %d -2000\n",gyro_x,gyro_y,gyro_z);

	// 	if(gyro_y>5000)
	// 	{
	// 		ssd1306_clear_screen(&dev, true);
	// 		vTaskDelay(500/portTICK_PERIOD_MS);
	// 	}

	// 	ssd1306_clear_screen(&dev, false);
	// 	vTaskDelay(100/portTICK_PERIOD_MS);
    // }

	xTaskCreate(mpu6050_get_data, "Mpu6050",8000, NULL, 1, NULL);
    xTaskCreate(detect_gait, "detect_gait",8192, NULL, 1, NULL);


}