#include "hall.h"

#include <stdint.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/adc.h"

/*
To detect when the lighthouse is at 12'o'clock a hall sensor is sensing a neodymium magnet.

ESP-PICO-KIT ESP32 PICO D4

Pin	Description
IO34	ADC1_CH7 POT1 hooked up to hall sensor
IO35	ADC1_CH6 POT2
PCBA Schematics https://raw.githubusercontent.com/TimGremalm/LightBoxNano/master/output/LightBoxNano.pdf
OH49E https://datasheetspdf.com/pdf-file/1418337/Ouzhuo/OH49E/1
*/
#define POT1 ADC1_CHANNEL_6  // IO34
#define POT2 ADC1_CHANNEL_7  // IO35
#define PIN_Hall POT1

static const char *TAG = "FyrHall";

void halltask(void *pvParameters) {
	ESP_LOGI(TAG, "Start");

	ESP_LOGI(TAG, "Configure ADC1");
	adc1_config_width(ADC_WIDTH_BIT_DEFAULT);
	adc1_config_channel_atten(PIN_Hall, ADC_ATTEN_DB_11);

	int raw_adc = 0;
	while(1) {
		raw_adc = adc1_get_raw(PIN_Hall);
		// ESP_LOGI(TAG, "Pot 1: %d", raw_adc);
		hall_data = raw_adc;
		vTaskDelay(10 / portTICK_PERIOD_MS);
	}
}

