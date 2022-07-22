#include "controller.h"

#include <stdint.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "dmx.h"
#include "hall.h"

static const char *TAG = "FyrController";

void controllertask(void *pvParameters) {
	ESP_LOGI(TAG, "Start");
	while(1) {
		ESP_LOGI(TAG, "DMX Channel 1: %d", dmx_data[1]);
		// ESP_LOGI(TAG, "DMX Channel 2: %d", dmx_data[2]);
		// ESP_LOGI(TAG, "DMX Channel 512: %d", dmx_data[512]);
		ESP_LOGI(TAG, "Hall sensor: %d", hall_data);
		vTaskDelay(2000 / portTICK_PERIOD_MS);
	}
}

