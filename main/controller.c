#include "controller.h"

#include <stdint.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "dmx.h"

static const char *TAG = "FyrController";

void controllertask(void *pvParameters) {
	ESP_LOGI(TAG, "Start");
	while(1) {
		// ESP_LOGI(TAG, "Controller loop");
		ESP_LOGI(TAG, "%d", dmx_data[1]);
		vTaskDelay(2000 / portTICK_PERIOD_MS);
	}
}

