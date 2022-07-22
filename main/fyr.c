/* Fyr
A controller for a lighthouse with an rotating LED assembly.
Recive DMX512 over a RS-485 serial connection.

tim@gremalm.se
http://tim.gremalm.se
*/
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "dmx.h"
#include "controller.h"
#include "hall.h"
#include "light.h"
#include "motor.h"

// Init global variable
uint8_t dmx_data[513];
uint16_t hall_data;

// Declare thread pin to core
#define CORE_APP 0
#define CORE_IMP 1

static const char *TAG = "FyrMain";

void app_main(void) {
	ESP_LOGI(TAG, "Start FYR");

	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

	xTaskCreatePinnedToCore(&dmxtask, "dmx_task", 4096, NULL, 5, NULL, CORE_APP);
	xTaskCreatePinnedToCore(&controllertask, "controller_task", 4096, NULL, 5, NULL, CORE_APP);
	xTaskCreatePinnedToCore(&halltask, "hall_task", 4096, NULL, 5, NULL, CORE_APP);
	xTaskCreatePinnedToCore(&lighttask, "light_task", 4096, NULL, 5, NULL, CORE_APP);
	xTaskCreatePinnedToCore(&motortask, "motor_task", 4096, NULL, 5, NULL, CORE_IMP);
}

