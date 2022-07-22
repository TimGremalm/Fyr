#include "motor.h"

#include <stdint.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/ledc.h"
#include "driver/gpio.h"

// #include "dmx.h"

/*
A A3967SLB Stepper Driver rotates the LED assembly.

ESP-PICO-KIT ESP32 PICO D4

Pin	Description
IO25	Stepper Step
IO18	Stepper Enable
IO5		Stepper MS1
IO10	Stepper MS2
IO9		Stepper Dir
PCBA Schematics https://raw.githubusercontent.com/TimGremalm/LightBoxNano/master/output/LightBoxNano.pdf
*/
#define STEPPER_STEP_PIN 25
#define STEPPER_ENABLE_PIN 18
#define STEPPER_MS1_PIN 5
#define STEPPER_MS2_PIN 10
#define STEPPER_DIR_PIN 9

static const char *TAG = "FyrMotor";

void motortask(void *pvParameters) {
	ESP_LOGI(TAG, "Start");
	gpio_reset_pin(STEPPER_ENABLE_PIN);  // Set pad driver capability to GPIO
	gpio_set_direction(STEPPER_ENABLE_PIN, GPIO_MODE_OUTPUT);  // Set the GPIO as a push/pull output
	gpio_set_level(STEPPER_ENABLE_PIN, 0);  // Low to enable receiver

	gpio_reset_pin(STEPPER_MS1_PIN);  // Set pad driver capability to GPIO
	gpio_set_direction(STEPPER_MS1_PIN, GPIO_MODE_OUTPUT);  // Set the GPIO as a push/pull output
	gpio_set_level(STEPPER_MS1_PIN, 0);  // Low to enable receiver

	gpio_reset_pin(STEPPER_MS2_PIN);  // Set pad driver capability to GPIO
	gpio_set_direction(STEPPER_MS2_PIN, GPIO_MODE_OUTPUT);  // Set the GPIO as a push/pull output
	gpio_set_level(STEPPER_MS2_PIN, 0);  // Low to enable receiver

	gpio_reset_pin(STEPPER_STEP_PIN);  // Set pad driver capability to GPIO
	gpio_set_direction(STEPPER_STEP_PIN, GPIO_MODE_OUTPUT);  // Set the GPIO as a push/pull output
	gpio_set_level(STEPPER_STEP_PIN, 0);  // Low to enable receiver

	gpio_reset_pin(STEPPER_DIR_PIN);  // Set pad driver capability to GPIO
	gpio_set_direction(STEPPER_DIR_PIN, GPIO_MODE_OUTPUT);  // Set the GPIO as a push/pull output
	gpio_set_level(STEPPER_DIR_PIN, 0);  // Low to enable receiver

	while(1) {
		// ESP_LOGI(TAG, "Loop");
		/*
		gpio_set_level(STEPPER_ENABLE_PIN, 0);  // Low to enable receiver
		gpio_set_level(STEPPER_MS1_PIN, 0);  // Low to enable receiver
		gpio_set_level(STEPPER_MS2_PIN, 0);  // Low to enable receiver
		gpio_set_level(STEPPER_STEP_PIN, 0);  // Low to enable receiver
		gpio_set_level(STEPPER_DIR_PIN, 0);  // Low to enable receiver
		vTaskDelay(2000 / portTICK_PERIOD_MS);

		gpio_set_level(STEPPER_ENABLE_PIN, 1);  // Low to enable receiver
		gpio_set_level(STEPPER_MS1_PIN, 1);  // Low to enable receiver
		gpio_set_level(STEPPER_MS2_PIN, 1);  // Low to enable receiver
		gpio_set_level(STEPPER_STEP_PIN, 1);  // Low to enable receiver
		gpio_set_level(STEPPER_DIR_PIN, 1);  // Low to enable receiver
		*/
		vTaskDelay(2000 / portTICK_PERIOD_MS);
	}
}

