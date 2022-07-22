#include "dmx.h"

#include <stdint.h>
#include <strings.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "driver/gpio.h"
#include "driver/uart.h"

/*
For sending and receiving DMX512 a Analog Devices
ADM268 is used which isolates the RS-485 bus from the
low voltage micro controller side.

ESP-PICO-KIT ESP32 PICO D4

Pin	Description
VCC		3.3VDC
IO22	Receive
IO19	Receive Enable	Low=Enable, High=Disable
IO23	Send Enable		High=Enable, Low=Disable
IO21	Send
https://raw.githubusercontent.com/TimGremalm/LightBoxNano/master/output/LightBoxNano.pdf
https://www.analog.com/media/en/technical-documentation/data-sheets/adm2682e_2687e.pdf
*/
#define PIN_RECEIVE 22
#define PIN_RECEIVE_ENABLE 19
#define PIN_SEND 21
#define PIN_SEND_ENABLE 23

#define DMX_UART_NUM UART_NUM_2
#define BUF_SIZE 1024
#define DMX_SIZE 513


static const char *TAG = "FyrDMX";

void dmxtask(void *pvParameters) {
	ESP_LOGI(TAG, "Start");
	// Setup receive-pins
	gpio_reset_pin(PIN_RECEIVE_ENABLE);  // Set pad driver capability to GPIO
	gpio_set_direction(PIN_RECEIVE_ENABLE, GPIO_MODE_OUTPUT);  // Set the GPIO as a push/pull output
	gpio_set_level(PIN_RECEIVE_ENABLE, 0);  // Low to enable receiver

	// Setup send-pins
	gpio_reset_pin(PIN_SEND_ENABLE);  // Set pad driver capability to GPIO
	gpio_set_direction(PIN_SEND_ENABLE, GPIO_MODE_OUTPUT);  // Set the GPIO as a push/pull output
	gpio_set_level(PIN_SEND_ENABLE, 0);  // Low to disable sender

	// DMX inspired by https://github.com/luksal/ESP32-DMX
	// Setup UART dor DMX512
	QueueHandle_t dmx_rx_queue;
	uart_config_t uart_config = {
		.baud_rate = 250000,
		.data_bits = UART_DATA_8_BITS,
		.parity = UART_PARITY_DISABLE,
		.stop_bits = UART_STOP_BITS_2,
		.flow_ctrl = UART_HW_FLOWCTRL_DISABLE
	};
	uart_param_config(DMX_UART_NUM, &uart_config);

	// Set pins for UART
	uart_set_pin(DMX_UART_NUM, PIN_SEND, PIN_RECEIVE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

	// Install queue
	uart_driver_install(DMX_UART_NUM, BUF_SIZE, BUF_SIZE, 20, &dmx_rx_queue, 0);

	uart_event_t event;
	uint16_t current_rx_addr = 0;
	uint8_t rx_buf[BUF_SIZE] = {0};
	uint8_t dmx_data_temp[DMX_SIZE] = {0};  // DMX Start Code + 512 Channel bytes
	ESP_LOGI(TAG, "Start receive DMX512 on UART %d IO %d", DMX_UART_NUM, PIN_RECEIVE);
	while(1) {
		if(xQueueReceive(dmx_rx_queue, (void * )&event, (TickType_t)portMAX_DELAY)) {
			switch(event.type) {
				case UART_DATA:
					uart_read_bytes(DMX_UART_NUM, rx_buf, event.size, 23 / portTICK_PERIOD_MS);
					for(int i = 0; i < event.size; i++) {
						if (current_rx_addr <= DMX_SIZE) {
							dmx_data_temp[current_rx_addr] = rx_buf[i];
						}
						current_rx_addr++;
					}
					break;
				case UART_BREAK:
					// Only consider valid if we have 513 received bytes
					if (current_rx_addr == DMX_SIZE) {
						// Skip first start-byte
						for(int i = 1; i < current_rx_addr; i++) {
							// ESP_LOGI(TAG, "%d %d", i, dmx_data_temp[i]);
							dmx_data[i] = dmx_data_temp[i];
						}
					}
					current_rx_addr = 0;
					uart_flush_input(DMX_UART_NUM);
					xQueueReset(dmx_rx_queue);
					break;
				case UART_FRAME_ERR:
				case UART_PARITY_ERR:
				case UART_BUFFER_FULL:
				case UART_FIFO_OVF:
				default:
					current_rx_addr = 0;
					uart_flush_input(DMX_UART_NUM);
					xQueueReset(dmx_rx_queue);
					break;
			}
		}
	}
}

