#include "motor.h"

#include <stdint.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/ledc.h"
#include "driver/gpio.h"

#include "dmx.h"
#include "hall.h"

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

// DMX Channel mapping
#define DMX_CHANNEL_STARTT 1
#define DMX_CHANNEL_MOTOR_LSB 5
#define DMX_CHANNEL_MOTOR_MSB 6
#define DMX_CHANNEL_MOTOR_SPEED_MIN 7
#define DMX_CHANNEL_MOTOR_SPEED_MAX 8

static const char *TAG = "FyrMotor";

// State variables
uint16_t positionGoal = 0;
uint16_t positionActual = 0;
uint16_t stepperSpeedNow = 0;  // Current motor speed in ms per pulse half
int8_t stepperSpeedAcceleration = 0;  // Acceleration -1 decrease, 0 stable, 1 increasing
int8_t stepperSpeedAccelerationPrevious = 0;
int8_t stepperDirection = 0;  // Direction: -1 backwards, 0 stop, 1 forward
int8_t stepperDirectionPrevious = 0;
uint32_t lastSleepKick = 0;

// Motor adjustment variables
uint16_t stepperStepsPerRevolution = 0;
uint16_t stepperPosPerTurn = 0;
uint16_t stepperSpeedMin = 0;
uint16_t stepperSpeedMax = 0;
uint16_t stepperStepMinPulseMs = 0;
uint16_t stepperStepMaxPulseMs = 0;
uint16_t stepperBrakeDistance = 0;
uint16_t stepperRamp = 0;

uint8_t motor_lsb = 0;
uint8_t motor_msb = 0;
uint8_t motor_speed_min = 0;
uint8_t motor_speed_min_previous = 0;
uint8_t motor_speed_max = 0;
uint8_t motor_speed_max_previous = 0;

// Hall limits
uint16_t hall_limit_on = 2190;
uint16_t hall_limit_off = 2000;
uint16_t twelve_gap = 3000;
uint16_t hall_twelve_position = 0;
uint16_t hall_twelve_direction = 0;
uint16_t hall_twelve_state = 0;

enum microstepping {
	step_full,
	step_half,
	step_quarter,
	step_eigth
};

void setMicroSteppingMode(enum microstepping mode) {
	//Stepper driver A3967
	switch (mode) {
		case step_full:
			//Full Step (2 Phase)
			gpio_set_level(STEPPER_MS1_PIN, 0);
			gpio_set_level(STEPPER_MS2_PIN, 0);
			break;
		case step_half:
			//1/2th
			gpio_set_level(STEPPER_MS1_PIN, 1);
			gpio_set_level(STEPPER_MS2_PIN, 0);
			break;
		case step_quarter:
			//1/4th
			gpio_set_level(STEPPER_MS1_PIN, 0);
			gpio_set_level(STEPPER_MS2_PIN, 1);
			break;
		case step_eigth:
			//1/8th
			gpio_set_level(STEPPER_MS1_PIN, 1);
			gpio_set_level(STEPPER_MS2_PIN, 1);
			break;
	}
}

void setDirectionLeft() {
	gpio_set_level(STEPPER_DIR_PIN, 0);
}

void setDirectionRight() {
	gpio_set_level(STEPPER_DIR_PIN, 1);
}

void disableDriver() {
	gpio_set_level(STEPPER_ENABLE_PIN, 1);
}

void enableDriver() {
	gpio_set_level(STEPPER_ENABLE_PIN, 0);
}

void setSpeedMin(uint16_t value) {
	stepperSpeedMin = value;
	// stepperStepMinPulseMs = (1000/stepperSpeedMin) / 2;  // 1s / min speed / 2
	// stepperStepMinPulseMs = (((uint32_t)1000000)/stepperSpeedMin) / 2;  // 1s / min speed / 2
	// ESP_LOGI(TAG, "Step Min Puls us: %d", stepperStepMinPulseMs);
}

void check_hall() {
	/*
	Hall sensor seems to be around 1800 when idle.
	At 12'o'clock it's around 2190.
	limit_on and off creates a hysteresis effect.
	*/
	if (hall_twelve_state == 0 && hall_data > hall_limit_on) {
		hall_twelve_state = 1;
		hall_twelve_position = positionActual;
		hall_twelve_direction = stepperDirection;
		ESP_LOGI(TAG, "Hall On: %d, Position: %d, Direction: %d",
		         hall_data, positionActual, stepperDirection);
		if (stepperDirection == 1) {
			positionActual = 0 - (twelve_gap/2);
		} else if (stepperDirection == -1) {
			positionActual = 0 + (twelve_gap/2);
		}
	}
	if (hall_twelve_state == 1 && hall_data < hall_limit_off) {
		hall_twelve_state = 0;
		hall_twelve_position = positionActual;
		hall_twelve_direction = stepperDirection;
		ESP_LOGI(TAG, "Hall Off: %d, Position: %d, Direction: %d",
		         hall_data, positionActual, stepperDirection);
	}
}

void step() {
	int16_t diff = positionGoal - positionActual;
	int16_t diffAbs = abs(diff);
	// Check direction
	if (diff > 0) {
		stepperDirection =  1;
	} else if (diff < 0) {
		stepperDirection =  -1;
	} else {
		stepperDirection =  0;
	}
	// Check direction change
	if (stepperDirection != stepperDirectionPrevious) {
		stepperDirectionPrevious = stepperDirection;
		if (stepperDirection == 1) {
			setDirectionRight();
			ESP_LOGI(TAG, "Position %d Going: Right", positionActual);
		} else if (stepperDirection == -1) {
			setDirectionLeft();
			ESP_LOGI(TAG, "Position %d Going: Left", positionActual);
		} else {
			ESP_LOGI(TAG, "Position %d Going: Stop", positionActual);
		}
		/*
		// When switching direction acceleration shall always restart
		stepperSpeedAcceleration = 1;
		*/
		stepperSpeedNow = stepperStepMinPulseMs;
	}
	stepperSpeedNow = stepperStepMinPulseMs;
	// motor_speed_min = 0;
	// motor_speed_max = 0;

	// Test new acceleration and speed
	// speed = diff
	//   SpeedMin - SpeedMax / DiffMin - DiffMax
	// smoother

	// Set stepper speed extremes based on DMX-values.
	float diff_scale = (float)diffAbs / (float)32767;
	// Invert DMX-value, multiply by a range, and add offset                Range   Offset
	uint16_t motor_speed_min_ms = ((((float)(255-motor_speed_min)) / 255) * 5000) + 1000;
	// Sacale to float, multiply with range                                                          Range
	// uint16_t motor_speed_max_ms = motor_speed_min_ms - ((uint16_t)((((float)motor_speed_max) / 255) * 5500));
	uint16_t motor_speed_max_ms = (uint16_t)((((float)motor_speed_max) / 255) * 5500);
	if (motor_speed_max_ms + 50 > motor_speed_min_ms) {
		motor_speed_max_ms = 50;
	} else {
		motor_speed_max_ms = motor_speed_min_ms - motor_speed_max_ms;
	}
	// ESP_LOGI(TAG, "diff_scale: %f", diff_scale);
	// ESP_LOGI(TAG, "motor_speed_min_ms: %d", motor_speed_min_ms);
	// ESP_LOGI(TAG, "motor_speed_max_ms: %d", motor_speed_max_ms);

	// Set goal target speed, but first get range
	uint16_t stepper_speed_range_ms = motor_speed_min_ms - motor_speed_max_ms;
	// Inverse scale, multiply to range
	uint16_t stepper_speed_us = (stepper_speed_range_ms * (1.0 - diff_scale)) + motor_speed_max_ms;
	// ESP_LOGI(TAG, "stepper_speed_range_ms: %d", stepper_speed_range_ms);
	// ESP_LOGI(TAG, "stepper_speed_us: %d", stepper_speed_us);
	stepperSpeedNow = stepper_speed_us;

	// If at goal position, stop and return
	if (diff == 0) {
		stepperSpeedNow = 0;
		return;
	}

	// Make step
	if ((positionActual % stepperPosPerTurn) == 0) {
		// Step High
		gpio_set_level(STEPPER_STEP_PIN, 1);
		esp_rom_delay_us(stepperSpeedNow);
		// Step Low
		gpio_set_level(STEPPER_STEP_PIN, 0);
		esp_rom_delay_us(stepperSpeedNow);
	}
	// Increment position
	positionActual += stepperDirection;
	// ESP_LOGI(TAG, "positionActual: %d", positionActual);
}

void dmx_read() {
	// Go throgh DMX channels, the channel number depends on DMX start and par can number
	motor_lsb = dmx_data[DMX_CHANNEL_STARTT + DMX_CHANNEL_MOTOR_LSB];
	motor_msb = dmx_data[DMX_CHANNEL_STARTT + DMX_CHANNEL_MOTOR_MSB];
	positionGoal = (motor_lsb<<8) + motor_msb;

	motor_speed_min = dmx_data[DMX_CHANNEL_STARTT + DMX_CHANNEL_MOTOR_SPEED_MIN];
	if (motor_speed_min != motor_speed_min_previous) {
		// Set new speed, multiply 8-bit DMX-channel by 3
		setSpeedMin(((uint16_t)motor_speed_min)*5);
		motor_speed_min_previous = motor_speed_min;
	}

	motor_speed_max = dmx_data[DMX_CHANNEL_STARTT + DMX_CHANNEL_MOTOR_SPEED_MAX];
	if (motor_speed_max != motor_speed_max_previous) {
		// Set new speed, multiply 8-bit DMX-channel by 3
		motor_speed_max_previous = motor_speed_min;
	}
}

void check_last_sleep() {
	uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
	if ((now - lastSleepKick) > 4000) {
		// ESP_LOGI(TAG, "Time to pet thread watchdog timer.");
		vTaskDelay(11 / portTICK_PERIOD_MS);
		lastSleepKick = now;
	}
}

void motortask(void *pvParameters) {
	ESP_LOGI(TAG, "Start");

	// Motor adjustment variables
	stepperStepsPerRevolution = 1100;  // Steps per revolution
	stepperPosPerTurn = 65535 / stepperStepsPerRevolution;  // Positions per Turn
	stepperSpeedMin = 60;  // Minimum steps per second
	stepperSpeedMax = 200;  // Maximum steps per second
	stepperStepMinPulseMs = (((uint32_t)1000000)/stepperSpeedMin) / 2;  // 1s / min speed / 2
	stepperStepMaxPulseMs = (1000/stepperSpeedMax) / 2;  // 1s / max speed / 2
	stepperBrakeDistance = 500;// stepperPosPerTurn * stepperSpeedMin;  // Position from goal to start brake
	stepperRamp = 1;  // Ramp up/down increment

	// Setup stepper pins
	gpio_reset_pin(STEPPER_ENABLE_PIN);  // Set pad driver capability to GPIO
	gpio_set_direction(STEPPER_ENABLE_PIN, GPIO_MODE_OUTPUT);  // Set the GPIO as a push/pull output
	gpio_set_level(STEPPER_ENABLE_PIN, 0);

	gpio_reset_pin(STEPPER_MS1_PIN);  // Set pad driver capability to GPIO
	gpio_set_direction(STEPPER_MS1_PIN, GPIO_MODE_OUTPUT);  // Set the GPIO as a push/pull output
	gpio_set_level(STEPPER_MS1_PIN, 0);

	gpio_reset_pin(STEPPER_MS2_PIN);  // Set pad driver capability to GPIO
	gpio_set_direction(STEPPER_MS2_PIN, GPIO_MODE_OUTPUT);  // Set the GPIO as a push/pull output
	gpio_set_level(STEPPER_MS2_PIN, 0);

	gpio_reset_pin(STEPPER_STEP_PIN);  // Set pad driver capability to GPIO
	gpio_set_direction(STEPPER_STEP_PIN, GPIO_MODE_OUTPUT);  // Set the GPIO as a push/pull output
	gpio_set_level(STEPPER_STEP_PIN, 0);

	gpio_reset_pin(STEPPER_DIR_PIN);  // Set pad driver capability to GPIO
	gpio_set_direction(STEPPER_DIR_PIN, GPIO_MODE_OUTPUT);  // Set the GPIO as a push/pull output
	gpio_set_level(STEPPER_DIR_PIN, 0);

	// Stepper driver A3967
	setMicroSteppingMode(step_full);

	while(1) {
		dmx_read();
		check_hall();
		step();
		check_last_sleep();
	}
}

