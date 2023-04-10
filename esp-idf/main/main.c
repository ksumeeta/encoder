#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_spi_flash.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "math.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define mainTAG "MAIN"

#define ESP_INTR_FLAG_DEFAULT 0

/*
# Encode setup
Get the pinout details of esp32 module.

Attach :
GPIO23 of ESP32 to CLK of Encoder
GPIO22 of ESP32 to DT of Encoder
*/

#define SW 15
#define DT 22
#define CLK 23

uint8_t counter = 0;
uint8_t currentStateCLK;
uint8_t lastStateCLK;
bool ClockWiseDir;
bool clkB;
bool dtB;

// Queue for interrupt
static QueueHandle_t encoderQueue = NULL;

// Structure for data to be passed to the Queue
typedef struct {
  uint32_t gpioNum; /*!< GPIO pin: set with bit mask, each bit maps to a GPIO */
  bool gpioValue;
} encoderData_t;

// GPIO Interrupt Service Routine. Will be fired when GPIO 22 or 23 Changes its
// level and will send the data to Queue.
static void IRAM_ATTR encoderISR(void *arg) {
  uint32_t gpioNum = (uint32_t)arg;
  encoderData_t encoderData;
  encoderData.gpioNum = gpioNum;
  if (gpio_get_level(gpioNum)) {
    encoderData.gpioValue = true;
  } else {
    encoderData.gpioValue = false;
  }
  xQueueSendFromISR(encoderQueue, &encoderData, NULL);
}

// Main Encode Task. This Task will wait for the Queue data, when the interrupt
// is fired.
static void encoderTask(void *args) {
  uint8_t i = 0;
  while (1) { // infinite Loop
    encoderData_t encoderData;
    uint32_t gpioNum;

    // Queue waiting for MAX_DELAY
    if (xQueueReceive(encoderQueue, &encoderData, portMAX_DELAY)) {
      gpioNum = encoderData.gpioNum;
      // Rotation Counting Logic
      if ((gpioNum == CLK)) {
        if ((encoderData.gpioValue) && (gpio_get_level(gpioNum)) &&
            (clkB == false)) {
          clkB = true;
          if (dtB) {
            i--;
            printf("%d CLK H C\n", i);
          } else {
            i++;
            printf("%d CLK H CC\n", i);
          }
        } else if ((encoderData.gpioValue == false) &&
                   (gpio_get_level(gpioNum) == false) && (clkB)) {
          clkB = false;
          if (dtB) {
            i++;
            printf("%d CLK L CC\n", i);
          } else {
            i--;
            printf("%d CLK L C\n", i);
          }
        }
      } else if ((gpioNum == DT)) {
        if ((encoderData.gpioValue) && (gpio_get_level(gpioNum)) &&
            (dtB == false)) {
          dtB = true;
          if (clkB) {
            i++;
            printf("%d DT H CC\n", i);
          } else {
            i--;
            printf("%d DT H C\n", i);
          }
        } else if ((encoderData.gpioValue == false) &&
                   (gpio_get_level(gpioNum) == false) && (dtB)) {
          dtB = false;
          if (clkB) {
            i--;
            printf("%d DT L C\n", i);
          } else {
            i++;
            printf("%d DT L CC\n", i);
          }
        }
      }
    }
  }
}

// Setting GPIO pins for inputmode and configuring the interrupts
static void gpioInit() {
  gpio_reset_pin(CLK);
  gpio_reset_pin(DT);
  gpio_reset_pin(SW);

  // gpio_set_direction(CLK, GPIO_MODE_INPUT);
  // gpio_set_pull_mode(CLK, GPIO_PULLUP_ONLY);

  // gpio_set_direction(DT, GPIO_MODE_INPUT);
  // gpio_set_pull_mode(DT, GPIO_PULLUP_ONLY);

  gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
  gpio_isr_handler_add(DT, encoderISR, (void *)DT);
  gpio_isr_handler_add(CLK, encoderISR, (void *)CLK);

  gpio_config_t io_conf;
  io_conf.intr_type = GPIO_INTR_ANYEDGE;
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pin_bit_mask = ((1ULL << DT) | (1ULL << CLK));
  io_conf.pull_up_en = 1;
  io_conf.pull_down_en = 0;
  gpio_config(&io_conf);

  if (gpio_get_level(CLK)) {
    clkB = true;
  } else {
    clkB = false;
  }

  if (gpio_get_level(DT)) {
    dtB = true;
  } else {
    dtB = false;
  }
  lastStateCLK = gpio_get_level(CLK);
}

void app_main(void) {

  // Checking Non Volatile Flash---standard startup routine to check the health
  // of Module Flash.
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);
  nvs_stats_t nvs_stats;
  ESP_ERROR_CHECK(nvs_get_stats(NULL, &nvs_stats));
  ESP_LOGI(mainTAG,
           "Count: UsedEntries = (%d), FreeEntries = (%d), AllEntries = (%d)",
           nvs_stats.used_entries, nvs_stats.free_entries,
           nvs_stats.total_entries);
  // Checking Non Volatile Flash---standard startup routine to check the health
  // of Module Flash.

  // Create Queue
  encoderQueue = xQueueCreate(100, sizeof(encoderData_t));

  // Create encode task
  xTaskCreate(encoderTask, "ENCTSK", 2048, NULL, 10, NULL);

  gpioInit();
}
