//
// Created by hanwool on 2018-10-18.
//

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "driver/gpio.h"
#include "esp_adc_cal.h"
#include "esp_log.h"
#include "sdkconfig.h"

#include "lora.h"

static void loraTxTask(void* pvParams) {
    uint8_t hello[5] = "HELLO";
    esp_err_t ret;
    while(true) {
        if ((ret = lora_send_packet(hello, 5)) != ESP_OK) {
            ESP_LOGE(__func__, "lora_send_packet: (0x%04X)", ret);
        } else {
            ESP_LOGI(__func__, "packet sent");
        }
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

void task_rx(void *p)
{
    for(;;) {
        lora_receive();    // put into receive mode
        while(lora_received()) {
            ESP_LOGI(__func__, "LoRa Received!");
            int receiveSize = 1024;
            uint8_t* buf = (uint8_t*)calloc(sizeof(uint8_t), (size_t)receiveSize);
            if(lora_receive_packet(buf, &receiveSize) != ESP_OK) {
                ESP_LOGE(__func__, "LoRa receive failed: ");
            } else {
                ESP_LOGI(__func__, "Received(%dbytes): %s", receiveSize, buf);
            }
            free(buf);
            lora_receive();
        }
//        ESP_LOGI(__func__, "LoRa Receive waiting...");
        vTaskDelay(10/portTICK_PERIOD_MS);
    }
}

void initSpi() {
    spi_bus_config_t bus = {
            .miso_io_num = GPIO_NUM_19,
            .mosi_io_num = GPIO_NUM_27,
            .sclk_io_num = GPIO_NUM_5,
//            .miso_io_num = GPIO_NUM_19,
//            .mosi_io_num = GPIO_NUM_23,
//            .sclk_io_num = GPIO_NUM_18,
            .quadwp_io_num = -1,
            .quadhd_io_num = -1,
            .max_transfer_sz = 0,
            .flags = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_SCLK | SPICOMMON_BUSFLAG_MISO | SPICOMMON_BUSFLAG_MOSI | SPICOMMON_BUSFLAG_DUAL
    };

    ESP_ERROR_CHECK(spi_bus_initialize(VSPI_HOST , &bus, 0));
}

void initLora() {
    lora_esp32_param_t lora_esp32_param = LORA_ESP32_PARAM_USING_SYSTEM_SPI;
    lora_esp32_param.spi_host_device = VSPI_HOST;
    lora_esp32_param.cs = GPIO_NUM_18;
    lora_esp32_param.rst = GPIO_NUM_14;
//    lora_esp32_param.cs = GPIO_NUM_5;
//    lora_esp32_param.rst = GPIO_NUM_4;
    ESP_ERROR_CHECK(lora_init(lora_esp32_param));
}

void app_main() {
    initSpi();
    initLora();
    ESP_ERROR_CHECK(lora_set_frequency(915e6));
    ESP_ERROR_CHECK(lora_enable_crc());
    ESP_ERROR_CHECK(lora_set_bandwidth(E_LORA_BANDWIDTH_250_KHZ));
    ESP_ERROR_CHECK(lora_set_sync_word(0x33));
//    ESP_ERROR_CHECK(lora_set_spreading_factor(E_LORA_SPREADING_FACTOR_MAX));
//    lora_dump_registers();
//    xTaskCreate(loraTxTask, "loraTxTask", 4096, NULL, 5, NULL);
    xTaskCreate(task_rx, "task_rx", 4096, NULL, 6, NULL);
}