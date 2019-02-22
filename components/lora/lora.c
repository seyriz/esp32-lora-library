
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "lora.h"



static lora_esp32_param_t esp32_param;
static const char* TAG = "lora-esp32";

typedef enum {
    E_LORA_REG_FIFO = 0x00,
    E_LORA_REG_OP_MODE = 0x01,
    E_LORA_REG_FRF_MSB = 0x06,
    E_LORA_REG_FRF_MID = 0x07,
    E_LORA_REG_FRF_LSB = 0x08,
    E_LORA_REG_PA_CONFIG = 0x09,
    E_LORA_REG_LNA = 0x0c,
    E_LORA_REG_FIFO_ADDR_PTR = 0x0d,
    E_LORA_REG_FIFO_TX_BASE_ADDR = 0x0e,
    E_LORA_REG_FIFO_RX_BASE_ADDR = 0x0f,
    E_LORA_REG_FIFO_RX_CURRENT_ADDR = 0x10,
    E_LORA_REG_IRQ_FLAGS = 0x12,
    E_LORA_REG_RX_NB_BYTES = 0x13,
    E_LORA_REG_PKT_SNR_VALUE = 0x19,
    E_LORA_REG_PKT_RSSI_VALUE = 0x1a,
    E_LORA_REG_MODEM_CONFIG_1 = 0x1d,
    E_LORA_REG_MODEM_CONFIG_2 = 0x1e,
    E_LORA_REG_PREAMBLE_MSB = 0x20,
    E_LORA_REG_PREAMBLE_LSB = 0x21,
    E_LORA_REG_PAYLOAD_LENGTH = 0x22,
    E_LORA_REG_MODEM_CONFIG_3 = 0x26,
    E_LORA_REG_RSSI_WIDEBAND = 0x2c,
    E_LORA_REG_DETECTION_OPTIMIZE = 0x31,
    E_LORA_REG_DETECTION_THRESHOLD = 0x37,
    E_LORA_REG_SYNC_WORD = 0x39,
    E_LORA_REG_DIO_MAPPING_1 = 0x40,
    E_LORA_REG_VERSION = 0x42,
} lora_esp32_register;

typedef enum {
    E_LORA_MODE_LONG_RANGE_MODE = 0x80,
    E_LORA_MODE_SLEEP = 0x00,
    E_LORA_MODE_STDBY = 0x01,
    E_LORA_MODE_TX = 0x03,
    E_LORA_MODE_RX_CONTINUOUS = 0x05,
    E_LORA_MODE_RX_SINGLE = 0x06,
} lora_esp32_transceiver_mode;

typedef enum {
    E_LORA_IRQ_TX_DONE_MASK = 0x08,
    E_LORA_IRQ_PAYLOAD_CRC_ERROR_MASK = 0x20,
    E_LORA_IRQ_RX_DONE_MASK = 0x40,
} lora_esp32_irq_mask;

/*
 * PA configuration
 */
#define PA_BOOST                       0x80


#define PA_OUTPUT_RFO_PIN              0
#define PA_OUTPUT_PA_BOOST_PIN         1

#define TIMEOUT_RESET                  100

static bool __implicit;
static long __frequency;

/**
 * Write a value to a register.
 * @param reg Register index.
 * @param val Value to write.
 */
esp_err_t lora_write_reg(lora_esp32_register reg, uint8_t val) {
    uint8_t out[2] = { (0x80 | reg), val };
    uint8_t in[2];

    spi_transaction_t t = {
            .flags = 0,
            .length = 16,
            .tx_buffer = out,
            .rx_buffer = in
    };
    esp_err_t err;
    gpio_set_level(esp32_param.cs, 0);
    err = spi_device_transmit(esp32_param.spi_device_handle, &t);
    gpio_set_level(esp32_param.cs, 1);
    return err;
}

/**
 * Read the current value of a register.
 * @param reg Register index.
 * @return Value of the register.
 */
esp_err_t lora_read_reg(lora_esp32_register reg, uint8_t* read) {
    uint8_t out[2] = { reg, 0xff };
    uint8_t in[2];

    spi_transaction_t t = {
            .flags = 0,
            .length = 16,
            .tx_buffer = out,
            .rx_buffer = in
    };

    esp_err_t err;

    gpio_set_level(esp32_param.cs, 0);
    if((err = spi_device_transmit(esp32_param.spi_device_handle, &t)) != ESP_OK) {
        *read = 0;
        return err;
    }

    gpio_set_level(esp32_param.cs, 1);
//    ESP_LOGD(TAG, "lora_read_reg READED value: {%02X, %02X}", in[0], in[1]);
    *read = in[1];
    return ESP_OK;
}

/**
 * Perform physical reset on the Lora chip
 */
void lora_reset() {
    if(esp32_param.rst != ESP32_HAL_UNDEFINED) {
        gpio_set_level(esp32_param.rst, 0);
        vTaskDelay(20/portTICK_PERIOD_MS);
        gpio_set_level(esp32_param.rst, 1);
        vTaskDelay(10/portTICK_PERIOD_MS);
    }
}

/**
 * Configure explicit header mode.
 * Packet size will be included in the frame.
 */
esp_err_t lora_explicit_header_mode() {
    __implicit = false;
    uint8_t reg_value;
    esp_err_t err;
    if((err = lora_read_reg(E_LORA_REG_MODEM_CONFIG_1, &reg_value)) != ESP_OK) {
        return err;
    }
    if((err = lora_write_reg(E_LORA_REG_MODEM_CONFIG_1, (uint8_t)reg_value & (uint8_t) (0xfe))) != ESP_OK) {
        return err;
    }
    return ESP_OK;
}

/**
 * Configure implicit header mode.
 * All packets will have a predefined size.
 * @param size Size of the packets.
 */
esp_err_t lora_implicit_header_mode(uint8_t size) {
    __implicit = true;

    uint8_t reg_value;
    esp_err_t err;
    if((err = lora_read_reg(E_LORA_REG_MODEM_CONFIG_1, &reg_value)) != ESP_OK) {
        return err;
    }
    if((err = lora_write_reg(E_LORA_REG_MODEM_CONFIG_1, (uint8_t)reg_value | (uint8_t)(0x01))) != ESP_OK) {
        return err;
    }
    if((err = lora_write_reg(E_LORA_REG_PAYLOAD_LENGTH, size)) != ESP_OK) {
        return err;
    }
    return ESP_OK;
}

/**
 * Sets the radio transceiver in idle mode.
 * Must be used to change registers and access the FIFO.
 */
esp_err_t lora_idle() {
    esp_err_t err;

    if((err = lora_write_reg(E_LORA_REG_OP_MODE, E_LORA_MODE_LONG_RANGE_MODE | E_LORA_MODE_STDBY)) != ESP_OK) {
        return err;
    }
    return ESP_OK;
}

/**
 * Sets the radio transceiver in sleep mode.
 * Low power consumption and FIFO is lost.
 */
esp_err_t lora_sleep() {
    esp_err_t err;

    if((err = lora_write_reg(E_LORA_REG_OP_MODE, E_LORA_MODE_LONG_RANGE_MODE | E_LORA_MODE_SLEEP)) != ESP_OK) {
        return err;
    }
    return ESP_OK;
}

/**
 * Sets the radio transceiver in receive mode.
 * Incoming packets will be received.
 */
esp_err_t lora_receive() {
    esp_err_t err;

    if((err = lora_write_reg(E_LORA_REG_OP_MODE, E_LORA_MODE_LONG_RANGE_MODE | E_LORA_MODE_RX_CONTINUOUS)) != ESP_OK) {
        return err;
    }
    return ESP_OK;
}

/**
 * Configure power level for transmission
 * @param level 2-17, from least to most power
 */
esp_err_t lora_set_tx_power(lora_esp32_power_level level) {
    // RF9x module uses PA_BOOST pin
    esp_err_t err;

    if((err = lora_write_reg(E_LORA_REG_PA_CONFIG, (uint8_t)(PA_BOOST | level))) != ESP_OK) {
        return err;
    }
    return ESP_OK;
}

/**
 * Set carrier frequency.
 * @param frequency Frequency in Hz
 */
esp_err_t lora_set_frequency(long frequency) {
    __frequency = frequency;

    ESP_LOGI(TAG, "LoRa frequency set to %dMHz", (int)(frequency/1000000));
    uint64_t frf = ((uint64_t)frequency << 19) / 32000000;


    esp_err_t err;

    if((err = lora_write_reg(E_LORA_REG_FRF_MSB, (uint8_t)(frf >> 16))) != ESP_OK) {
        return err;
    }
    if((err = lora_write_reg(E_LORA_REG_FRF_MID, (uint8_t)(frf >> 8))) != ESP_OK) {
        return err;
    }
    if((err = lora_write_reg(E_LORA_REG_FRF_LSB, (uint8_t)(frf >> 0))) != ESP_OK) {
        return err;
    }
    return ESP_OK;
}

/**
 * Set spreading factor.
 * @param sf 6-12, Spreading factor to use.
 */
esp_err_t lora_set_spreading_factor(lora_esp32_spreading_factor sf) {
    esp_err_t err;
    uint8_t reg_value;

    if (sf == E_LORA_SPREADING_FACTOR_MIN) {
        if((err = lora_write_reg(E_LORA_REG_DETECTION_OPTIMIZE, 0xc5)) != ESP_OK) {
            return err;
        }
        if((err = lora_write_reg(E_LORA_REG_DETECTION_THRESHOLD, 0x0c)) != ESP_OK) {
            return err;
        }
    } else {

        if((err = lora_write_reg(E_LORA_REG_DETECTION_OPTIMIZE, 0xc3)) != ESP_OK) {
            return err;
        }
        if((err = lora_write_reg(E_LORA_REG_DETECTION_THRESHOLD, 0x0a)) != ESP_OK) {
            return err;
        }
    }

    if((err = lora_read_reg(E_LORA_REG_MODEM_CONFIG_2, &reg_value)) != ESP_OK) {
        return err;
    }
    if((err = lora_write_reg(E_LORA_REG_MODEM_CONFIG_2, (uint8_t)reg_value & (uint8_t)0x0f) | (uint8_t)((sf << 4) & 0xf0)) != ESP_OK) {
        return err;
    }

    return ESP_OK;
}

/**
 * Set bandwidth (bit rate)   * @param sbw Bandwidth in Hz (up to 500000)   */
esp_err_t lora_set_bandwidth(lora_esp32_bandwidth sbw) {
    esp_err_t ret;
    uint8_t reg_value;
    if((ret = lora_read_reg(E_LORA_REG_MODEM_CONFIG_1, &reg_value)) != ESP_OK) {
        return ret;
    }
    if((ret = lora_write_reg(E_LORA_REG_MODEM_CONFIG_1, (uint8_t)((reg_value & 0x0f) | (sbw << 4)))) != ESP_OK) {
        return ret;
    }
    return ESP_OK;
}

/**
 * Set coding rate 
 * @param denominator 5-8, Denominator for the coding rate 4/x
 */
esp_err_t lora_set_coding_rate(lora_esp32_coding_rate denominator) {
    int cr = denominator - 4;
    esp_err_t ret;
    uint8_t reg_value;
    if((ret = lora_read_reg(E_LORA_REG_MODEM_CONFIG_1, &reg_value)) != ESP_OK) {
        return ret;
    }
    if((ret = lora_write_reg(E_LORA_REG_MODEM_CONFIG_1, (uint8_t)((reg_value & 0xf1) | (cr << 1)))) != ESP_OK) {
        return ret;
    }
    return ESP_OK;
}

/**
 * Set the size of preamble.
 * @param length Preamble length in symbols.
 */
esp_err_t lora_set_preamble_length(uint16_t length) {
    esp_err_t ret;
    if((ret = lora_write_reg(E_LORA_REG_PREAMBLE_MSB, (uint8_t)(length >> 8))) != ESP_OK) {
        return ret;
    }
    if((ret = lora_write_reg(E_LORA_REG_PREAMBLE_LSB, (uint8_t)(length >> 0))) != ESP_OK) {
        return ret;
    }
    return ESP_OK;
}

/**
 * Change radio sync word.
 * @param sw New sync word to use.
 */
esp_err_t lora_set_sync_word(uint8_t sw) {
    esp_err_t ret;
    if((ret = lora_write_reg(E_LORA_REG_SYNC_WORD, sw)) != ESP_OK) {
        return ret;
    }
    return ESP_OK;
}

/**
 * Enable appending/verifying packet CRC.
 */
esp_err_t lora_enable_crc() {
    esp_err_t ret;
    uint8_t reg_value;
    if((ret = lora_read_reg(E_LORA_REG_MODEM_CONFIG_2, &reg_value)) != ESP_OK) {
        return ret;
    }
    if((ret = lora_write_reg(E_LORA_REG_MODEM_CONFIG_2, (uint8_t)(reg_value | 0x04))) != ESP_OK) {
        return ret;
    }
    return ESP_OK;
}

/**
 * Disable appending/verifying packet CRC.
 */
esp_err_t lora_disable_crc() {
    esp_err_t ret;
    uint8_t reg_value;
    if((ret = lora_read_reg(E_LORA_REG_MODEM_CONFIG_2, &reg_value)) != ESP_OK) {
        return ret;
    }
    if((ret = lora_write_reg(E_LORA_REG_MODEM_CONFIG_2, (uint8_t)(reg_value & 0xfb))) != ESP_OK) {
        return ret;
    }
    return ESP_OK;
}

/**
 * Perform hardware initialization.
 */
esp_err_t lora_init(lora_esp32_param_t params) {
    esp32_param = params;
    esp_err_t ret;

    if(esp32_param.rst == ESP32_HAL_UNDEFINED) {
        ESP_LOGW(TAG, "RESET pin is not configured. reset function is disabled");
    } else {
        gpio_pad_select_gpio(esp32_param.rst);
        gpio_set_direction(esp32_param.rst, GPIO_MODE_OUTPUT);
    }

    if(!esp32_param.using_preconfigured_spi_bus) {
        if (esp32_param.sck == ESP32_HAL_UNDEFINED &&
            esp32_param.mosi == ESP32_HAL_UNDEFINED &&
            esp32_param.miso == ESP32_HAL_UNDEFINED) {
            ESP_LOGE(TAG, "SPI PIN is NOT configured");
            return ESP_ERR_INVALID_ARG;
        }
        spi_bus_config_t bus = {
                .miso_io_num = esp32_param.miso,
                .mosi_io_num = esp32_param.mosi,
                .sclk_io_num = esp32_param.sck,
                .quadwp_io_num = -1,
                .quadhd_io_num = -1,
                .max_transfer_sz = 0,
                .flags = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_SCLK | SPICOMMON_BUSFLAG_MISO | SPICOMMON_BUSFLAG_MOSI | SPICOMMON_BUSFLAG_DUAL
        };

        if ((ret = spi_bus_initialize(esp32_param.spi_host_device, &bus, 0)) != ESP_OK) {
            ESP_LOGE(TAG, "spi_bus_initialize failed: %s(0x%04X)", esp_err_to_name(ret), ret);
            return ret;
        } else {
            ESP_LOGD(TAG, "spi_bus_initialize success");
        }
    }
    if(!esp32_param.using_preconfigured_spi_device) {
        if(esp32_param.cs == ESP32_HAL_UNDEFINED) {
            ESP_LOGE(TAG, "CS PIN is NOT configured");
            return ESP_ERR_INVALID_ARG;
        }

        gpio_pad_select_gpio(esp32_param.cs);
        gpio_set_direction(esp32_param.cs, GPIO_MODE_OUTPUT);
        gpio_set_level(esp32_param.cs, 1);

        spi_device_interface_config_t dev = {
                .clock_speed_hz = esp32_param.spi_clock_speed_hz,
                .mode = 0,
                .spics_io_num = -1,
                .queue_size = 1,
                .flags = 0,
                .pre_cb = NULL
        };
        if ((ret = spi_bus_add_device(esp32_param.spi_host_device, &dev, &esp32_param.spi_device_handle)) != ESP_OK) {
            ESP_LOGE(TAG, "spi_bus_add_device failed: %s(0x%04X)", esp_err_to_name(ret), ret);
            return ret;
        } else {
            ESP_LOGD(TAG, "spi_bus_add_device success");
        }
    } else {
        if(esp32_param.spi_device_handle == NULL) {
            ESP_LOGE(TAG, "using_preconfigured_spi_device is true but spi_device_handle is NULL");
            return ESP_ERR_INVALID_ARG;
        }
    }

    /*
     * Perform hardware reset.
     */
    lora_reset();

    /*
     * Check version.
     */
    uint8_t version = 0;
    int i = 0;
    while(i++ < TIMEOUT_RESET) {
        if((ret = lora_read_reg(E_LORA_REG_VERSION, &version)) != ESP_OK) {
            return ret;
        }
        ESP_LOGD(TAG, "VERSION READ: 0x%02X", version);
        if(version == 0x12) break;
        vTaskDelay(30/portTICK_PERIOD_MS);
    }
    ESP_LOGD(TAG, "VERSION READ: 0x%02X", version);

    if(i >= TIMEOUT_RESET) {
        ESP_LOGE(TAG, "RESET TIME OUT.");
        return LORA_ESP32_ERR_RESET_TIMEOUT;
    }

    /*
     * Default configuration.
     */
    uint8_t reg_value;
    lora_sleep();
    lora_write_reg(E_LORA_REG_FIFO_RX_BASE_ADDR, 0);
    lora_write_reg(E_LORA_REG_FIFO_TX_BASE_ADDR, 0);

    if((ret = lora_read_reg(E_LORA_REG_LNA, &reg_value)) != ESP_OK) {
        return ret;
    }
    lora_write_reg(E_LORA_REG_LNA, (uint8_t)reg_value | (uint8_t)0x03);
    lora_write_reg(E_LORA_REG_MODEM_CONFIG_3, 0x04);
    lora_set_tx_power(E_LORA_POWER_LEVEL_MAX);

    lora_idle();
    return ESP_OK;
}

/**
 * Send a packet.
 * @param buf Data to be sent
 * @param size Size of data.
 */
esp_err_t lora_send_packet(uint8_t *buf, uint8_t size) {
    esp_err_t err;
    uint8_t reg_value;
    /*
     * Transfer data to radio.
     */
    lora_idle();
    if((err = lora_write_reg(E_LORA_REG_FIFO_ADDR_PTR, 0)) != ESP_OK) {
        return err;
    }

    for(int i=0; i<size; i++) {
        if((err = lora_write_reg(E_LORA_REG_FIFO, *buf++)) != ESP_OK) {
            return err;
        }
    }

    if((err = lora_write_reg(E_LORA_REG_PAYLOAD_LENGTH, (uint8_t)size)) != ESP_OK) {
        return err;
    }
    /*
     * Start transmission and wait for conclusion.
     */
    if((err = lora_write_reg(E_LORA_REG_OP_MODE, E_LORA_MODE_LONG_RANGE_MODE | E_LORA_MODE_TX)) != ESP_OK) {
        return err;
    }
    if((err = lora_read_reg(E_LORA_REG_IRQ_FLAGS, &reg_value)) != ESP_OK) {
        return err;
    }
    ESP_LOGD(TAG, "LORA_TRANSMITTING...(REG: 0x%04X)", reg_value);
    while((reg_value & E_LORA_IRQ_TX_DONE_MASK) == 0) {
        if((err = lora_read_reg(E_LORA_REG_IRQ_FLAGS, &reg_value)) != ESP_OK) {
            return err;
        }
        ESP_LOGD(TAG, "LORA_TRANSMITTING...(REG: 0x%04X)", reg_value);
        vTaskDelay(2);
    }

    if((err = lora_write_reg(E_LORA_REG_IRQ_FLAGS, E_LORA_IRQ_TX_DONE_MASK)) != ESP_OK) {
        return err;
    }
    ESP_LOGD(TAG, "LORA_TRANSMIT COMPLETE");
    return ESP_OK;
}

esp_err_t lora_receive_packet(uint8_t *buf, int* size) {
    uint8_t reg_value;
    esp_err_t err;

    if((err = lora_read_reg(E_LORA_REG_IRQ_FLAGS, &reg_value)) != ESP_OK) {
        *buf = 0;
        *size = 0;
        return err;
    }
    uint8_t irq = (uint8_t)reg_value;
    lora_write_reg(E_LORA_REG_IRQ_FLAGS, irq);
    if((irq & E_LORA_IRQ_RX_DONE_MASK) == 0) return LORA_ESP32_ERR_DATA_NOT_RECEIVED;
    if(irq & E_LORA_IRQ_PAYLOAD_CRC_ERROR_MASK) return LORA_ESP32_ERR_DATA_CRC_ERROR;

    if (__implicit) {
        if((err = lora_read_reg(E_LORA_REG_PAYLOAD_LENGTH, &reg_value)) != ESP_OK) {
            buf = NULL;
            *size = 0;
            return err;
        }
        *size = reg_value;
    }
    else {
        if((err = lora_read_reg(E_LORA_REG_RX_NB_BYTES, &reg_value)) != ESP_OK) {
            buf = (uint8_t*)NULL;
            *size = 0;
            return err;
        }
        *size = reg_value;
    }

    /*
     * Transfer data from radio.
     */
    lora_idle();

    if((err = lora_read_reg(E_LORA_REG_FIFO_RX_CURRENT_ADDR, &reg_value)) != ESP_OK) {
        free(buf);
        buf = NULL;
        *size = 0;
        return err;
    }

    if((err = lora_write_reg(E_LORA_REG_FIFO_ADDR_PTR, reg_value)) != ESP_OK) {
        free(buf);
        buf = NULL;
        *size = 0;
        return err;
    }
    for(uint8_t i=0; i<*size; i++) {
        if((err = lora_read_reg(E_LORA_REG_FIFO, &reg_value)) != ESP_OK) {
            *buf++ = 0;
            *size = i;
            return err;
        }
        buf[i] = (uint8_t)reg_value;
    }

    buf[*size] = 0;

    return ESP_OK;
}

/**
 * Returns non-zero if there is data to read (packet received).
 */
bool lora_received() {
    uint8_t reg_value;
    if(lora_read_reg(E_LORA_REG_IRQ_FLAGS, &reg_value) != ESP_OK) {
        return false;
    }
    if(reg_value & E_LORA_IRQ_RX_DONE_MASK) return true;
    return false;
}

/**
 * Return last packet's RSSI.
 */
esp_err_t lora_packet_rssi(int* rssi) {
    uint8_t reg_value;
    esp_err_t err;
    if((err = lora_read_reg(E_LORA_REG_PKT_RSSI_VALUE, &reg_value)) != ESP_OK) {
        *rssi = 0;
        return err;
    }
    *rssi = reg_value - (__frequency < 868E6 ? 164 : 157);
    return ESP_OK;
}

/**
 * Return last packet's SNR (signal to noise ratio).
 */
esp_err_t lora_packet_snr(double* snr) {
    uint8_t reg_value;
    esp_err_t err;
    if((err = lora_read_reg(E_LORA_REG_PKT_SNR_VALUE, &reg_value)) != ESP_OK) {
        *snr = 0;
        return err;
    }
    *snr = (double)(reg_value)*0.25;
    return ESP_OK;
}

/**
 * Shutdown hardware.
 */
esp_err_t lora_close() {
    lora_sleep();
    esp_err_t err;
    if((err = spi_bus_remove_device(esp32_param.spi_device_handle)) != ESP_OK) {
        return err;
    }
    if(!esp32_param.using_preconfigured_spi_bus) {
        return spi_bus_free(esp32_param.spi_host_device);
    }
    return ESP_OK;
}

void lora_dump_registers() {
    int i;
    uint8_t reg_value;
    printf("00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F\n");
    for(i=0; i<0x40; i++) {
        lora_read_reg(i, &reg_value);
        printf("%02X ", (uint8_t)reg_value);
        if((i & 0x0f) == 0x0f) printf("\n");
    }
    printf("\n");
}

