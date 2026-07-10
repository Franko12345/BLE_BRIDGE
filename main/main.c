/*
 * SPDX-FileCopyrightText: 2021-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 *
 * Versao enxuta do exemplo esp_hid_device:
 *   - Somente NimBLE (Bluetooth Classic, SDP e Bluedroid foram removidos)
 *   - Somente role de MOUSE (teclado e consumer control removidos)
 *   - Report Map alterado para coordenadas ABSOLUTAS (X/Y de 16 bits, 0-32767)
 *
 * Pre-requisito no menuconfig:
 *   Component config -> Bluetooth -> Host = NimBLE - Host
 *   Component config -> Bluetooth -> Bluetooth Controller = Enabled (BLE only)
 */

#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stddef.h>

#include "driver/spi_slave.h"
#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "host/ble_hs.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"


#include "esp_hidd.h"
#include "esp_hid_gap.h"


#define RCV_HOST    SPI2_HOST

#define GPIO_HANDSHAKE      10
#define GPIO_MOSI           6
#define GPIO_MISO           5
#define GPIO_SCLK           4
#define GPIO_CS             7

//Called after a transaction is queued and ready for pickup by master. We use this to set the handshake line high.
void my_post_setup_cb(spi_slave_transaction_t *trans)
{
    gpio_set_level(GPIO_HANDSHAKE, 1);
}

//Called after transaction is sent/received. We use this to set the handshake line low.
void my_post_trans_cb(spi_slave_transaction_t *trans)
{
    gpio_set_level(GPIO_HANDSHAKE, 0);
}


static const char *TAG = "HID_MOUSE_DEMO";

enum BT_STATE{
    BT_CONNECTED,
    BT_DISCONNECTED
};

static volatile enum BT_STATE state = BT_DISCONNECTED;

typedef struct {
    TaskHandle_t task_hdl;
    esp_hidd_dev_t *hid_dev;
} local_param_t;

static local_param_t s_ble_hid_param = {0};

/* ---------------------------------------------------------------------
 * Report Map: mouse com posicao ABSOLUTA
 * Formato do relatorio enviado (5 bytes):
 *   [0] botoes (3 bits usados + 5 de padding)
 *   [1-2] X (16 bits, little endian, 0..32767)
 *   [3-4] Y (16 bits, little endian, 0..32767)
 * ------------------------------------------------------------------- */
static const unsigned char mouseReportMap[] = {
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x02,        // Usage (Mouse)
    0xA1, 0x01,        // Collection (Application)
    0x09, 0x01,        //   Usage (Pointer)
    0xA1, 0x00,        //   Collection (Physical)

    0x05, 0x09,        //     Usage Page (Button)
    0x19, 0x01,        //     Usage Minimum (Button 1)
    0x29, 0x03,        //     Usage Maximum (Button 3)
    0x15, 0x00,        //     Logical Minimum (0)
    0x25, 0x01,        //     Logical Maximum (1)
    0x95, 0x03,        //     Report Count (3)
    0x75, 0x01,        //     Report Size (1)
    0x81, 0x02,        //     Input (Data,Var,Abs)   -- 3 bits de botoes
    0x95, 0x01,        //     Report Count (1)
    0x75, 0x05,        //     Report Size (5)
    0x81, 0x03,        //     Input (Const,Var,Abs)  -- padding

    0x05, 0x01,        //     Usage Page (Generic Desktop)
    0x09, 0x30,        //     Usage (X)
    0x09, 0x31,        //     Usage (Y)
    0x16, 0x00, 0x00,  //     Logical Minimum (0)
    0x26, 0xFF, 0x7F,  //     Logical Maximum (32767)
    0x75, 0x10,        //     Report Size (16)
    0x95, 0x02,        //     Report Count (2)
    0x81, 0x02,        //     Input (Data,Var,Abs)   -- X e Y absolutos

    0xC0,              //   End Collection
    0xC0               // End Collection
};

static esp_hid_raw_report_map_t ble_report_maps[] = {
    {
        .data = mouseReportMap,
        .len  = sizeof(mouseReportMap)
    },
};

static esp_hid_device_config_t ble_hid_config = {
    .vendor_id         = 0x16C0,
    .product_id        = 0x05DF,
    .version           = 0x0100,
    .device_name       = "ESP32-C3 Mouse",
    .manufacturer_name = "Espressif",
    .serial_number     = "1234567890",
    .report_maps       = ble_report_maps,
    .report_maps_len   = 1
};

/* Envia posicao absoluta (0..32767) + estado dos botoes */
void send_mouse_absolute(uint8_t buttons, uint16_t x, uint16_t y)
{
    static uint8_t buffer[5] = {0};
    buffer[0] = buttons;
    buffer[1] = (uint8_t)(x & 0xFF);
    buffer[2] = (uint8_t)(x >> 8);
    buffer[3] = (uint8_t)(y & 0xFF);
    buffer[4] = (uint8_t)(y >> 8);
    esp_hidd_dev_input_set(s_ble_hid_param.hid_dev, 0, 0, buffer, sizeof(buffer));
}

/* Task de demonstracao via serial: move o cursor para posicoes fixas
 * usando teclas de teste. Substitua pela sua fonte real de coordenadas
 * (touchscreen, sensor, etc.) */

static void spi_reciever_thread(void *pvParameters){
        int n = 0;
    esp_err_t ret;

    //Configuration for the SPI bus
    spi_bus_config_t buscfg = {
        .mosi_io_num = GPIO_MOSI,
        .miso_io_num = GPIO_MISO,
        .sclk_io_num = GPIO_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };

    //Configuration for the SPI slave interface
    spi_slave_interface_config_t slvcfg = {
        .mode = 0,
        .spics_io_num = GPIO_CS,
        .queue_size = 3,
        .flags = 0,
        .post_setup_cb = my_post_setup_cb,
        .post_trans_cb = my_post_trans_cb
    };

    //Configuration for the handshake line
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = BIT64(GPIO_HANDSHAKE),
    };

    //Configure handshake line as output
    gpio_config(&io_conf);
    //Enable pull-ups on SPI lines so we don't detect rogue pulses when no master is connected.
    gpio_set_pull_mode(GPIO_MOSI, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_SCLK, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(GPIO_CS, GPIO_PULLUP_ONLY);



    //Initialize SPI slave interface
    ret = spi_slave_initialize(RCV_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO);
    assert(ret == ESP_OK);

    char *sendbuf = spi_bus_dma_memory_alloc(RCV_HOST, 6, 0);
    uint8_t *recvbuf = spi_bus_dma_memory_alloc(RCV_HOST, 6, 0);
    assert(sendbuf && recvbuf);
    spi_slave_transaction_t t = {0};
    uint16_t x,y;
    while (1) {
        //Clear receive buffer, set send buffer to something sane
        memset(recvbuf, 0, 6);
        sprintf(sendbuf, "AAAAA");

        //Set up a transaction of 128 bytes to send/receive
        t.length = 6 * 8;
        t.tx_buffer = sendbuf;
        t.rx_buffer = recvbuf;
        /* This call enables the SPI slave interface to send/receive to the sendbuf and recvbuf. The transaction is
        initialized by the SPI master, however, so it will not actually happen until the master starts a hardware transaction
        by pulling CS low and pulsing the clock etc. In this specific example, we use the handshake line, pulled up by the
        .post_setup_cb callback that is called as soon as a transaction is ready, to let the master know it is free to transfer
        data.
        */
        ret = spi_slave_transmit(RCV_HOST, &t, portMAX_DELAY);
        
        //spi_slave_transmit does not return until the master has done a transmission, so by here we have sent our data and
        //received data from the master. Print it.
        
        if(recvbuf[3] != 0x67) continue;

        x = (recvbuf[1] << 8) + recvbuf[2];
        y = (recvbuf[4] << 8) + recvbuf[5];
        printf("Received X: %u Y: %u\n", x,y);
        if (state == BT_CONNECTED)
            send_mouse_absolute(0x00, x, y);

        // ret = spi_slave_disable(RCV_HOST);
        // if (ret == ESP_OK) {
            // printf("slave paused ...\n");
        // }
        // vTaskDelay(10);    //now is able to sleep or do something to save power, any following transaction will be ignored
        // ret = spi_slave_enable(RCV_HOST);
        // if (ret == ESP_OK) {
            // printf("slave ready !\n");
        // }
        n++;
    }
}


void ble_hid_task_start_up(void)
{
    if (s_ble_hid_param.task_hdl) {
        return; // task ja existe
    }
    // xTaskCreate(ble_hid_demo_task_mouse, "ble_hid_demo_task_mouse", 3 * 1024, NULL,
                // configMAX_PRIORITIES - 3, &s_ble_hid_param.task_hdl);
}

static void ble_hid_task_shut_down(void)
{
    if (s_ble_hid_param.task_hdl) {
        vTaskDelete(s_ble_hid_param.task_hdl);
        s_ble_hid_param.task_hdl = NULL;
    }
}

static void ble_hidd_event_callback(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
{
    esp_hidd_event_t event = (esp_hidd_event_t)id;
    esp_hidd_event_data_t *param = (esp_hidd_event_data_t *)event_data;

    switch (event) {
    case ESP_HIDD_START_EVENT:
        ESP_LOGI(TAG, "START");
        esp_hid_ble_gap_adv_start();
        break;

    case ESP_HIDD_CONNECT_EVENT:
        ESP_LOGI(TAG, "CONNECT");
        state = BT_CONNECTED;
        break;

    case ESP_HIDD_PROTOCOL_MODE_EVENT:
        ESP_LOGI(TAG, "PROTOCOL MODE[%u]: %s", param->protocol_mode.map_index,
                 param->protocol_mode.protocol_mode ? "REPORT" : "BOOT");
        break;

    case ESP_HIDD_CONTROL_EVENT:
        ESP_LOGI(TAG, "CONTROL[%u]: %sSUSPEND", param->control.map_index,
                 param->control.control ? "EXIT_" : "");
        if (param->control.control) {
            ble_hid_task_start_up();   // sai do suspend
        } else {
            ble_hid_task_shut_down();  // entra em suspend
        }
        break;

    case ESP_HIDD_OUTPUT_EVENT:
        ESP_LOGI(TAG, "OUTPUT[%u]: %8s ID: %2u, Len: %d, Data:",
                 param->output.map_index, esp_hid_usage_str(param->output.usage),
                 param->output.report_id, param->output.length);
        ESP_LOG_BUFFER_HEX(TAG, param->output.data, param->output.length);
        break;

    case ESP_HIDD_DISCONNECT_EVENT:
        state = BT_DISCONNECTED;
        ESP_LOGI(TAG, "DISCONNECT: %s",
                 esp_hid_disconnect_reason_str(esp_hidd_dev_transport_get(param->disconnect.dev),
                                                param->disconnect.reason));
        ble_hid_task_shut_down();
        esp_hid_ble_gap_adv_start();
        break;

    case ESP_HIDD_STOP_EVENT:
        ESP_LOGI(TAG, "STOP");
        break;

    default:
        break;
    }
}

static void ble_hid_device_host_task(void *param)
{
    ESP_LOGI(TAG, "BLE Host Task Started");
    nimble_port_run();      // so retorna quando nimble_port_stop() for chamado
    nimble_port_freertos_deinit();
}

/* Definida em store.c do exemplo original (persistencia de bonding) */
void ble_store_config_init(void);

void app_main(void)
{
   xTaskCreate(spi_reciever_thread, "SPI Reciever", 1024* 4, NULL, configMAX_PRIORITIES - 3, NULL);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "setting hid gap, mode: BLE");
    ESP_ERROR_CHECK(esp_hid_gap_init(HID_DEV_MODE));

    ESP_ERROR_CHECK(esp_hid_ble_gap_adv_init(ESP_HID_APPEARANCE_MOUSE, ble_hid_config.device_name));

    ESP_LOGI(TAG, "setting ble device");
    ESP_ERROR_CHECK(esp_hidd_dev_init(&ble_hid_config, ESP_HID_TRANSPORT_BLE,
                                       ble_hidd_event_callback, &s_ble_hid_param.hid_dev));

    ble_store_config_init();
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    nimble_port_freertos_init(ble_hid_device_host_task);
}