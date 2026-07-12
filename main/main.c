#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stddef.h>
#include <math.h>

#include <mpu6050.h>
#include "esp_timer.h"

// Globais para os ângulos do MPU6050, que poderão ser usados pela task do Mouse
volatile int16_t global_pitch = 0;
volatile int16_t global_roll = 0;

// Offsets de calibração
float gyro_x_offset = -0.84f;
float gyro_y_offset = 0.65f;
float gyro_z_offset = 0.13f;
float accel_x_offset = 0.0217f;
float accel_y_offset = 0.0953f;


#include "driver/spi_slave.h"
#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "host/ble_hs.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"


#define ADDR MPU6050_I2C_ADDRESS_LOW

#define CONFIG_EXAMPLE_MPU6050_I2C_MASTER_SDA 0
#define CONFIG_EXAMPLE_MPU6050_I2C_MASTER_SCL 1

#include "esp_hidd.h"
#include "esp_hid_gap.h"
#include <esp_err.h>


#define RCV_HOST    SPI2_HOST

// #define GPIO_HANDSHAKE      10
#define GPIO_MOSI           6
#define GPIO_MISO           5
#define GPIO_SCLK           4
#define GPIO_CS             7

#define MSB(x) ((uint8_t)((x >> 8) & 0xFF))
#define LSB(x) ((uint8_t)(x & 0xFF))

//Called after a transaction is queued and ready for pickup by master. We use this to set the handshake line high.
void my_post_setup_cb(spi_slave_transaction_t *trans)
{
    // gpio_set_level(GPIO_HANDSHAKE, 1);
}

//Called after transaction is sent/received. We use this to set the handshake line low.
void my_post_trans_cb(spi_slave_transaction_t *trans)
{
    // gpio_set_level(GPIO_HANDSHAKE, 0);
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

void calibrate_mpu6050(mpu6050_dev_t *dev) {
    ESP_LOGI(TAG, "Iniciando calibracao... MANTENHA O SENSOR PARADO!");
    int num_samples = 200; // ~1 segundo
    float sum_gx = 0, sum_gy = 0, sum_gz = 0;
    float sum_ax = 0, sum_ay = 0;
    
    mpu6050_acceleration_t accel;
    mpu6050_rotation_t rotation;

    // Descartar primeiras leituras para estabilizar
    for (int i = 0; i < 50; i++) {
        mpu6050_get_motion(dev, &accel, &rotation);
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    // Acumular amostras
    for (int i = 0; i < num_samples; i++) {
        mpu6050_get_motion(dev, &accel, &rotation);
        sum_gx += rotation.x;
        sum_gy += rotation.y;
        sum_gz += rotation.z;
        sum_ax += accel.x;
        sum_ay += accel.y;
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    // Se preferir hardcodar (colar os valores fixos) depois para nao precisar
    // ficar com a mao parada ao ligar, pegue esses valores do log e substitua
    // as globais de offset no topo do arquivo.
    gyro_x_offset = sum_gx / num_samples;
    gyro_y_offset = sum_gy / num_samples;
    gyro_z_offset = sum_gz / num_samples;
    accel_x_offset = sum_ax / num_samples;
    accel_y_offset = sum_ay / num_samples;
    
    ESP_LOGI(TAG, "Calibracao concluida! Para hardcodar e pular essa etapa, anote os valores abaixo:");
    ESP_LOGI(TAG, "Offsets - Gyro: X=%.2f Y=%.2f Z=%.2f", gyro_x_offset, gyro_y_offset, gyro_z_offset);
    ESP_LOGI(TAG, "Offsets - Accel: X=%.4f Y=%.4f", accel_x_offset, accel_y_offset);
}

void mpu6050_test(void *pvParameters)
{
    mpu6050_dev_t dev = { 0 };

    ESP_ERROR_CHECK(mpu6050_init_desc(&dev, ADDR, 0, 0, 1));

    while (1)
    {
        esp_err_t res = i2c_dev_probe(&dev.i2c_dev, I2C_DEV_WRITE);
        if (res == ESP_OK)
        {
            ESP_LOGI(TAG, "Found MPU6050 device");
            break;
        }
        ESP_LOGE(TAG, "Probe falhou: %s (%d)",
         esp_err_to_name(res), res);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }


    ESP_ERROR_CHECK(mpu6050_init(&dev));

    ESP_LOGI(TAG, "Accel range: %d", dev.ranges.accel);
    ESP_LOGI(TAG, "Gyro range:  %d", dev.ranges.gyro);

    // Chama a calibração antes do loop
    // DICA: Se voce hardcodar os valores globais la no topo, comente a linha abaixo!
    calibrate_mpu6050(&dev);

    float pitch = 0.0f;
    float roll = 0.0f;
    uint64_t last_time = esp_timer_get_time();

    while (1)
    {
        float temp;
        mpu6050_acceleration_t accel = { 0 };
        mpu6050_rotation_t rotation = { 0 };

        ESP_ERROR_CHECK(mpu6050_get_temperature(&dev, &temp));
        ESP_ERROR_CHECK(mpu6050_get_motion(&dev, &accel, &rotation));

        // Aplicar Offsets calibrados
        accel.x -= accel_x_offset;
        accel.y -= accel_y_offset;
        rotation.x -= gyro_x_offset;
        rotation.y -= gyro_y_offset;
        rotation.z -= gyro_z_offset;

        uint64_t current_time = esp_timer_get_time();
        float dt = (current_time - last_time) / 1000000.0f;
        last_time = current_time;

        // Calcular os ângulos usando apenas o acelerômetro
        float accel_pitch = atan2(-accel.x, sqrt(accel.y * accel.y + accel.z * accel.z)) * 180.0 / M_PI;
        float accel_roll = atan2(accel.y, accel.z) * 180.0 / M_PI;

        // Aplicar o Filtro Complementar original (direto)
        pitch = 0.95 * (pitch + rotation.y * dt) + 0.05 * accel_pitch;
        roll = 0.95 * (roll + rotation.x * dt) + 0.05 * accel_roll;
        
        // Atualizar as variáveis globais
        global_pitch = (int16_t)(pitch*10.0f);
        global_roll = (int16_t)(roll*10.0f);

        // ESP_LOGI(TAG, "**********************************************************************");
        // ESP_LOGI(TAG, "Acceleration: x=%.4f   y=%.4f   z=%.4f", accel.x, accel.y, accel.z);
        // ESP_LOGI(TAG, "Rotation:     x=%.4f   y=%.4f   z=%.4f", rotation.x, rotation.y, rotation.z);
        // ESP_LOGI(TAG, "Temperature:  %.1f", temp);
        // ESP_LOGI(TAG, "Angles:       Pitch=%.2f  Roll=%.2f", pitch, roll);

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}


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
        .pin_bit_mask = BIT64(8),
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

        //SendBuf formatting

        sendbuf[0] = (uint8_t)state;
        sendbuf[1] = MSB(global_pitch);
        sendbuf[2] = LSB(global_pitch);
        sendbuf[3] = 0x68;
        sendbuf[4] = MSB(global_roll);
        sendbuf[5] = LSB(global_roll);

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
        // printf("Received X: %u Y: %u\n", x,y);
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
    ESP_ERROR_CHECK(i2cdev_init());
    
    xTaskCreate(mpu6050_test, "mpu6050_test", configMINIMAL_STACK_SIZE * 6, NULL, 5, NULL);

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