#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stddef.h>
#include <math.h>

#include <mpu6050.h>
#include "esp_timer.h"

// Globais para os ângulos do MPU6050, que poderão ser usados pela task do Mouse
volatile int32_t global_pitch = 0;
volatile int32_t global_roll = 0;
volatile int32_t global_yaw = 0;
volatile int32_t global_x_accel = 0;

// Offsets de calibração (Apenas Giroscópio)
float gyro_x_offset = -0.73f;
float gyro_y_offset = 1.07f;
float gyro_z_offset = -0.13f;
#define GYRO_DEADBAND 0.3f


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

#define MSB_1_32bits(x) ((uint8_t)((x >> 24) & 0xFF))
#define MSB_2_32bits(x) ((uint8_t)((x >> 16) & 0xFF))
#define MSB_3_32bits(x) ((uint8_t)((x >> 8) & 0xFF))
#define MSB_4_32bits(x) ((uint8_t)((x >> 0) & 0xFF))

#define MSB(x) ((uint8_t)((((int16_t)x) >> 8) & 0xFF))
#define LSB(x) ((uint8_t)(((int16_t)x) & 0xFF))

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

enum OP_MODE{
  IDLE_MODE,
  CURSOR_MODE,
  SCROLL_MODE
};

static volatile enum BT_STATE state = BT_DISCONNECTED;
static volatile enum OP_MODE mode = IDLE_MODE;


typedef struct {
    TaskHandle_t task_hdl;
    esp_hidd_dev_t *hid_dev;
} local_param_t;

static local_param_t s_ble_hid_param = {0};

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

    // --- Wheel (scroll), relativo, agora 16 bits ---
    0x09, 0x38,        //     Usage (Wheel)
    0x16, 0x01, 0x80,  //     Logical Minimum (-32767)
    0x26, 0xFF, 0x7F,  //     Logical Maximum (32767)
    0x75, 0x10,        //     Report Size (16)
    0x95, 0x01,        //     Report Count (1)
    0x81, 0x06,        //     Input (Data,Var,Rel)
 

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

// Sinais de montagem — a determinar com o teste de poses que te passei
#define ACCEL_SIGN_X  1
#define ACCEL_SIGN_Y  1
#define ACCEL_SIGN_Z  -1

#define GYRO_SIGN_X   1
#define GYRO_SIGN_Y   1
#define GYRO_SIGN_Z   1

static inline void apply_mounting_transform(mpu6050_acceleration_t *a, mpu6050_rotation_t *r)
{
    float ax = a->x, ay = a->y, az = a->z;
    float rx = r->x, ry = r->y, rz = r->z;

    // Acelerômetro: papel de "frente" (pitch) vem do Y físico, "direita" (roll) vem do X físico
    a->x = -ay;   // logical X (usado no cálculo de pitch) = -physical Y
    a->y = -ax;   // logical Y (usado no cálculo de roll)  = -physical X
    a->z = -az;   // logical Z (vertical)                  = -physical Z

    // Giroscópio sofre a MESMA rotação física, então mesma transformação
    r->x = -ry;
    r->y = -rx;
    r->z = -rz;
}

static inline void apply_sensor_filters(mpu6050_acceleration_t *a,
                                        mpu6050_rotation_t *r)
{
    /* ---------- Configuração ---------- */

    const float accel_alpha = 0.40f;
    const float gyro_alpha  = 0.45f;

    const float gyro_deadband = 0.30f;

    /* ---------- Estado interno ---------- */

    static float accel_x = 0.0f;
    static float accel_y = 0.0f;
    static float accel_z = 0.0f;

    static float gyro_x = 0.0f;
    static float gyro_y = 0.0f;
    static float gyro_z = 0.0f;

    static bool initialized = false;

    /* ---------- Inicialização ---------- */

    if (!initialized)
    {
        accel_x = a->x;
        accel_y = a->y;
        accel_z = a->z;

        gyro_x = r->x;
        gyro_y = r->y;
        gyro_z = r->z;

        initialized = true;
    }

    /* ---------- Deadband do giroscópio ---------- */

    if (fabsf(r->x) < gyro_deadband)
        r->x = 0.0f;

    if (fabsf(r->y) < gyro_deadband)
        r->y = 0.0f;

    if (fabsf(r->z) < gyro_deadband)
        r->z = 0.0f;

    /* ---------- Filtro exponencial do acelerômetro ---------- */

    accel_x += accel_alpha * (a->x - accel_x);
    accel_y += accel_alpha * (a->y - accel_y);
    accel_z += accel_alpha * (a->z - accel_z);

    /* ---------- Filtro exponencial do giroscópio ---------- */

    gyro_x += gyro_alpha * (r->x - gyro_x);
    gyro_y += gyro_alpha * (r->y - gyro_y);
    gyro_z += gyro_alpha * (r->z - gyro_z);

    /* ---------- Retorna valores filtrados ---------- */

    a->x = accel_x;
    a->y = accel_y;
    a->z = accel_z;

    r->x = gyro_x;
    r->y = gyro_y;
    r->z = gyro_z;
}

static inline void apply_angle_filters(float *pitch, float *roll)
{
    const float alpha = 0.60f;
    const float max_step = 3.0f;

    static float filtered_pitch = 0.0f;
    static float filtered_roll = 0.0f;

    static bool initialized = false;

    if (!initialized)
    {
        filtered_pitch = *pitch;
        filtered_roll  = *roll;
        initialized = true;
    }

    /* Filtro exponencial */

    filtered_pitch += alpha * (*pitch - filtered_pitch);
    filtered_roll  += alpha * (*roll  - filtered_roll);

    /* Limitador de velocidade */

    float dp = filtered_pitch - *pitch;
    float dr = filtered_roll  - *roll;

    if (fabsf(dp) > max_step)
        filtered_pitch = *pitch + copysignf(max_step, dp);

    if (fabsf(dr) > max_step)
        filtered_roll = *roll + copysignf(max_step, dr);

    *pitch = filtered_pitch;
    *roll  = filtered_roll;
}

static inline void apply_complementary_filter(float *pitch,
                                              float *roll,
                                              float gyro_x,
                                              float gyro_y,
                                              float accel_pitch,
                                              float accel_roll,
                                              float dt)
{
    const float tau = 0.18f;

    float alpha = tau / (tau + dt);

    *pitch = alpha * (*pitch + gyro_y * dt)
           + (1.0f - alpha) * accel_pitch;

    *roll = alpha * (*roll + gyro_x * dt)
          + (1.0f - alpha) * accel_roll;
}
void calibrate_mpu6050(mpu6050_dev_t *dev) {
    ESP_LOGI(TAG, "Iniciando calibracao... MANTENHA O SENSOR PARADO!");
    int num_samples = 200;
    float sum_gx = 0, sum_gy = 0, sum_gz = 0;

    mpu6050_acceleration_t accel;
    mpu6050_rotation_t rotation;

    for (int i = 0; i < 50; i++) {
        mpu6050_get_motion(dev, &accel, &rotation);
        apply_mounting_transform(&accel, &rotation);   // <-- adicionado
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    for (int i = 0; i < num_samples; i++) {
        mpu6050_get_motion(dev, &accel, &rotation);
        apply_mounting_transform(&accel, &rotation);   // <-- adicionado
        sum_gx += rotation.x;
        sum_gy += rotation.y;
        sum_gz += rotation.z;
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    gyro_x_offset = sum_gx / num_samples;
    gyro_y_offset = sum_gy / num_samples;
    gyro_z_offset = sum_gz / num_samples;

    ESP_LOGI(TAG, "Calibracao concluida! Offsets - Gyro: X=%.2f Y=%.2f Z=%.2f",
             gyro_x_offset, gyro_y_offset, gyro_z_offset);
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
    
    ESP_ERROR_CHECK(mpu6050_set_dlpf_mode(&dev, MPU6050_DLPF_3));
    
    ESP_LOGI(TAG, "Accel range: %d", dev.ranges.accel);
    ESP_LOGI(TAG, "Gyro range:  %d", dev.ranges.gyro);
    
    // Chama a calibração antes do loop
    // DICA: Se voce hardcodar os valores globais la no topo, comente a linha abaixo!
    calibrate_mpu6050(&dev);
    
    float pitch = 0.0f;
    float roll = 0.0f;
    float yaw = 0.0f;

    uint64_t last_time = esp_timer_get_time();
    
    TickType_t lastWakeTime = xTaskGetTickCount();
    while (1)
    {
        mpu6050_acceleration_t accel = { 0 };
        mpu6050_rotation_t rotation = { 0 };
        
        ESP_ERROR_CHECK(mpu6050_get_motion(&dev, &accel, &rotation));
        
        apply_mounting_transform(&accel, &rotation);
        
        // ESP_LOGI(TAG, "RAW accel: x=%.3f y=%.3f z=%.3f", accel.x, accel.y, accel.z);

        // Aplicar Offsets calibrados (apenas giroscópio)
        rotation.x -= gyro_x_offset;
        rotation.y -= gyro_y_offset;
        rotation.z -= gyro_z_offset;

        apply_sensor_filters(&accel, &rotation);

        uint64_t current_time = esp_timer_get_time();
        float dt = (current_time - last_time) / 1000000.0f;
        last_time = current_time;

        if (dt > 0.02f)
            dt = 0.02f;
        
        if (dt < 0.001f)
            dt = 0.001f;


        // Calcular os ângulos usando apenas o acelerômetro
        float accel_pitch =
            atan2f(-accel.x, sqrtf(accel.y * accel.y + accel.z * accel.z))
            * 180.0f / (float)M_PI;

        float accel_roll = atan2f(accel.y, accel.z)* 180.0f / (float)M_PI;

        // Aplicar o Filtro Complementar original (direto) para Pitch e Roll
        apply_complementary_filter(
            &pitch,
            &roll,
            rotation.x,
            rotation.y,
            accel_pitch,
            accel_roll,
            dt);

        apply_angle_filters(&pitch, &roll);

        // Calcular Yaw apenas por integração do giroscópio (sem acelerômetro)
        yaw = (mode != IDLE_MODE) ? yaw - rotation.z * dt : 0.0;
        
        // Atualizar as variáveis globais
        global_pitch = (int32_t)(-pitch*1000);
        global_roll = (int32_t)(roll*1000);
        global_yaw = (int32_t)(yaw*1000);
        global_x_accel = (int32_t)(accel.x*1000);

        // ESP_LOGI(TAG, "**********************************************************************");
        // ESP_LOGI(TAG, "Acceleration: x=%.4f   y=%.4f   z=%.4f", accel.x, accel.y, accel.z);
        // ESP_LOGI(TAG, "Rotation:     x=%.4f   y=%.4f   z=%.4f", rotation.x, rotation.y, rotation.z);
        // ESP_LOGI(TAG, "Temperature:  %.1f", temp);
        // ESP_LOGI(TAG, "Angles: Pitch=%.2f  Roll=%.2f  Yaw=%.2f", -pitch, roll, yaw);

        vTaskDelay(pdMS_TO_TICKS(3));
    }
}


/* Envia posicao absoluta (0..32767) + estado dos botoes */
void send_mouse_absolute(uint8_t buttons, uint16_t x, uint16_t y, int8_t wheel)
{
    static uint8_t buffer[7] = {0};
    buffer[0] = buttons;
    buffer[1] = (uint8_t)(x & 0xFF);
    buffer[2] = (uint8_t)(x >> 8);
    buffer[3] = (uint8_t)(y & 0xFF);
    buffer[4] = (uint8_t)(y >> 8);
    buffer[5] = (uint8_t)(wheel & 0xFF);
    buffer[6] = (uint8_t)(wheel >> 8);
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

    size_t SPI_BUFF_SIZE = 16;

    char *sendbuf = spi_bus_dma_memory_alloc(RCV_HOST, SPI_BUFF_SIZE, 0);
    uint8_t *recvbuf = spi_bus_dma_memory_alloc(RCV_HOST, SPI_BUFF_SIZE, 0);
    assert(sendbuf && recvbuf);
    spi_slave_transaction_t t = {0};
    uint16_t x,y;
    while (1) {
        //Clear receive buffer, set send buffer to something sane
        memset(recvbuf, 0, SPI_BUFF_SIZE);

        //SendBuf formatting

        sendbuf[0] = 0x68;
        sendbuf[1] = (uint8_t)state;
        sendbuf[2] = MSB_1_32bits(global_pitch);
        sendbuf[3] = MSB_2_32bits(global_pitch);
        sendbuf[4] = MSB_3_32bits(global_pitch);
        sendbuf[5] = MSB_4_32bits(global_pitch);
        sendbuf[6] = MSB_1_32bits(global_roll);
        sendbuf[7] = MSB_2_32bits(global_roll);
        sendbuf[8] = MSB_3_32bits(global_roll);
        sendbuf[9] = MSB_4_32bits(global_roll);
        sendbuf[10] = MSB_1_32bits(global_yaw);
        sendbuf[11] = MSB_2_32bits(global_yaw);
        sendbuf[12] = MSB_3_32bits(global_yaw);
        sendbuf[13] = MSB_4_32bits(global_yaw);
        sendbuf[14] = MSB(global_x_accel);
        sendbuf[15] = LSB(global_x_accel);

        //Set up a transaction of 128 bytes to send/receive
        t.length = SPI_BUFF_SIZE * 8;
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
        
        if(recvbuf[0] != 0x67) continue;

        mode = (enum OP_MODE)recvbuf[1];
        x = (recvbuf[2] << 8) + recvbuf[3];
        y = (recvbuf[4] << 8) + recvbuf[5];
        uint8_t buttons = recvbuf[6];
        int16_t wheel = (recvbuf[7] << 8) + recvbuf[8];
        // printf("Yaw: %li Pitch: %li Roll: %li\n", global_yaw, global_pitch, global_roll);
        // printf("State: %u  Received X: %u Y: %u Buttons: %u  Whell: %i\n", recvbuf[1], x, y, buttons, wheel);
        if (state == BT_CONNECTED)
            send_mouse_absolute(buttons, x, y, wheel);

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
    // Chamado apos ENC_CHANGE com sucesso (link ja criptografado). So aqui
    // liberamos o envio de reports HID, para nao notificar num link ainda
    // nao criptografado/sem subscricao durante a reconexao.
    state = BT_CONNECTED;

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
        // NAO marca BT_CONNECTED aqui: o link ainda nao esta criptografado.
        // O envio de reports so e liberado em ble_hid_task_start_up(), chamado
        // apos BLE_GAP_EVENT_ENC_CHANGE com sucesso.
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