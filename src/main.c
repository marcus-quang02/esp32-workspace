#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"

#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"


#define BLINK_GPIO GPIO_NUM_2

#define UART_TX0 4
#define UART_RX0 5
#define BUF_SIZE 128  

#define UART_TX2  17
#define UART_RX2  16

#define EXAMPLE_ESP_WIFI_SSID      "mywifissid"
#define EXAMPLE_ESP_WIFI_PASS      "12345678"
#define EXAMPLE_ESP_WIFI_CHANNEL   10
#define EXAMPLE_MAX_STA_CONN       4

static const char *TAG = "wifi softAP";



int uart_buf_size = 2048;

uint8_t buf[BUF_SIZE];
int len = 0;




void blink_led_task(void *pvParameter)
{
    // Set the GPIO as a push/pull output
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    while (1) {
        // Blink off (output low)
        gpio_set_level(BLINK_GPIO, 0);
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        // Blink on (output high)
        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void blink_led_task_2(void *pvParameter)
{
    // Set the GPIO as a push/pull output
    gpio_set_direction(18, GPIO_MODE_OUTPUT);

    while (1) {
        // Blink off (output low)
        gpio_set_level(18, 0);
        vTaskDelay(750 / portTICK_PERIOD_MS);

        // Blink on (output high)
        gpio_set_level(18, 1);
        vTaskDelay(750 / portTICK_PERIOD_MS);
    }
}

void blink_led_task_3(void *pvParameter)
{
    // Set the GPIO as a push/pull output
    gpio_set_direction(25, GPIO_MODE_OUTPUT);

    while (1) {
        // Blink off (output low)
        gpio_set_level(25, 0);
        vTaskDelay(500 / portTICK_PERIOD_MS);

        // Blink on (output high)
        gpio_set_level(25, 1);
        vTaskDelay(500 / portTICK_PERIOD_MS);
    }
}

void init()
{
    const int uart_num = UART_NUM_0;
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,    
        .rx_flow_ctrl_thresh = 122,

    };

    ESP_ERROR_CHECK(uart_driver_install(uart_num, uart_buf_size, uart_buf_size , 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_num, UART_TX0, UART_RX0, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}


void init2()
{
    const int uart_num = UART_NUM_2;
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,    
        .rx_flow_ctrl_thresh = 122,

    };

    ESP_ERROR_CHECK(uart_driver_install(uart_num, uart_buf_size, uart_buf_size , 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_num, UART_TX2, UART_RX2, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

void tx_task(void *arg)
{
    int num = 0;
    uint8_t* data = (uint8_t*) malloc(30);
    while (1) {
    	int len = sprintf ((char*)data, "Hello world %d\n", num++);
        uart_write_bytes(UART_NUM_0, data, len);
        ESP_LOGI("TX_TASK", "write %d bytes: '%s'", len, data);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    free (data);
}

static void rx2_task(void *arg)
{
    static const char *RX_TASK_TAG = "RX2_TASK";
    esp_log_level_set(RX_TASK_TAG, ESP_LOG_INFO);
    uint8_t* data = (uint8_t*) malloc(BUF_SIZE+1);
    while (1) {
        const int rxBytes = uart_read_bytes(UART_NUM_2, data, BUF_SIZE, 20 / portTICK_PERIOD_MS);
        if (rxBytes > 0) {
            data[rxBytes] = 0;
            //ESP_LOGI(RX_TASK_TAG, "Read %d bytes: '%s'", rxBytes, data);
        }
    }
    free(data);
}


static void wifi_event_handler(void* arg, esp_event_base_t event_base,int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" join, AID=%d",
                 MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d, reason=%d",
                 MAC2STR(event->mac), event->aid, event->reason);
    }
}


void wifi_init_softap(void)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .ssid_len = strlen(EXAMPLE_ESP_WIFI_SSID),
            .channel = EXAMPLE_ESP_WIFI_CHANNEL,
            .password = EXAMPLE_ESP_WIFI_PASS,
            .max_connection = EXAMPLE_MAX_STA_CONN,
#ifdef CONFIG_ESP_WIFI_SOFTAP_SAE_SUPPORT
            .authmode = WIFI_AUTH_WPA3_PSK,
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
#else /* CONFIG_ESP_WIFI_SOFTAP_SAE_SUPPORT */
            .authmode = WIFI_AUTH_WPA2_PSK,
#endif
            .pmf_cfg = {
                    .required = true,
            },
        },
    };
    if (strlen(EXAMPLE_ESP_WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s channel:%d",
             EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS, EXAMPLE_ESP_WIFI_CHANNEL);
}



void app_main(void)
{
    xTaskCreate(&blink_led_task, "blink_led_task", 1024, NULL, 5, NULL);
    xTaskCreate(&blink_led_task_2, "blink_led_task_2", 1024, NULL, 5, NULL);
    xTaskCreate(&blink_led_task_3, "blink_led_task_3", 1024, NULL, 5, NULL);
    ESP_LOGI("Blinkl task 3", "Task created !!");
    init();
    init2();
    //xTaskCreate(rx2_task, "uart_rx2_task", 1024*2, NULL, 5, NULL);

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_AP");
    wifi_init_softap();
}





