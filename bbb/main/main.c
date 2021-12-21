
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include <esp_system.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_sntp.h>
#include <esp_pm.h>

#include <nvs_flash.h>

#include <driver/gpio.h>

#include <lwip/err.h>
#include <lwip/sys.h>

#include "config.h"
#include "mqtt.h"
#include "wifi.h"



#define TAG "main"
#define BLINK_GPIO 5


// --------------------
#include <stdio.h>
#include "esp32/rom/ets_sys.h"
#include "driver/gpio.h"

enum dht11_status
{
    DHT11_CRC_ERROR = -2,
    DHT11_TIMEOUT_ERROR = -1,
    DHT11_OK = 0
};

struct dht11_reading
{
    int status;
    int temperature;
    int humidity;
};


gpio_num_t dht_gpio; // ???? //  Không phải gán chết chân, GPIO num t -> gán chân vào.
int64_t last_read_time = -2000000; // 2 triệu port tick = 2s
struct dht11_reading last_read;
// Check level trong thời gian được truyền ( microseconds).
// Để check xem thử có dht11 có gửi dữ liệu về hay không hay time out.
int waitOrTimeout(uint16_t microSeconds, int level)
{
    int micros_ticks = 0; 
    while (gpio_get_level(dht_gpio) == level)
    {
        if (micros_ticks++ > microSeconds)
            return DHT11_TIMEOUT_ERROR;
        ets_delay_us(1);
    }
    return micros_ticks;
}
int checkCRC(uint8_t data[])
{
    if (data[4] == (data[0] + data[1] + data[2] + data[3]))
        return DHT11_OK;
    else
        return DHT11_CRC_ERROR;
}
void sendStartSignal()
{
    gpio_set_direction(dht_gpio, GPIO_MODE_OUTPUT);
    gpio_set_level(dht_gpio, 0);
    ets_delay_us(20 * 1000);
    gpio_set_level(dht_gpio, 1);
    ets_delay_us(40);
    gpio_set_direction(dht_gpio, GPIO_MODE_INPUT);
}
// Check response from DHT11
int checkResponse()
{
    /* Wait for next step ~80us*/
    if (waitOrTimeout(80, 0) == DHT11_TIMEOUT_ERROR)
        return DHT11_TIMEOUT_ERROR;
    /* Wait for next step ~80us*/
    if (waitOrTimeout(80, 1) == DHT11_TIMEOUT_ERROR)
        return DHT11_TIMEOUT_ERROR;
    return DHT11_OK;

}

struct dht11_reading timeoutError()
{
    struct dht11_reading timeoutError = {DHT11_TIMEOUT_ERROR, -1, -1};
    return timeoutError;
}
struct dht11_reading crcError()
{
    struct dht11_reading crcError = {DHT11_CRC_ERROR, -1, -1};
    return crcError;
}
void DHT11_init(gpio_num_t gpio_num)
{
    /* Wait 1 seconds to make the device
 pass its initial unstable status */
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    dht_gpio = gpio_num;
}
struct dht11_reading DHT11_read()
{
    /* Tried to sense too son since last read
 (dht11 needs ~2 seconds to make a new read) */
 // 1 triệu port tick = 1s.
    if (esp_timer_get_time() - 2000000 < last_read_time)
        return last_read;
    last_read_time = esp_timer_get_time();
    sendStartSignal();
    if (checkResponse() == DHT11_TIMEOUT_ERROR)
        return last_read = timeoutError();
    uint8_t data[5] = {0, 0, 0, 0, 0};
    /* Read response */
    for (int i = 0; i < 40; i++)
    {
        /* Initial data */
        if (waitOrTimeout(50, 0) == DHT11_TIMEOUT_ERROR)
            return last_read = timeoutError();

        if (waitOrTimeout(70, 1) > 28)
            /* Bit received was a 1 */
            data[i / 8] |= (1 << (7 - (i % 8)));
    }
    if (checkCRC(data) != DHT11_CRC_ERROR)
    {
        last_read.status = DHT11_OK;
        last_read.temperature = data[2];
        last_read.humidity = data[0];
        return last_read;
    }
    else
        return last_read = crcError();
}

// ---------------------
// Main task
void main_task(void *pvParameter)
{
   
    gpio_reset_pin(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_INPUT);
    DHT11_init(GPIO_NUM_4);
    while (1)
    {
        int rain;
        vTaskDelay(2000 / portTICK_RATE_MS);
        int temp = DHT11_read().temperature;
        int humi = DHT11_read().humidity;
        int status = DHT11_read().status;
        printf("Temperature: %d \n", temp);
        printf("Humidity: %d \n", humi);

        if (gpio_get_level(BLINK_GPIO) == 0)
        {
            printf("Mua\n");
            rain =1;
            
        }
        else
        {
        printf("Khong mua\n");
        rain =2;
        
        }
        
        printf("Status: %d \n", status);
        if (status == 0) 
        {
            mqtt_handler_publish_values(temp, humi, rain);
            
             
        }
    }
}

void app_main(void)
{
    // initialize NVS
    ESP_ERROR_CHECK(nvs_flash_init());
    // initialize NETIF
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Start Wi-Fi connection
    ESP_LOGI(TAG, "WiFi init");
    wifi_init();

    // Init and connect to MQTT
    ESP_LOGI(TAG, "Connecting to MQTT...");
    mqtt_init();
    mqtt_handler_start();

    // start the main task
    xTaskCreate(&main_task, "main_task", 2048, NULL, 5, NULL);
}

