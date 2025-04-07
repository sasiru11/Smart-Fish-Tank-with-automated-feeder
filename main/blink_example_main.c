#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_event.h"
#include "nvs_flash.h"
//SNTP
#include <time.h>
#include <sys/time.h>
#include "esp_attr.h"
#include "esp_sleep.h"
#include "esp_sntp.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#define TAG "FISH TANK"

//Wifi SSID and Password
#define EXAMPLE_ESP_WIFI_SSID      "RRBJ-MIFI"
#define EXAMPLE_ESP_WIFI_PASS      "0099ravindu"

//Bulb Pin
#define BULB1_GPIO_PIN 2  //Bulb Light Pin
#define BULB2_GPIO_PIN 22 //Bulb Heeter Pin
#define BULB3_GPIO_PIN 23 //Bulb Filter Pin

//DS18B20 Pin
#define DS18B20_GPIO 4

//LIGHT Relay
#define LIGHT_RELAY_GPIO 21

//Heter Relay
#define RELAY_GPIO 5
#define TEMP_THRESHOLD_LOW  25.0
#define TEMP_THRESHOLD_HIGH 26.5

//TURBIDITY Pin
#define TURBIDITY_SENSOR_PIN ADC1_CHANNEL_6  // GPIO34 (ADC1 channel 6)
#define ADC_WIDTH ADC_WIDTH_BIT_12           // 12-bit ADC resolution
#define ADC_ATTEN ADC_ATTEN_DB_0            // ADC attenuation (0dB for 0-1V range)

//Filter Relay
#define FILTER_RELAY_GPIO 18
#define TURBIDITY_THRESHOLD_HIGH 3200
#define TURBIDITY_THRESHOLD_LOW 3300


//WIFI and TIME Code START FROM HERE

static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
static const char *TAG1 = "wifi station";

char Current_Date_Time[100];


void time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGI(TAG1, "Notification of a time synchronization event");
}
static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
	{
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
	{
       // if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY)
		//{
            esp_wifi_connect();
           // s_retry_num++;
            ESP_LOGI(TAG1, "retry to connect to the AP");
        //} else {
        //    xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        ESP_LOGI(TAG1,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG1, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        //s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}


void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

	ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                    WIFI_EVENT_STA_DISCONNECTED,
                    &event_handler,
                    NULL,
                    NULL));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
	     .threshold.authmode = WIFI_AUTH_WPA2_PSK,

            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG1, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG1, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG1, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG1, "UNEXPECTED EVENT");
    }

    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);
}



void Get_current_date_time(char *date_time){
	char strftime_buf[64];
	time_t now;
	    struct tm timeinfo;
	    time(&now);
	    localtime_r(&now, &timeinfo);

	    	// Set timezone to Indian Standard Time
	    	    setenv("TZ", "UTC-05:30", 1);
	    	    tzset();
	    	    localtime_r(&now, &timeinfo);

	    	    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
	    	    ESP_LOGI(TAG1, "The current date/time in Delhi is: %s", strftime_buf);
                strcpy(date_time,strftime_buf);
}


static void initialize_sntp(void)
{
    ESP_LOGI(TAG1, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_set_time_sync_notification_cb(time_sync_notification_cb);
#ifdef CONFIG_SNTP_TIME_SYNC_METHOD_SMOOTH
    sntp_set_sync_mode(SNTP_SYNC_MODE_SMOOTH);
#endif
    sntp_init();
}
static void obtain_time(void)
{


    initialize_sntp();
    // wait for time to be set
    time_t now = 0;
    struct tm timeinfo = { 0 };
    int retry = 0;
    const int retry_count = 10;
    while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count) {
        ESP_LOGI(TAG1, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    time(&now);
    localtime_r(&now, &timeinfo);
}
 void Set_SystemTime_SNTP()  {

	 time_t now;
	    struct tm timeinfo;
	    time(&now);
	    localtime_r(&now, &timeinfo);
	    // Is time set? If not, tm_year will be (1970 - 1900).
	    if (timeinfo.tm_year < (2016 - 1900)) {
	        ESP_LOGI(TAG1, "Time is not set yet. Connecting to WiFi and getting time over NTP.");
	        obtain_time();
	        // update 'now' variable with current time
	        time(&now);
	    }
}
//WIFI AND TIME CODE END HERE

//Control bulb based on time function
void control_bulb_based_on_time(void) {
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);

    int hour = timeinfo.tm_hour;
    int min = timeinfo.tm_min;

    gpio_reset_pin(LIGHT_RELAY_GPIO);
    gpio_set_direction(LIGHT_RELAY_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(FILTER_RELAY_GPIO, 1);

    // Check if the current time is between 06:00 AM and 06:00 PM
    if ((hour > 18 || (hour == 18 && min >= 00)) && (hour < 22 || (hour == 22 && min < 00))) {
        // Turn on the bulb (GPIO High)

        gpio_set_level(BULB1_GPIO_PIN, 1); // Turn ON  
        gpio_set_level(LIGHT_RELAY_GPIO, 0);
        ESP_LOGI(TAG, "Bulb ON");    

    } else {
        // Turn off the bulb (GPIO Low)
        gpio_set_level(BULB1_GPIO_PIN, 0); // Turn OFF
        gpio_set_level(LIGHT_RELAY_GPIO, 1);
        ESP_LOGI(TAG, "Bulb OFF");
    }

}


// Utility: delay in microseconds
static void delay_us(uint32_t us) {
    esp_rom_delay_us(us);
}

// Set pin as output/input
static void ds18b20_set_pin_output() {
    gpio_set_direction(DS18B20_GPIO, GPIO_MODE_OUTPUT);
}

static void ds18b20_set_pin_input() {
    gpio_set_direction(DS18B20_GPIO, GPIO_MODE_INPUT);
}

static void ds18b20_write_pin(uint8_t level) {
    gpio_set_level(DS18B20_GPIO, level);
}

static int ds18b20_read_pin() {
    return gpio_get_level(DS18B20_GPIO);
}

// Send reset pulse
bool ds18b20_reset() {
    ds18b20_set_pin_output();
    ds18b20_write_pin(0);
    delay_us(480); // pull low for reset
    ds18b20_set_pin_input();
    delay_us(70); // wait for presence pulse
    int presence = ds18b20_read_pin() == 0;
    delay_us(410); // wait until end of timeslot
    return presence;
}

// Write 1 bit
void ds18b20_write_bit(int bit) {
    ds18b20_set_pin_output();
    ds18b20_write_pin(0);
    delay_us(bit ? 6 : 60);
    ds18b20_write_pin(1);
    delay_us(bit ? 64 : 10);
}

// Read 1 bit
int ds18b20_read_bit() {
    int bit;
    ds18b20_set_pin_output();
    ds18b20_write_pin(0);
    delay_us(6);
    ds18b20_set_pin_input();
    delay_us(9);
    bit = ds18b20_read_pin();
    delay_us(55);
    return bit;
}

// Write 1 byte
void ds18b20_write_byte(uint8_t byte) {
    for (int i = 0; i < 8; i++) {
        ds18b20_write_bit(byte & 0x01);
        byte >>= 1;
    }
}

// Read 1 byte
uint8_t ds18b20_read_byte() {
    uint8_t byte = 0;
    for (int i = 0; i < 8; i++) {
        byte >>= 1;
        if (ds18b20_read_bit()) {
            byte |= 0x80;
        }
    }
    return byte;
}

// Start temperature conversion
void ds18b20_start_conversion() {
    ds18b20_reset();
    ds18b20_write_byte(0xCC); // Skip ROM
    ds18b20_write_byte(0x44); // Convert T
}

// Read temperature from scratchpad
float ds18b20_read_temperature() {
    uint8_t lsb, msb;
    int16_t raw;

    ds18b20_reset();
    ds18b20_write_byte(0xCC); // Skip ROM
    ds18b20_write_byte(0xBE); // Read Scratchpad

    lsb = ds18b20_read_byte();
    msb = ds18b20_read_byte();

    raw = (msb << 8) | lsb;
    return raw / 16.0;
}

// Turbidity sensor setup
void turbidity_sensor_init() {
    adc1_config_width(ADC_WIDTH);           // Set ADC width
    adc1_config_channel_atten(TURBIDITY_SENSOR_PIN, ADC_ATTEN); // Set attenuation
}

// Read turbidity sensor value
int read_turbidity() {
    return adc1_get_raw(TURBIDITY_SENSOR_PIN);  // Read the ADC value
}

// Relay control
void relay_init() {
    gpio_reset_pin(RELAY_GPIO);
    gpio_reset_pin(BULB2_GPIO_PIN);
    gpio_set_direction(RELAY_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(BULB2_GPIO_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(RELAY_GPIO, 1); // Initially OFF (active-low)

}

void relay_on() {
    gpio_set_level(RELAY_GPIO, 0); // Active-low: 0 = ON BULB2_GPIO_PIN
    gpio_set_level(BULB2_GPIO_PIN, 1);  
    ESP_LOGI(TAG, "Heater turned ON");
}

void relay_off() {
    gpio_set_level(RELAY_GPIO, 1); // Active-low: 1 = OFF
    gpio_set_level(BULB2_GPIO_PIN, 0);  
    ESP_LOGI(TAG, "Heater turned OFF");
}

float convert_adc_to_ntu(int adc_value) {
    // Example linear map: Adjust based on your sensor's datasheet
    return (3300.0 - adc_value) * (100.0 / 3300.0);
}

//Filter relay Functions
void filter_relay_init() {
    gpio_reset_pin(FILTER_RELAY_GPIO);
    gpio_reset_pin(BULB3_GPIO_PIN);
    gpio_set_direction(FILTER_RELAY_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(BULB3_GPIO_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(FILTER_RELAY_GPIO, 1); // Initially OFF (active-low)
 
    
}

void filter_on() {
    gpio_set_level(FILTER_RELAY_GPIO, 0); // Active-low
    gpio_set_level(BULB3_GPIO_PIN, 1);  
    ESP_LOGI(TAG, "Water Filter turned ON");
}

void filter_off() {
    gpio_set_level(FILTER_RELAY_GPIO, 1);
    gpio_set_level(BULB3_GPIO_PIN, 0);  
    ESP_LOGI(TAG, "Water Filter turned OFF");
}


// Main app
void app_main(void) {

// Configure GPIO pi for the bulb
gpio_set_direction(BULB1_GPIO_PIN, GPIO_MODE_OUTPUT);


//Initialize NVS
esp_err_t ret = nvs_flash_init();
if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
  ESP_ERROR_CHECK(nvs_flash_erase());
  ret = nvs_flash_init();
}
ESP_ERROR_CHECK(ret);

ESP_LOGI(TAG1, "ESP_WIFI_MODE_STA");
wifi_init_sta();


Set_SystemTime_SNTP();

    gpio_reset_pin(DS18B20_GPIO);
    gpio_set_pull_mode(DS18B20_GPIO, GPIO_PULLUP_ONLY);
    relay_init();
    turbidity_sensor_init(); 
    
 // Initialize turbidity sensor

 //Filter relay function
 filter_relay_init();


 
 
    bool heater_on = false;

    if (!ds18b20_reset()) {
        ESP_LOGE(TAG, "Sensor not detected!");
        return;
    }

    static bool filter_on_flag = false;

    while (1) {
    //Time show
        Get_current_date_time(Current_Date_Time);
		printf("current date and time is = %s\n",Current_Date_Time); 

    // Control the bulb based on the time
    control_bulb_based_on_time();

    vTaskDelay(1000 / portTICK_PERIOD_MS);

        //Temp sensor
        ds18b20_start_conversion();
        vTaskDelay(pdMS_TO_TICKS(750)); // Wait for conversion

        float temp = ds18b20_read_temperature();
        ESP_LOGI(TAG, "Water Temperature: %.2f °C", temp);

        // Control heater using thresholds with hysteresis
        if (temp < TEMP_THRESHOLD_LOW && !heater_on) {
            relay_on();
            heater_on = true;
        } else if (temp > TEMP_THRESHOLD_HIGH && heater_on) {
            relay_off();
            heater_on = false;
        }

        // Read turbidity sensor value
        int turbidity = read_turbidity();
        ESP_LOGI(TAG, "Turbidity Sensor Value: %d", turbidity);

        if (turbidity < TURBIDITY_THRESHOLD_HIGH && !filter_on_flag) {
            filter_on();
            filter_on_flag = true;
        } else if (turbidity > TURBIDITY_THRESHOLD_LOW && filter_on_flag) {
            filter_off();
            filter_on_flag = false;
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}