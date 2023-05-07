/*
  Maxwell Bakalos, Miguel Ianus-Valdiva, Emre Karabay, Madeline Hain
  EC444 Smart & Connected Systems
  Quest 3 - Cat Tracker
  UDP Client for Thermistor Readings
  3/31/2023
*/

// Include for UDP Client
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

// Include for Thermistor
#include <stdio.h>
#include <driver/gpio.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"


// DEFINE: UDP Client //////////////////////////////////// 
#ifdef CONFIG_EXAMPLE_SOCKET_IP_INPUT_STDIN
#include "addr_from_stdin.h"
#endif

#if defined(CONFIG_EXAMPLE_IPV4)
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV4_ADDR
#elif defined(CONFIG_EXAMPLE_IPV6)
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV6_ADDR
#else
#define HOST_IP_ADDR ""
#endif

#define PORT CONFIG_EXAMPLE_PORT


// DEFINE: Thermistor //////////////////////////////////// 
// Voltage Reader
#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling

// Temperature Sensor (Thermistor)
#define R_constant  10000   // Resistor value in Ohms (for voltage divider)
#define V_cc        3300     // Positive voltage going into voltage divider in milivolts
#define table_len   29      // Length of the Resistance & temp Tables

// DEFINE: LED Button //////////////////////////////////// 
#define LED_PIN 19

// GLOBAL VARIABLES: UDP Client //////////////////////////////////// 
static const char *TAG = "Thermistor_Client";
// static const char *payload = "Message from ESP32 ";
char payload[20];

// GLOBAL VARIABLES: Thermistor //////////////////////////////////// 

// Structure to hold the ADC reading & Voltage when the pin is sampled
struct adc_volt {
    uint32_t adc;
    uint32_t volt;
};

// Temperature Sensor (Thermistor)
float R_thermistor;

// Voltage Reader Variables
static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_6;     //GPIO34 if ADC1, GPIO14 if ADC2
static const adc_atten_t atten = ADC_ATTEN_DB_11;   // changed from 0 -> 11
static const adc_unit_t unit = ADC_UNIT_1;


////////////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCTIONS: UDP Client ///////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////

static void udp_client_task(void *pvParameters)
{
    char rx_buffer[128];
    char host_ip[] = HOST_IP_ADDR;
    int addr_family = 0;
    int ip_protocol = 0;
    // short blink_state = 0;

    while (1) {

#if defined(CONFIG_EXAMPLE_IPV4)
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
#elif defined(CONFIG_EXAMPLE_IPV6)
        struct sockaddr_in6 dest_addr = { 0 };
        inet6_aton(HOST_IP_ADDR, &dest_addr.sin6_addr);
        dest_addr.sin6_family = AF_INET6;
        dest_addr.sin6_port = htons(PORT);
        dest_addr.sin6_scope_id = esp_netif_get_netif_impl_index(EXAMPLE_INTERFACE);
        addr_family = AF_INET6;
        ip_protocol = IPPROTO_IPV6;
#elif defined(CONFIG_EXAMPLE_SOCKET_IP_INPUT_STDIN)
        struct sockaddr_storage dest_addr = { 0 };
        ESP_ERROR_CHECK(get_addr_from_stdin(PORT, SOCK_DGRAM, &ip_protocol, &addr_family, &dest_addr));
#endif

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }

        // Set timeout
        struct timeval timeout;
        timeout.tv_sec = 10;
        timeout.tv_usec = 0;
        setsockopt (sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);

        ESP_LOGI(TAG, "Socket created, sending to %s:%d", HOST_IP_ADDR, PORT);

        while (1) {

            int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            if (err < 0) {
                ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                break;
            }
            ESP_LOGI(TAG, "Message sent");

            struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
            socklen_t socklen = sizeof(source_addr);
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                break;
            }
            // Data received
            else {
                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, host_ip);
                ESP_LOGI(TAG, "%s", rx_buffer);
                if (strncmp(rx_buffer, "OK: ", 4) == 0) {
                    ESP_LOGI(TAG, "Received expected message, reconnecting");
                    break;
                }

                // // ADDED CODE //////
                // if (strncmp(rx_buffer, "BUTTON_PRESSED", 10) == 0) {
                //     ESP_LOGI(TAG, "LED BLINKING!!!");
                //     blink_state = !(blink_state);           // flip LED state
                //     gpio_set_level(LED_PIN, blink_state);   // turn on/off LED
                // }
                // else {
                //     ESP_LOGI(TAG, "LED OFF");
                //     gpio_set_level(LED_PIN, 0);  // turn off LED
                // }

            }

            vTaskDelay(2000 / portTICK_PERIOD_MS);
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCTIONS: Thermistor ///////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////
// VOLTAGE READER FUNCTIONS ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

static void check_efuse(void)
{
    //Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }

    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}

// Setup the ADC Voltage Reader
void voltReadSetup() 
{
    // Check if Two Point or Vref are burned into eFuse
    check_efuse();

    // Configure ADC
    if (unit == ADC_UNIT_1) {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(channel, atten);
    } else {
        adc2_config_channel_atten((adc2_channel_t)channel, atten);
    }

    // Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);
}

// Function to Sample the ADC and get the Voltage
struct adc_volt voltage_reader() 
{
    // SAMPLE THE ANALOG-TO-DIGITAL CONVERTER ~ ~ ~
    uint32_t adc_reading = 0;
    // Multisampling
    for (int i = 0; i < NO_OF_SAMPLES; i++) {
        if (unit == ADC_UNIT_1) {
            adc_reading += adc1_get_raw((adc1_channel_t)channel);
        } else {
            int raw;
            adc2_get_raw((adc2_channel_t)channel, ADC_WIDTH_BIT_12, &raw);
            adc_reading += raw;
        }
    }
    adc_reading /= NO_OF_SAMPLES;

    // Convert adc_reading to voltage in mV
    uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);

    // Return the ampled Voltage from the ADC Pin
    struct adc_volt sample_vals = {adc_reading, voltage};
    return sample_vals;
}


///////////////////////////////////////////////////////////////////////////////////////
// THERMISTOR FUNCTIONS & TABLES ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

// Table Value copied from Thermistor Datasheet: "NTC THERMISTOR OF MF52-TYPE SERIES SPECIFICATION"
// Table of Temperatures (Celsius)
int table_T[] = {
    -30,    // 0
    -25,    // 1
    -20,    // 2
    -15,    // 3
    -10,    // 4
    -5,     // 5
    0,      // 6
    5,      // 7
    10,     // 8
    15,     // 9
    20,     // 10
    25,     // 11
    30,     // 12
    35,     // 13
    40,     // 14
    45,     // 15
    50,     // 16
    55,     // 17
    60,     // 18
    65,     // 19 
    70,     // 20
    75,     // 21
    80,     // 22
    85,     // 23
    90,     // 24
    95,     // 25
    100,    // 26
    105,    // 27
    110     // 28
};

// Table of Resistances (kOhms)
float table_R[] = {
    181.70, // 0
    133.30, // 1
    98.88,  // 2
    74.10,  // 3
    56.06,  // 4
    42.80,  // 5
    38.96,  // 6 (was 98.96 but I'm assuming it's a typo)
    25.58,  // 7
    20.00,  // 8
    15.76,  // 9
    12.51,  // 10
    10.00,  // 11
    8.048,  // 12
    6.518,  // 13
    5.312,  // 14
    4.354,  // 15
    3.588,  // 16
    2.974,  // 17
    2.476,  // 18
    2.072,  // 19
    1.743,  // 20
    1.473,  // 21
    1.250,  // 22
    1.065,  // 23
    0.911,  // 24
    0.7824, // 25
    0.6744, // 26
    0.5836, // 27
    0.5066  // 28
};

// Function to Convert the voltage divider voltage into the resistance of the R_1 resistor
float volt2resistance(float V_div, float V_plus, float R_2) 
{
    // V_div    =   voltage in milivolts
    // V_plus   =   voltage in milivolts
    // R_2      =   resistance in Ohms
    float R_1 = R_2 * ((V_plus/V_div) - 1);
    return R_1;
}

// Range Mapping Function (PW <-> Angle)
float range_map(float num_old, float max_old, float min_old, float max_new, float min_new) 
{
    float num_new =  (  ( (num_old - min_old) / (max_old - min_old) ) * (max_new - min_new)  +  min_new  );
    return num_new;
}

// Function to Convert the Thermistor Resistance into Temperature in Celsius by Interpolating Between the Table Values
float resist2temp(float R_therm) 
{
    // Preallocate
    int i_up;       // index of upper temperature bound (in table)
    int i_low;      // index of lower temperature bound (in table)
    float temp = 0; // Measured Temperature (calculated by linearly interpolating the table values)


    // CALCULATE TEMPERATURE ~ ~ ~ ~ ~ ~ ~

    // If the Thermistor Resistance is in the Given Range from the Table
    if ((R_therm < table_R[0]) && (R_therm > table_R[table_len-1])) {

        // Find the 2 Closest Thermistor Resistances in the Table
        int i;  // loop increment index
        // For each 2 adjacent values in the resistance table . . .
        for (i = 0; i < (table_len-1); i++) {
            // If the thermistor value is between the upper & lower values
            if ((R_therm <= table_R[i]) && (R_therm > table_R[i+1])) {
                // Set the upper & lower temperature table indeces
                i_low  = i + 1;
                i_up = i;
            }
        }

        // Set the Temperature to the interpolation between the upper & lower 
        // temperature values corrsponding to the resistance values (in the table)
            temp = range_map(R_therm, table_R[i_up], table_R[i_low], table_T[i_up], table_T[i_low]);
    }

    // Else If the Thermistor Resistance is outside the range (extremely High or Low)
    else {
        // Set it to the extreme (High or Low) temperature:
        // If very hot . . .
        if (R_therm <= table_R[table_len-1]) {
            temp = table_T[table_len-1];
            printf("\n TOO HOT");
        }
        // If very cold . . .
        else if (R_therm >= table_R[0]) {
            temp = table_T[0];
            printf("\n TOO COLD");
        }
    }

    // Return the Found Temperature
    return temp;
    
}

// Get Thermistor Readings & Put in Message for UDP Server
static void thermistor_task() {
    // Initialize the Sample Index
    int sample_index = 0;

    // Continuously Sample the ADC & Calculate the Temperature
    while (1) 
    {
        // Sample Voltage
        struct adc_volt sample = voltage_reader();

        // Calculate Thermistor Resistance
        R_thermistor = volt2resistance(sample.volt, V_cc, R_constant) / 1000;

        // Calculate Temperature
        float temperature = resist2temp(R_thermistor);

        // Print the Values
        // printf("\nRaw: %d \tVoltage: %d mV \tThermistor Resistance: %f KOhms \t\tTemperature (Approx.): %f °C", sample.adc, sample.volt, R_thermistor, temperature);

        sample_index += 1;  // Increment Sample Index

        // sprintf(payload, "%d, %.3f", sample_index, temperature);
        sprintf(payload, "%d, %.3f\n", sample_index, temperature);
        printf("%d, %.3f\n", sample_index, temperature);


        vTaskDelay(pdMS_TO_TICKS(2000)); // delay
    }
}



///////////////////////////////////////////////////////////////////////////////////////
// MAIN MODULE ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    // Setup the ADC Pin Voltage Reader
    voltReadSetup();

    // Initializing LED
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

//    xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 5, NULL);
    xTaskCreate(udp_client_task, "udp_client", 4096, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(thermistor_task, "thermistor", 4096, NULL, configMAX_PRIORITIES-1, NULL);
}
