/*
  Maxwell Bakalos
  EC444 Smart & Connected Systems
  Quest 2 - Skill 14
  Ultrasonic Range Sensor
  2/23/2023
*/

#include <stdio.h>
//#include <stdlib.h>
#include <string.h>         // for strings
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"


// Voltage Reader
#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling

// Ultrasonic Range Sensor
#define V_cc        3300.00  // Positive voltage going into voltage divider in milivolts


// GLOBAL VARIABLES ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

// Ultrasonic Range Sensor
// float volt2dist_ratio = 3 * ( 1.00 / (V_cc * (1000.00 / 5120.00)) );    // ratio to convert voltage (milivolts) from the ultrasonic range sensor into distance in meters
float volt2dist_ratio = 1.00000 / 644.53125;    // ratio to convert voltage (milivolts) from the ultrasonic range sensor into distance in meters

// Structure to hold the ADC reading & Voltage when the pin is sampled
struct adc_volt {
    uint32_t adc;
    uint32_t volt;
};

// Voltage Reader Variables
static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_6;     //GPIO34 if ADC1, GPIO14 if ADC2
static const adc_atten_t atten = ADC_ATTEN_DB_11;   // changed from 0 -> 11
static const adc_unit_t unit = ADC_UNIT_1;


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
// ULTRASONIC RANGE SENSOR FUNCTIONS ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

float volt2dist(float milivolt) 
{
    float meters = volt2dist_ratio * milivolt;
    return meters;
}

///////////////////////////////////////////////////////////////////////////////////////
// MAIN MODULE ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

void app_main(void)
{
    // Print the Code's Title
    printf("\n\n~-~-~-~-~-~-~-~-~-~-~-~-~-~-\n|| UTRASONIC RANGE SENSOR ||\n~-~-~-~-~-~-~-~-~-~-~-~-~-~-\n\n");
    printf("\nvolt2dist_ratio = %f", volt2dist_ratio);

    // Setup the ADC Pin Voltage Reader
    voltReadSetup();

    // Continuously Sample the ADC & Calculate the Temperature
    while (1) 
    {
        // Sample Voltage
        struct adc_volt sample = voltage_reader();

        // Calculate Distance
        float distance = volt2dist(sample.volt);

        // Print the Values
        printf("\nRaw: %d \tVoltage: %d mV \t\tDistance: %f meters", sample.adc, sample.volt, distance);

        vTaskDelay(pdMS_TO_TICKS(2000)); // delay
    }
}
