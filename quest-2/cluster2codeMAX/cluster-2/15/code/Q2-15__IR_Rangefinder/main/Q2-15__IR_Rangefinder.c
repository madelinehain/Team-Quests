/*
  Maxwell Bakalos
  EC444 Smart & Connected Systems
  Quest 2 - Skill 15
  IR Rangefinder
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
#define DEFAULT_VREF    1100        // Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          // Multisampling

// Ultrasonic Range Sensor
#define V_cc        3300.00  // Positive voltage going into voltage divider in milivolts
#define table_len_IR   15    // # of values in the reference voltage -> distance table (from graph)


// GLOBAL VARIABLES ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

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
// INRARED RANGEFINDER TABLES & FUNCTIONS ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

// Tables are from the GP2Y0A21YK IR Rangefinder Datasheet (from graph) ~ ~ ~
// Table of Distances in Centimeters (cm)
float table_dist[] = {
    0,      // 0
    5,      // 1
    6.1,    // 2
    7,      // 3
    8,      // 4
    10,     // 5
    15,     // 6
    20,     // 7
    25,     // 8
    30,     // 9
    40,     // 10
    50,     // 11
    60,     // 12
    70,     // 13
    80      // 14
};

// Table of Voltages in Mili-Volts (mV)
float table_volt[] = {
    0,      // 0    absolute MINIMUM voltage
    3090,   // 1    MAXIMUM voltage
    3150,   // 2
    2970,   // 3
    2740,   // 4
    2310,   // 5
    1650,   // 6
    1310,   // 7
    1080,   // 8
    930,    // 9
    740,    // 10
    620,    // 11
    520,    // 12
    450,    // 13
    410     // 14   practical MINIMUM voltage
};

// Range Mapping Function
float range_map(float num_old, float max_old, float min_old, float max_new, float min_new) 
{
    float num_new =  (  ( (num_old - min_old) / (max_old - min_old) ) * (max_new - min_new)  +  min_new  );
    return num_new;
}


// Function to Convert the Measured Voltage into Distance in cm by Interpolating Between the Table Values
float volt2dist_IR(float volt) 
{
    // Preallocate
    int i_up;       // index of upper distance bound (in table)
    int i_low;      // index of lower distance bound (in table)
    float dist = 0; // Measured distance (calculated by linearly interpolating the table values)


    // CALCULATE DISTANCE ~ ~ ~ ~ ~ ~ ~

    // If the Voltage is in the Given Range from the Table
    if ((volt < table_volt[1]) && (volt > table_volt[table_len_IR-1])) {

        // Find the 2 Closest Voltages in the Table
        int i;  // loop increment index
        // For each 2 adjacent values in the voltage table . . .
        for (i = 1; i < (table_len_IR-1); i++) {
            // If the voltage value is between the upper & lower values
            if ((volt <= table_volt[i]) && (volt > table_volt[i+1])) {
                // Set the upper & lower distance table indeces
                i_low  = i + 1;
                i_up = i;
            }
        }

        // Set the distance to the interpolation between the upper & lower 
        // distance values corrsponding to the voltage values (in the table)
            dist = range_map(volt, table_volt[i_up], table_volt[i_low], table_dist[i_up], table_dist[i_low]);
    }

    // Else If the Voltage is outside the range (extremely High or Low)
    else {
        // Set it to the extreme (High or Low) distance:
        // If very far . . .
        if (volt <= table_volt[table_len_IR-1]) {
            dist = table_dist[table_len_IR-1];
            printf("\n TOO FAR");
        }
        // If very close . . .
        else if (volt >= table_volt[1]) {
            dist = table_dist[0];
            printf("\n TOO CLOSE");
        }
    }

    // Return the Found distance
    return dist;
    
}


///////////////////////////////////////////////////////////////////////////////////////
// MAIN MODULE ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

void app_main(void)
{
    // Print the Code's Title
    printf("\n\n~-~-~-~-~-~-~-~-~-~-~-~-~-\n|| INFRARED RANGEFINDER ||\n~-~-~-~-~-~-~-~-~-~-~-~-~-\n\n");

    // Setup the ADC Pin Voltage Reader
    voltReadSetup();

    // Continuously Sample the ADC & Calculate the Distance
    while (1) 
    {
        // Sample Voltage
        struct adc_volt sample = voltage_reader();

        // Calculate Distance
        float distance = volt2dist_IR(sample.volt);

        // Print the Values
        printf("\nRaw: %d \tVoltage: %d mV \t\tDistance: %f cm", sample.adc, sample.volt, distance);

        vTaskDelay(pdMS_TO_TICKS(2000)); // delay
    }
}