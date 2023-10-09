#include <stdio.h>
// #include <stdlib.h>
#include <string.h> // for strings
#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"

// Voltage Reader
#define DEFAULT_VREF 1100 // Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES 64  // Multisampling

// Temperature Sensor (Thermistor)
#define R_constant 10000 // Resistor value in Ohms (for voltage divider)
#define V_cc 3300        // Positive voltage going into voltage divider in milivolts
#define table_len 29     // Length of the Resistance & temp Tables
#define table_len_IR 15  // # of values in the reference voltage -> distance table (from graph)
#define led1 26          // A0

// GLOBAL VARIABLES ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

// Ultrasonic Range Sensor
// float volt2dist_ratio = 3 * ( 1.00 / (V_cc * (1000.00 / 5120.00)) );    // ratio to convert voltage (milivolts) from the ultrasonic range sensor into distance in meters
float volt2dist_ratio = 1.00000 / 644.53125; // ratio to convert voltage (milivolts) from the ultrasonic range sensor into distance in meters
int time = 0;

// Structure to hold the ADC reading & Voltage when the pin is sampled
struct adc_volt
{
    uint32_t adc;
    uint32_t volt;
};

// Temperature Sensor (Thermistor)
float R_thermistor;

// Voltage Reader Variables
static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t therm_channel = ADC_CHANNEL_6; // GPIO34 if ADC1, GPIO14 if ADC2
static const adc_channel_t ultra_channel = ADC_CHANNEL_5;
static const adc_channel_t ir_channel = ADC_CHANNEL_4;
static const adc_channel_t light_channel = ADC_CHANNEL_3;

static const adc_atten_t atten = ADC_ATTEN_DB_11; // changed from 0 -> 11
static const adc_unit_t unit = ADC_UNIT_1;

///////////////////////////////////////////////////////////////////////////////////////
// VOLTAGE READER FUNCTIONS ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

static void check_efuse(void)
{
    // Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK)
    {
        // printf("eFuse Two Point: Supported\n");
    }
    else
    {
        // printf("eFuse Two Point: NOT supported\n");
    }

    // Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK)
    {
        // printf("eFuse Vref: Supported\n");
    }
    else
    {
        // printf("eFuse Vref: NOT supported\n");
    }
}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP)
    {
        // printf("Characterized using Two Point Value\n");
    }
    else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF)
    {
        // printf("Characterized using eFuse Vref\n");
    }
    else
    {
        // printf("Characterized using Default Vref\n");
    }
}

// Setup the ADC Voltage Reader
void voltReadSetup(adc_channel_t channel)
{
    // Check if Two Point or Vref are burned into eFuse
    check_efuse();

    // Configure ADC
    if (unit == ADC_UNIT_1)
    {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(channel, atten);
    }
    else
    {
        adc2_config_channel_atten((adc2_channel_t)channel, atten);
    }

    // Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);
}

// Function to Sample the ADC and get the Voltage
struct adc_volt voltage_reader(adc_channel_t channel)
{
    // SAMPLE THE ANALOG-TO-DIGITAL CONVERTER ~ ~ ~
    uint32_t adc_reading = 0;
    // Multisampling
    for (int i = 0; i < NO_OF_SAMPLES; i++)
    {
        if (unit == ADC_UNIT_1)
        {
            adc_reading += adc1_get_raw((adc1_channel_t)channel);
        }
        else
        {
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
    -30, // 0
    -25, // 1
    -20, // 2
    -15, // 3
    -10, // 4
    -5,  // 5
    0,   // 6
    5,   // 7
    10,  // 8
    15,  // 9
    20,  // 10
    25,  // 11
    30,  // 12
    35,  // 13
    40,  // 14
    45,  // 15
    50,  // 16
    55,  // 17
    60,  // 18
    65,  // 19
    70,  // 20
    75,  // 21
    80,  // 22
    85,  // 23
    90,  // 24
    95,  // 25
    100, // 26
    105, // 27
    110  // 28
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

///////////////////////////////////////////////////////////////////////////////////////
// INRARED RANGEFINDER TABLES & FUNCTIONS ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

// Tables are from the GP2Y0A21YK IR Rangefinder Datasheet (from graph) ~ ~ ~
// Table of Distances in Centimeters (cm)
float table_dist[] = {
    0,   // 0
    5,   // 1
    6.1, // 2
    7,   // 3
    8,   // 4
    10,  // 5
    15,  // 6
    20,  // 7
    25,  // 8
    30,  // 9
    40,  // 10
    50,  // 11
    60,  // 12
    70,  // 13
    80   // 14
};

// Table of Voltages in Mili-Volts (mV)
float table_volt[] = {
    0,    // 0    absolute MINIMUM voltage
    3090, // 1    MAXIMUM voltage
    3150, // 2
    2970, // 3
    2740, // 4
    2310, // 5
    1650, // 6
    1310, // 7
    1080, // 8
    930,  // 9
    740,  // 10
    620,  // 11
    520,  // 12
    450,  // 13
    410   // 14   practical MINIMUM voltage
};

// Function to Convert the voltage divider voltage into the resistance of the R_1 resistor
float volt2resistance(float V_div, float V_plus, float R_2)
{
    // V_div    =   voltage in milivolts
    // V_plus   =   voltage in milivolts
    // R_2      =   resistance in Ohms
    float R_1 = R_2 * ((V_plus / V_div) - 1);
    return R_1;
}

// Range Mapping Function (PW <-> Angle)
float range_map(float num_old, float max_old, float min_old, float max_new, float min_new)
{
    float num_new = (((num_old - min_old) / (max_old - min_old)) * (max_new - min_new) + min_new);
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
    if ((R_therm < table_R[0]) && (R_therm > table_R[table_len - 1]))
    {

        // Find the 2 Closest Thermistor Resistances in the Table
        int i; // loop increment index
        // For each 2 adjacent values in the resistance table . . .
        for (i = 0; i < (table_len - 1); i++)
        {
            // If the thermistor value is between the upper & lower values
            if ((R_therm <= table_R[i]) && (R_therm > table_R[i + 1]))
            {
                // Set the upper & lower temperature table indeces
                i_low = i + 1;
                i_up = i;
            }
        }

        // Set the Temperature to the interpolation between the upper & lower
        // temperature values corrsponding to the resistance values (in the table)
        temp = range_map(R_therm, table_R[i_up], table_R[i_low], table_T[i_up], table_T[i_low]);
    }

    // Else If the Thermistor Resistance is outside the range (extremely High or Low)
    else
    {
        // Set it to the extreme (High or Low) temperature:
        // If very hot . . .
        if (R_therm <= table_R[table_len - 1])
        {
            temp = table_T[table_len - 1];
            // printf("\n TOO HOT");
        }
        // If very cold . . .
        else if (R_therm >= table_R[0])
        {
            temp = table_T[0];
            // printf("\n TOO COLD");
        }
    }

    // Return the Found Temperature
    return temp;
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
    if ((volt < table_volt[1]) && (volt > table_volt[table_len_IR - 1]))
    {

        // Find the 2 Closest Voltages in the Table
        int i; // loop increment index
        // For each 2 adjacent values in the voltage table . . .
        for (i = 1; i < (table_len_IR - 1); i++)
        {
            // If the voltage value is between the upper & lower values
            if ((volt <= table_volt[i]) && (volt > table_volt[i + 1]))
            {
                // Set the upper & lower distance table indeces
                i_low = i + 1;
                i_up = i;
            }
        }

        // Set the distance to the interpolation between the upper & lower
        // distance values corrsponding to the voltage values (in the table)
        dist = range_map(volt, table_volt[i_up], table_volt[i_low], table_dist[i_up], table_dist[i_low]);
    }

    // Else If the Voltage is outside the range (extremely High or Low)
    else
    {
        // Set it to the extreme (High or Low) distance:
        // If very far . . .
        if (volt <= table_volt[table_len_IR - 1])
        {
            dist = table_dist[table_len_IR - 1];
            // printf("\n TOO FAR");
        }
        // If very close . . .
        else if (volt >= table_volt[1])
        {
            dist = table_dist[0];
            // printf("\n TOO CLOSE");
        }
    }

    // Return the Found distance
    return dist;
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
    esp_rom_gpio_pad_select_gpio(led1);
    gpio_set_direction(led1, GPIO_MODE_OUTPUT);

    // Print the Code's Title
    // printf("\n\n~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~\n|| ALL SENSORS ||\n~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~-~\n\n");

    // Setup the ADC Pin Voltage Reader
    voltReadSetup(therm_channel);
    voltReadSetup(ultra_channel);
    voltReadSetup(ir_channel);
    voltReadSetup(light_channel);

    // Continuously Sample the ADC & Calculate the Temperature
    while (1)
    {
        // Sample Voltage
        struct adc_volt therm_sample = voltage_reader(therm_channel);
        struct adc_volt ultra_sample = voltage_reader(ultra_channel);
        struct adc_volt ir_sample = voltage_reader(ir_channel);
        struct adc_volt light_sample = voltage_reader(light_channel);

        ///////// THERMISTOR
        // Calculate Thermistor Resistance
        R_thermistor = volt2resistance(therm_sample.volt, V_cc, R_constant) / 1000;

        // Calculate Temperature
        float temperature = resist2temp(R_thermistor);

        // Print the Values
        // printf("\nRaw: %ld \tVoltage: %ld mV \tThermistor Resistance: %f KOhms \t\tTemperature (Approx.): %f °C", therm_sample.adc, therm_sample.volt, R_thermistor, temperature);

        ///////// ULTRASONIC
        // Calculate Distance
        float ultra_distance = (volt2dist(ultra_sample.volt));

        // printf("\nRaw: %ld \tVoltage: %ld mV \t\tDistance: %f meters", ultra_sample.adc, ultra_sample.volt, ultra_distance);

        //////// IR RANGEFINDER
        float ir_distance = (volt2dist_IR(ir_sample.volt)) / 100;
        // printf("\nRaw: %ld \tVoltage: %ld mV \t\tDistance: %f m", ir_sample.adc, ir_sample.volt, ir_distance);

        vTaskDelay(pdMS_TO_TICKS(2000)); // delay

        //////// LIGHT SENSOR
        // printf("\nRaw: %ld \tVoltage: %ld mV \t", light_sample.adc, light_sample.volt);

        printf("\n%d, %f, %f, %f, %ld", time, temperature, ultra_distance, ir_distance, light_sample.volt);
        // printf("\n%d, %f", time, temperature);
        // printf("\n%d, %f, %f", time, temperature, ultra_distance);

        time++;

        if (!((ultra_distance < 0.5) && (ir_distance < 0.5)))
        {
            gpio_set_level(led1, 1);
        }
        else
        {
            gpio_set_level(led1, 0);
        }
    }
}
