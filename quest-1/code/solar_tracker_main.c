/*
  Maxwell Bakalos
  EC444 Smart & Connected Systems
  Quest 1 - Skill 09
  Servos
  2/8/2023
*/

#include <stdio.h>
#include "driver/mcpwm.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <math.h> // for round()

#define ACTIVE 1

// CONSTANTS ~_~_~_~_~_~_~_~_~_~_~_~_~_~_~_~_~_~_~_~_~_~_
#define servo_pin   18   // Control pin for the servo
#define max_pw      2500 // Maximum pulse width (miliseconds)
#define min_pw      500  // Minimum pulse width (miliseconds)
#define max_aval_pw 2400 // Maximum available pulse width (miliseconds)
#define min_aval_pw 600  // Minimum available pulse width (miliseconds)
#define max_angle   90   // Maximum angle (degrees)
#define min_angle   -90  // Minimum angle (degrees)
#define start_angle 2    // Initial angle (degrees)
#define delay_ms    200  // Delay between steps (miliseconds)

// Constant Variables for printing [-] [\] [|] [/] [-]
const float low_pw = (0.20 * (max_aval_pw - min_aval_pw)) + min_aval_pw;    // low (short) PW   [-]
const float mid_low_pw = (0.4 * (max_aval_pw - min_aval_pw)) + min_aval_pw; // middle-low PW    [/]
const float mid_high_pw = (0.6 * (max_aval_pw - min_aval_pw)) + min_aval_pw;// middle-high PW   [\]
const float high_pw = (0.80 * (max_aval_pw - min_aval_pw)) + min_aval_pw;   // high (long) PW   [-]

// Range Mapping Function (PW <-> Angle)
int range_map(float num_old, float max_old, float min_old, float max_new, float min_new) {
    int pw =  (  ( (num_old - min_new) / (max_new - min_new) ) * (max_old - min_old)  +  min_old  );
    return pw;
}

// Servo MCPWM Setup Function
void setup_servo(void) 
{
    // 1. Set GPIO 18 as PWM0A, to which servo control pin is connected
    printf("\nSetting GPIO Pin as Control");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, servo_pin);
    
    // 2. initial mcpwm configuration
    printf("\nConfiguring MCPWM");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 50;    //frequency = 50Hz, i.e. for every servo motor time period should be 20ms
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings
}

int read_ligh(int)
{
     uint32_t adc_reading = 36;
    //Multisampling
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
    //Convert adc_reading to voltage in mV
    uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
    return voltage
}

struct mapper
{
   int angle;
   int milivolt;
};

struct mapper arr[15];

void window_shifter(int *array, int window_length) 
{
    unsigned short i;   // preallocate increment index
    // For each sample . . .
    for (i = 0; i < (window_length-1); i++) {
        // Shift the sample into the next index
        array[i+1].angle = array[i].angle;
        array[i+1].milivolt = array[i].milivolt;
    }
}

static void init_servo_task(void)
{
    while (ACTIVE)
    {
        // If the PWM is at a bound of its range, reverse the direction
        if (i_pw >= max_aval_pw){
            step_pw = -step_pw;
            i_pw = max_aval_pw;
        } else if (i_pw <= min_aval_pw) {
            step_pw = -step_pw;
            i_pw = min_aval_pw;
        }

        // Angle-Character Selection for Print
        if ((i_pw <= low_pw) || (i_pw > high_pw)) {
            angle_char = '-';
        } else if ((i_pw <= high_pw) && (i_pw > mid_high_pw)) {
            angle_char = '\\';
        } else if ((i_pw <= mid_high_pw) && (i_pw > mid_low_pw)) {
            angle_char = '|';
        } else if ((i_pw <= mid_low_pw) && (i_pw > low_pw)) {
            angle_char = '/';
        }

        // Find actual angle (not necessarily the full +-90° at the bounds)
        int i_angle = range_map(i_pw, max_aval_pw, min_aval_pw, max_aval_angle, min_aval_angle);
        // Print Next Position
        printf("\n[%c] Setting PW to   %d   or   %d°", angle_char, i_pw, i_angle); // Print whats happeneing
        window_shifter()

        // Update Position
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, i_pw);
        vTaskDelay(delay_ms / portTICK_PERIOD_MS);  // delay

        i_pw += step_pw;  // increment PW by step

    }
}

// Main Function
void app_main(void) 
{
    // ~_~_~_~_~_~_~_~_~_~_~_~_~_~_~_~_~_~_~_~_~_~_~_~_~_~_~_~_~_~_~_~_~_~_~_~_
    // SETUP
    setup_servo();
    // Set Step for the Pulse Width
    int step_pw = 50;
    // Set Inital Pulse Width from start angle definition
    int i_pw = range_map(start_angle, max_aval_pw, min_aval_pw, max_angle, min_angle);
    // Character for Printing
    char angle_char = '|'; 
    printf("\nlow_pw = %f, mid_low_pw = %f, mid_high_pw = %f, high_pw = %f", low_pw, mid_low_pw, mid_high_pw, high_pw);
    
    // Calculate Maximum Available Angle Values
    int max_aval_angle = range_map(max_aval_pw, max_pw, min_pw, max_angle, min_angle);
    int min_aval_angle = range_map(min_aval_pw, max_pw, min_pw, max_angle, min_angle);

    // Go To Start Position
    printf("\nSetting PW to %d (Initial Position)", i_pw); // Print whats happeneing
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, i_pw); // set position
    vTaskDelay(1000 / portTICK_PERIOD_MS);  // delay

    // ~_~_~_~_~_~_~_~_~_~_~_~_~_~_~_~_~_~_~_~_~_~_~_~_~_~_~_~_~_~_~_~_~_~_~_~_
    // LOOP
    // Move the Servo Back and Forth Between it's Maximum and Minimum Angles
    while (ACTIVE)
    {
        // If the PWM is at a bound of its range, reverse the direction
        if (i_pw >= max_aval_pw){
            step_pw = -step_pw;
            i_pw = max_aval_pw;
        } else if (i_pw <= min_aval_pw) {
            step_pw = -step_pw;
            i_pw = min_aval_pw;
        }

        // Angle-Character Selection for Print
        if ((i_pw <= low_pw) || (i_pw > high_pw)) {
            angle_char = '-';
        } else if ((i_pw <= high_pw) && (i_pw > mid_high_pw)) {
            angle_char = '\\';
        } else if ((i_pw <= mid_high_pw) && (i_pw > mid_low_pw)) {
            angle_char = '|';
        } else if ((i_pw <= mid_low_pw) && (i_pw > low_pw)) {
            angle_char = '/';
        }

        // Find actual angle (not necessarily the full +-90° at the bounds)
        int i_angle = range_map(i_pw, max_aval_pw, min_aval_pw, max_aval_angle, min_aval_angle);
        // Print Next Position
        printf("\n[%c] Setting PW to   %d   or   %d°", angle_char, i_pw, i_angle); // Print whats happeneing

        // Update Position
        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, i_pw);
        vTaskDelay(delay_ms / portTICK_PERIOD_MS);  // delay

        i_pw += step_pw;  // increment PW by step

    }

}