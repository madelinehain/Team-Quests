/*
  Maxwell Bakalos, Emre Karabay, Miguel Ianus-Valdiva
  EC444 Smart & Connected Systems
  Quest 1 - Solar Tracker
  2/12/2023
*/

#include <stdio.h>
#include <string.h>         // for strings
#include <driver/gpio.h>
#include "driver/adc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc_cal.h"
#include "esp_log.h"
#include "driver/mcpwm.h"
#include <math.h> // for round()
//#include <stdlib.h>
#include "driver/uart.h"
#include "esp_vfs_dev.h"    // This is associated with VFS -- virtual file system interface and abstraction -- see the docs
#include <ctype.h>          // for isdigit()
#include "driver/i2c.h"     // for Alphanumeric Display

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

// Global Variables
struct mapper
{
   int angle;
   int milivolt;
};

struct mapper arr[15];


// User Input
#define ACTIVE 1

// 14-Segment Display
#define SLAVE_ADDR                         0x70 // alphanumeric address
#define OSC                                0x21 // oscillator cmd
#define HT16K33_BLINK_DISPLAYON            0x01 // Display on cmd
#define HT16K33_BLINK_OFF                  0    // Blink off cmd
#define HT16K33_BLINK_CMD                  0x80 // Blink cmd
#define HT16K33_CMD_BRIGHTNESS             0xE0 // Brightness cmd

// Master I2C
#define I2C_EXAMPLE_MASTER_SCL_IO          22   // gpio number for i2c clk
#define I2C_EXAMPLE_MASTER_SDA_IO          23   // gpio number for i2c data
#define I2C_EXAMPLE_MASTER_NUM             I2C_NUM_0  // i2c port
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_FREQ_HZ         100000     // i2c master clock freq
#define WRITE_BIT                          I2C_MASTER_WRITE // i2c master write
#define READ_BIT                           I2C_MASTER_READ  // i2c master read
#define ACK_CHECK_EN                       true // i2c master will check ack
#define ACK_CHECK_DIS                      false// i2c master will not check ack
#define ACK_VAL                            0x00 // i2c ack value
#define NACK_VAL                           0xFF // i2c nack value

// Voltage Reader
#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling

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


////////////////////////////////////////////////////////////////////////////////
// ALPHANUMERIC DISPLAY  & I2C FUNCTIONS ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

// Function to initiate i2c -- note the MSB declaration!
static void i2c_example_master_init(){
  // Debug
  printf("\n>> i2c Config\n");
  int err;

  // Port configuration
  int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;

  /// Define I2C configurations
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;                              // Master mode
  conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;              // Default SDA pin
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
  conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;              // Default SCL pin
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
  conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;       // CLK frequency
  conf.clk_flags = 0;                                       // <-- UNCOMMENT IF YOU GET ERRORS (see readme.md)
  err = i2c_param_config(i2c_master_port, &conf);           // Configure
  if (err == ESP_OK) {printf("- parameters: ok\n");}

  // Install I2C driver
  err = i2c_driver_install(i2c_master_port, conf.mode,
                      I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                      I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
  // i2c_set_data_mode(i2c_master_port,I2C_DATA_MODE_LSB_FIRST,I2C_DATA_MODE_LSB_FIRST);
  if (err == ESP_OK) {printf("- initialized: yes\n\n");}

  // Dat in MSB mode
  i2c_set_data_mode(i2c_master_port, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);
}


// Utility  Functions //////////////////////////////////////////////////////////

// Utility function to test for I2C device address -- not used in deploy
int testConnection(uint8_t devAddr, int32_t timeout) {
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  int err = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  return err;
}

// Utility function to scan for i2c device
static void i2c_scanner() {
  int32_t scanTimeout = 1000;
  printf("\n>> I2C scanning ..."  "\n");
  uint8_t count = 0;
  for (uint8_t i = 1; i < 127; i++) {
      // printf("0x%X%s",i,"\n");
      if (testConnection(i, scanTimeout) == ESP_OK) {
          printf( "- Device found at address: 0x%X%s", i, "\n");
          count++;
      }
  }
  if (count == 0)
      printf("- No I2C devices found!" "\n");
  printf("\n");
}


////////////////////////////////////////////////////////////////////////////////

// Alphanumeric Functions //////////////////////////////////////////////////////

// Turn on oscillator for alpha display
int alpha_oscillator() {
  int ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, OSC, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  vTaskDelay(200 / portTICK_PERIOD_MS);
  return ret;
}

// Set blink rate to off
int no_blink() {
  int ret;
  i2c_cmd_handle_t cmd2 = i2c_cmd_link_create();
  i2c_master_start(cmd2);
  i2c_master_write_byte(cmd2, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd2, HT16K33_BLINK_CMD | HT16K33_BLINK_DISPLAYON | (HT16K33_BLINK_OFF << 1), ACK_CHECK_EN);
  i2c_master_stop(cmd2);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd2, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd2);
  vTaskDelay(200 / portTICK_PERIOD_MS);
  return ret;
}

// Set Brightness
int set_brightness_max(uint8_t val) {
  int ret;
  i2c_cmd_handle_t cmd3 = i2c_cmd_link_create();
  i2c_master_start(cmd3);
  i2c_master_write_byte(cmd3, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd3, HT16K33_CMD_BRIGHTNESS | val, ACK_CHECK_EN);
  i2c_master_stop(cmd3);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd3, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd3);
  vTaskDelay(200 / portTICK_PERIOD_MS);
  return ret;
}

////////////////////////////////////////////////////////////////////////////////

// Set-Up the Alphanumeric Display
static void setup_alpha_display() {
  i2c_example_master_init();  // Initial Set-Up
  i2c_scanner();              // Scan to find I2C Device
  // Debug
  int ret;
  printf(">> Test Alphanumeric Display: \n");

  // Set up routines
  // Turn on alpha oscillator
  ret = alpha_oscillator();
  if(ret == ESP_OK) {printf("- oscillator: ok \n");}
  // Set display blink off
  ret = no_blink();
  if(ret == ESP_OK) {printf("- blink: off \n");}
  ret = set_brightness_max(0xF);
  if(ret == ESP_OK) {printf("- brightness: max \n");}
}

// Alphanumeric Display Table (index = integer value of character)
static const uint16_t alphafonttable[] = {

  0b0000000000000001, 0b0000000000000010, 0b0000000000000100,
  0b0000000000001000, 0b0000000000010000, 0b0000000000100000,
  0b0000000001000000, 0b0000000010000000, 0b0000000100000000,
  0b0000001000000000, 0b0000010000000000, 0b0000100000000000,
  0b0001000000000000, 0b0010000000000000, 0b0100000000000000,
  0b1000000000000000, 0b0000000000000000, 0b0000000000000000,
  0b0000000000000000, 0b0000000000000000, 0b0000000000000000,
  0b0000000000000000, 0b0000000000000000, 0b0000000000000000,
  0b0001001011001001, 0b0001010111000000, 0b0001001011111001,
  0b0000000011100011, 0b0000010100110000, 0b0001001011001000,
  0b0011101000000000, 0b0001011100000000,
  0b0000000000000000, //
  0b0000000000000110, // !
  0b0000001000100000, // "
  0b0001001011001110, // #
  0b0001001011101101, // $
  0b0000110000100100, // %
  0b0010001101011101, // &
  0b0000010000000000, // '
  0b0010010000000000, // (
  0b0000100100000000, // )
  0b0011111111000000, // *
  0b0001001011000000, // +
  0b0000100000000000, // ,
  0b0000000011000000, // -
  0b0100000000000000, // .
  0b0000110000000000, // /
  0b0000110000111111, // 0
  0b0000000000000110, // 1
  0b0000000011011011, // 2
  0b0000000010001111, // 3
  0b0000000011100110, // 4
  0b0010000001101001, // 5
  0b0000000011111101, // 6
  0b0000000000000111, // 7
  0b0000000011111111, // 8
  0b0000000011101111, // 9
  0b0001001000000000, // :
  0b0000101000000000, // ;
  0b0010010000000000, // <
  0b0000000011001000, // =
  0b0000100100000000, // >
  0b0001000010000011, // ?
  0b0000001010111011, // @
  0b0000000011110111, // A
  0b0001001010001111, // B
  0b0000000000111001, // C
  0b0001001000001111, // D
  0b0000000011111001, // E
  0b0000000001110001, // F
  0b0000000010111101, // G
  0b0000000011110110, // H
  0b0001001000001001, // I
  0b0000000000011110, // J
  0b0010010001110000, // K
  0b0000000000111000, // L
  0b0000010100110110, // M
  0b0010000100110110, // N
  0b0000000000111111, // O
  0b0000000011110011, // P
  0b0010000000111111, // Q
  0b0010000011110011, // R
  0b0000000011101101, // S
  0b0001001000000001, // T
  0b0000000000111110, // U
  0b0000110000110000, // V
  0b0010100000110110, // W
  0b0010110100000000, // X
  0b0001010100000000, // Y
  0b0000110000001001, // Z
  0b0000000000111001, // [
  0b0010000100000000, //
  0b0000000000001111, // ]
  0b0000110000000011, // ^
  0b0000000000001000, // _
  0b0000000100000000, // `
  0b0001000001011000, // a
  0b0010000001111000, // b
  0b0000000011011000, // c
  0b0000100010001110, // d
  0b0000100001011000, // e
  0b0000000001110001, // f
  0b0000010010001110, // g
  0b0001000001110000, // h
  0b0001000000000000, // i
  0b0000000000001110, // j
  0b0011011000000000, // k
  0b0000000000110000, // l
  0b0001000011010100, // m
  0b0001000001010000, // n
  0b0000000011011100, // o
  0b0000000101110000, // p
  0b0000010010000110, // q
  0b0000000001010000, // r
  0b0010000010001000, // s
  0b0000000001111000, // t
  0b0000000000011100, // u
  0b0010000000000100, // v
  0b0010100000010100, // w
  0b0010100011000000, // x
  0b0010000000001100, // y
  0b0000100001001000, // z
  0b0000100101001001, // {
  0b0001001000000000, // |
  0b0010010010001001, // }
  0b0000010100100000, // ~
  0b0011111111111111,
};

static void write_alpha_display(char bs[])
{
    // Variable Definitions
    int ret;
    int i_bs_0 = bs[0]; // buffer string character 1 (int)
    int i_bs_1 = bs[1]; // buffer string character 2 (int)
    int i_bs_2 = bs[2]; // buffer string character 3 (int)
    int i_bs_3 = bs[3]; // buffer string character 4 (int)
    uint16_t displaybuffer[8];    // display input variable

    // Convert Characters to Alphanumeric Display Binary Values
    displaybuffer[0] = alphafonttable[i_bs_0];
    displaybuffer[1] = alphafonttable[i_bs_1];
    displaybuffer[2] = alphafonttable[i_bs_2];
    displaybuffer[3] = alphafonttable[i_bs_3];

    // Send commands characters to display over I2C
    i2c_cmd_handle_t cmd4 = i2c_cmd_link_create();
    i2c_master_start(cmd4);
    i2c_master_write_byte(cmd4, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd4, (uint8_t)0x00, ACK_CHECK_EN);
    for (uint8_t i=0; i<8; i++) {
      i2c_master_write_byte(cmd4, displaybuffer[i] & 0xFF, ACK_CHECK_EN);
      i2c_master_write_byte(cmd4, displaybuffer[i] >> 8, ACK_CHECK_EN);
    }
    i2c_master_stop(cmd4);
    ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd4, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd4);

    /*
    if(ret == ESP_OK) {
        printf("\nWrote: %s \n\n", bs);
    }
    */
}


////////////////////////////////////////////////////////////////////////////////
// SERVO FUNCTIONS ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

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

////////////////////////////////////////////////////////////////////////////////
// SOLAR TRACKER FUNCTIONS ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

int read_light(int)
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


// Sample Window Shifting Function
void window_shifter(struct mapper *ar_struct, int window_length) 
{
    unsigned short i;   // preallocate increment index
    int ar1[3];
    int ar2[3];
    // For each sample . . .
    for (i = 0; i < (window_length-1); i++) {
        // Copy into array
        ar1[i] = ar_struct[i].angle;
        ar2[i] = ar_struct[i].milivolt;
    }
    // For each sample . . .
    for (i = 0; i < (window_length-1); i++) {
        // Shift the sample into the next index
        ar_struct[i+1].angle = ar1[i];
        ar_struct[i+1].milivolt = ar2[i];
    }
}

// 
static void wide_servo_sweep_task(void)
{
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


////////////////////////////////////////////////////////////////////////////////
// TASK & MAIN FUNCTIONS ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

void init() 
{
    setup_servo();
}

// Main Function
void app_main(void) 
{
    
}
