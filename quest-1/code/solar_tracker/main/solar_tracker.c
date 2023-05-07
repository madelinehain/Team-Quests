/*
  Maxwell Bakalos, Emre Karabay, Miguel Ianus-Valdiva
  EC444 Smart & Connected Systems
  Quest 1 - Solar Tracker
  2/12/2023
*/

#include <stdio.h>
#include <string.h>         // for strings
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/mcpwm.h"
#include <math.h> // for round()
#include <stdlib.h> // for abs()
#include "esp_vfs_dev.h"    // This is associated with VFS -- virtual file system interface and abstraction -- see the docs
#include <ctype.h>          // for isdigit()
#include "driver/i2c.h"     // for Alphanumeric Display
#include "esp_types.h"
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include <driver/gpio.h>    // GPIO
#include "driver/adc.h"     // ADC
#include "esp_adc_cal.h"    // ADC
#include "esp_log.h"        // ESP log
#include "driver/uart.h"    // UART

#define ACTIVE 1

// CONSTANTS ~_~_~_~_~_~_~_~_~_~_~_~_~_~_~_~_~_~_~_~_~_~_

// Task While Loop
#define ACTIVE      1
#define long_delay  1000
#define while_loop_delay 10

// SERVOS
#define servo_pin_1   18  // Control pin for the servo
#define servo_pin_2   4   // Control pin for the servo
#define max_pw      2500 // Maximum pulse width (miliseconds)
#define min_pw      500  // Minimum pulse width (miliseconds)
#define max_aval_pw 2300 // Maximum available pulse width (miliseconds)
#define min_aval_pw 700  // Minimum available pulse width (miliseconds)
#define max_angle   90   // Maximum angle (degrees)
#define min_angle   -90  // Minimum angle (degrees)
#define start_angle -90  // Initial angle (degrees)
#define delay_ms    100  // Delay between steps (miliseconds)

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

// Timer
#define TIMER_DIVIDER         16    //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // to seconds
#define TIMER_INTERVAL_SEC   (10)    // Sample test interval for the first timer (how quick it counts up)
#define TEST_WITH_RELOAD      1     // Testing will be done with auto reload

// Solar Tracker
#define WINDOW_SIZE     15   // size of the angle 1&2 voltage struct window
#define NUM_OF_SWEEPS   3   // # of sweeps (3)

// Constant Variables ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
// Printing Angle [-] [\] [|] [/] [-]
const float low_pw = (0.20 * (max_aval_pw - min_aval_pw)) + min_aval_pw;    // low (short) PW   [-]
const float mid_low_pw = (0.4 * (max_aval_pw - min_aval_pw)) + min_aval_pw; // middle-low PW    [/]
const float mid_high_pw = (0.6 * (max_aval_pw - min_aval_pw)) + min_aval_pw;// middle-high PW   [\]
const float high_pw = (0.80 * (max_aval_pw - min_aval_pw)) + min_aval_pw;   // high (long) PW   [-]

// Servo
const int servo_pins_AB[2] = {servo_pin_1, servo_pin_2};


// GLOBAL VARIABLES ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

// A simple structure to pass "events" to main task
typedef struct {
    int flag;     // flag for enabling stuff in main code
} timer_event_t;

// Initialize queue handler for timer-based events
xQueueHandle timer_queue;

int search_trigger = 0;   // light search toggle trigger (0 -> OFF, 1 -> ON)

struct mapper
{
   int angle;
   int volt;
};

struct mapper vol_ang[WINDOW_SIZE];




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
    // 1. Set PWM0A, to which servo control pin is connected
    printf("\nSetting GPIO Pin as Control");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, servo_pin_1);
    
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
// TIMER FUNCTIONS ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~


// ISR handler
void IRAM_ATTR timer_group0_isr(void *para) {

    // Prepare basic event data, aka set flag
    timer_event_t evt;
    evt.flag = 1;

    // Clear the interrupt, Timer 0 in group 0
    TIMERG0.int_clr_timers.t0 = 1;

    // After the alarm triggers, we need to re-enable it to trigger it next time
    TIMERG0.hw_timer[TIMER_0].config.alarm_en = TIMER_ALARM_EN;

    // Send the event data back to the main program task
    xQueueSendFromISR(timer_queue, &evt, NULL);
}

// Initialize timer 0 in group 0 for 1 sec alarm interval and auto reload
static void alarm_init() {
    /* Select and initialize basic parameters of the timer */
    timer_config_t config;
    config.divider = TIMER_DIVIDER;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en = TIMER_PAUSE;
    config.alarm_en = TIMER_ALARM_EN;
    config.intr_type = TIMER_INTR_LEVEL;
    config.auto_reload = TEST_WITH_RELOAD;
    timer_init(TIMER_GROUP_0, TIMER_0, &config);

    // Timer's counter will initially start from value below
    timer_set_counter_value(TIMER_GROUP_0, TIMER_0, 0x00000000ULL);

    // Configure the alarm value and the interrupt on alarm
    timer_set_alarm_value(TIMER_GROUP_0, TIMER_0, TIMER_INTERVAL_SEC * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, TIMER_0);
    timer_isr_register(TIMER_GROUP_0, TIMER_0, timer_group0_isr,
        (void *) TIMER_0, ESP_INTR_FLAG_IRAM, NULL);

    // Start timer
    timer_start(TIMER_GROUP_0, TIMER_0);
}


////////////////////////////////////////////////////////////////////////////////
// SOLAR TRACKER FUNCTIONS ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

// Reset the struct fo voltage & angle
void reset_struct() {
    // Set Up Structure for Samples
    int i;
    // Calculate Maximum Available Angle Values
    int min_aval_angle = range_map(min_aval_pw, max_pw, min_pw, max_angle, min_angle);

    for (i = 0; i < WINDOW_SIZE; i++) {
        vol_ang[i].volt = 0;            // set voltages to zero
        vol_ang[i].angle = min_aval_angle;  // set angles to the minimum available angle
    }
}

uint32_t read_voltage(void)
{
    // Sample ADC1

    uint32_t adc_reading = 0;   // preallocate

    // Multisampling (average of multiple samples)
    // For each sample
    for (int i = 0; i < NO_OF_SAMPLES; i++) {
        // Add the reading to the overall sum
        if (unit == ADC_UNIT_1) {
            adc_reading += adc1_get_raw((adc1_channel_t)channel);
        } else {
            int raw;
            adc2_get_raw((adc2_channel_t)channel, ADC_WIDTH_BIT_12, &raw);
            adc_reading += raw;
        }
    }
    // Divide by the # of samples to get the average value
    adc_reading /= NO_OF_SAMPLES;

    // Convert adc_reading to voltage in mV
    uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);

    return voltage;

    // printf("Raw: %d\tVoltage: %dmV\n", adc_reading, voltage);
}


// Sample Window Shifting Function
void window_shifter(struct mapper *ar_struct, int window_length) 
{
    unsigned short i;   // preallocate increment index
    int ar1[window_length];
    int ar2[window_length];
    // For each sample . . .
    for (i = 0; i < (window_length-1); i++) {
        // Copy into array
        ar1[i] = ar_struct[i].angle;
        ar2[i] = ar_struct[i].volt;
    }
    // For each sample . . .
    for (i = 0; i < (window_length-1); i++) {
        // Shift the sample into the next index
        ar_struct[i+1].angle = ar1[i];
        ar_struct[i+1].volt = ar2[i];
    }
}

// Sweeping Servo Task
static void wide_servo_sweep_task()
{
    // Initialize Sweep Index
    int i_sweep = 0;
    // Initialize Servo Pin Index
    int servo_pin_index = 0;
    // Maximum Light Variables
    int max_vol[NUM_OF_SWEEPS];     // Maximum voltage for each sweep
    int max_vol_ang[NUM_OF_SWEEPS]; // Angle of maximum voltage for each sweep
    int max_vol_pw[NUM_OF_SWEEPS];  // Pulse Width of maximum voltage for each sweep

    // Set Step for the Pulse Width
    int step_pw = 100;
    
    // Character for Printing
    char angle_char = '|'; 
    printf("\nlow_pw = %f, mid_low_pw = %f, mid_high_pw = %f, high_pw = %f", low_pw, mid_low_pw, mid_high_pw, high_pw);
    
    // Set Inital Pulse Width from start angle definition
    int start_pw_0 = min_aval_pw;   // -90 degrees
    int start_pw_1 = 1500;          // middle 0 degrees

    // Set Inital Pulse Width
    int i_pw = start_pw_0;

    // Calculate Maximum Available Angle Values
    int max_aval_angle = range_map(max_aval_pw, max_pw, min_pw, max_angle, min_angle);
    int min_aval_angle = range_map(min_aval_pw, max_pw, min_pw, max_angle, min_angle);

    // Print Parameters
    printf("\nParameters: step_pw = %d, angle_char = %c, max_aval_angle = %d, min_aval_angle = %d", step_pw, angle_char, max_aval_angle, min_aval_angle);

    // SERVO 1: Go To Start Position
    printf("\nSERVO #%d: Setting PW to %d (Initial Position)", servo_pin_index, start_pw_0); // Print whats happeneing
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, max_aval_pw);    // set to MAX position
    vTaskDelay(long_delay / portTICK_PERIOD_MS);                                    // delay
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, min_aval_pw);    // set to MIN position
    vTaskDelay(long_delay / portTICK_PERIOD_MS);                                    // delay
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, start_pw_0);     // set to START position
    vTaskDelay(long_delay / portTICK_PERIOD_MS);                                    // delay


    // SERVO 2: Go To Start Position
    servo_pin_index = ! servo_pin_index;    // switch servo
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, servo_pins_AB[servo_pin_index]);
    printf("\nSERVO #%d: Setting PW to %d (Initial Position)\n", servo_pin_index, start_pw_1); // Print whats happeneing
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, max_aval_pw);    // set to MAX position
    vTaskDelay(long_delay / portTICK_PERIOD_MS);                                    // delay
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, min_aval_pw);    // set to MIN position
    vTaskDelay(long_delay / portTICK_PERIOD_MS);                                    // delay
    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, start_pw_1);     // set to START position
    vTaskDelay(long_delay / portTICK_PERIOD_MS);                                    // delay

    servo_pin_index = ! servo_pin_index;    // switch servo back 

    int search_state = 0;

    int move_condition = 0;

    // Task While Loop
    while (ACTIVE)
    {

        // If a Search for Light has been Triggered . . . 
        if (search_trigger == 1) {
            // Turn the Search to the ON State
            search_state = 1;

            // Go to start positions . . .
            // SERVO 1: Go To Start Position
            printf("\nSERVO #%d: Setting PW to %d (Initial Position)", 0, start_pw_0); // Print whats happeneing
            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, start_pw_0);   // set position
            vTaskDelay(300 / portTICK_PERIOD_MS);                                  // delay

            // SERVO 2: Go To Start Position
            printf("\nSERVO #%d: Setting PW to %d (Initial Position)", 1, start_pw_1); // Print whats happeneing
            mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, start_pw_1);   // set position
            vTaskDelay(300 / portTICK_PERIOD_MS);                                  // delay

            // Reset the Light Search Trigger
            search_trigger = 0;
        }



        // If Searching is turned on . . .
        if (search_state == 1) {

            // If the servo hasn't reached the end of its rotational range 
            // AND the current voltage greater than the oldest voltge in the window . . .
            if ((i_pw <= max_aval_pw) && (vol_ang[0].volt >= vol_ang[WINDOW_SIZE-1].volt)) {
                // Turn ON the Search
                if  (vol_ang[0].volt >= vol_ang[WINDOW_SIZE-2].volt) {
                    move_condition = 1;
                    printf("\nMover Condition: ON, i_pw = %d, current = %d, last = %d, 2nd last = %d", i_pw, vol_ang[0].volt, vol_ang[WINDOW_SIZE-1].volt, vol_ang[WINDOW_SIZE-2].volt);
                }
            } else {
                // Turn OFF the search
                move_condition = 0;
                printf("\n\nFOUND MAXIMUM!\n");
            }
            

            // If the search in ON . . .
            if (move_condition == 1)
            {
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
                if (servo_pin_index == 0) {
                    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, i_pw);   // set the position
                } else if (servo_pin_index == 1) {
                    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, i_pw);   // set the position
                }
                vTaskDelay(delay_ms / portTICK_PERIOD_MS);  // delay

                // Shift the window over to make room for the new sample
                window_shifter(vol_ang, WINDOW_SIZE);

                // Read Voltage sample
                uint32_t volt_samp = read_voltage();

                // Put new values into struct
                vol_ang[0].angle = i_angle;
                vol_ang[0].volt = volt_samp;

                int i; // printing increment index variable
                printf("\nVoltage Window [i_sweep = %d] %d: ", i_sweep, vol_ang[WINDOW_SIZE-1].volt);
                for (i = 0; i < WINDOW_SIZE; i++) {
                    printf("%d, ", vol_ang[i].volt);
                }
                printf("\nAngle Window:   ");
                for (i = 0; i < WINDOW_SIZE; i++) {
                    printf("%d, ", vol_ang[i].angle);
                }

                // Print on Alphanumeric Display: ~ ~ ~ ~ ~
                // Convert into a String
                char angle_buf[50];
                if (servo_pin_index == 0) {
                    sprintf(angle_buf, "%d%d", abs(vol_ang[0].angle), abs(max_vol_ang[1]));
                } 
                else if (servo_pin_index == 1) {
                    sprintf(angle_buf, "%d%d", abs(max_vol_ang[0]), abs(vol_ang[0].angle));
                }
                
                write_alpha_display(angle_buf);

                i_pw += step_pw;  // increment PW by step
            }
            // Else: If this is one of the sweeps . . .
            else if ((i_sweep < NUM_OF_SWEEPS) && (move_condition == 0))
            {
                // FIND MAXIMUM VOLTAGE POSITION ~ ~ ~ ~ ~ ~ ~ ~ ~

                max_vol[i_sweep] = vol_ang[0].volt;     // initialize max voltage (for the sweep) as latest sample
                max_vol_ang[i_sweep] = vol_ang[0].volt; // initialize max voltage angle (for the sweep) as latest sample

                int j;
                // Go through the window and find the angle of maximum voltage (light brightness)
                for (j = 0; j < (WINDOW_SIZE-1); j++) {
                    // if the current voltage is higher . . .
                    if (vol_ang[j+1].volt > max_vol[i_sweep]) {
                        // Make it the new maximum
                        max_vol[i_sweep] = vol_ang[j+1].volt;
                        max_vol_ang[i_sweep] = vol_ang[j+1].angle;
                    }
                }

                int i; // printing increment index variable
                printf("\n\nVoltage Window %d: ", vol_ang[WINDOW_SIZE-1].volt);
                for (i = 0; i < WINDOW_SIZE; i++) {
                    printf("%d, ", vol_ang[i].volt);
                }

                // Convert angle -> pulse width
                max_vol_pw[i_sweep] = range_map(max_vol_ang[i_sweep], max_aval_angle, min_aval_angle, max_aval_pw, min_aval_pw);
                // Print results
                printf("\n\n<[%d]> Maximum Voltage Position: voltage = %dmV, Angle = %d° or Pulse Width = %dms\n\n", (i_sweep+1), max_vol[i_sweep], max_vol_ang[i_sweep], max_vol_pw[i_sweep]);

                // Go To Maximum Voltage Angle
                if (servo_pin_index == 0) {
                    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, max_vol_pw[i_sweep]);   // set the position
                } else if (servo_pin_index == 1) {
                    mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, max_vol_pw[i_sweep]);   // set the position
                }
                vTaskDelay(delay_ms / portTICK_PERIOD_MS);  // delay

                i_sweep += 1;   // increment sweep #

                // Reset pulse width increment
                i_pw = min_aval_pw;   

                // If there are still more sweeps left . . .
                if (i_sweep < NUM_OF_SWEEPS) 
                {
                    // SWITCH TO OTHER & RESET ~ ~ ~ ~ ~ ~ ~ ~ ~
                    // i_sweep += 1;   // increment to next sweep

                    // Switch which servo is being used
                    servo_pin_index = !(servo_pin_index);

                    // Reset Struct for Samples
                    reset_struct();

                    // Reset the next servo that needs to be used
                    start_pw_0 = min_aval_pw;   // reset start pulse width (SERVO #0) to mid-sweep start position
                    start_pw_1 = min_aval_pw;   // reset start pulse width (SERVO #1) to mid-sweep start position
                    if (servo_pin_index == 0) {
                        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, i_pw);   // set the position
                    } else if (servo_pin_index == 1) {
                        mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, i_pw);   // set the position
                    }
                    vTaskDelay(delay_ms / portTICK_PERIOD_MS);  // delay
                }
                // If it is the last sweep . . .
                else if (i_sweep == NUM_OF_SWEEPS){
                    // Reset Struct for Samples
                    reset_struct();

                    // Reset light search state toggle to OFF
                    search_state = 0;
                    
                    // Reset sweep index
                    i_sweep = 0; 
                    
                    // Reset Servos to light search start position
                    start_pw_0 = min_aval_pw;   // reset start pulse width (SERVO #0)
                    start_pw_1 = 1500;   // reset start pulse width (SERVO #1)
                }

            }
        }
        
        
    vTaskDelay(while_loop_delay / portTICK_PERIOD_MS);  // delay for the task while loop

    }
}


////////////////////////////////////////////////////////////////////////////////
// TASK & MAIN FUNCTIONS ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~

// TASK 1: counting timer task
static void timer_evt_task(void *arg) {
    printf("\nTask 1: Timer Event ~ ~ ~ ~ ~ ~ ~"); 

    while (1) {
       
        // Create dummy structure to store structure from queue
        timer_event_t evt;

        // Transfer from queue
        xQueueReceive(timer_queue, &evt, portMAX_DELAY);

        // Search for the light if triggered!
        if (evt.flag == 1) {
            printf("\n\n---SEARCHING---\n"); // print
            search_trigger = 1;               // Set light search toggle to ON
        }
    }
}

void init() 
{
    // Setup the servo
    setup_servo();

    // SET UP SERIAL COMMUNICATION ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    /* Install UART driver for interrupt-driven reads and writes */
    ESP_ERROR_CHECK( uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0) );
    /* Tell VFS to use UART driver */
    esp_vfs_dev_uart_use_driver(UART_NUM_0);

    // SET UP I2C ALPHANUMERIC DISPLAY ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    setup_alpha_display();      // Set-Up Oscillator, Blink, & Brightness
    // Set SCK & SDA pins as pullups
    gpio_set_pull_mode(22, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(23, GPIO_PULLUP_ONLY);
    // Write to display
    char disp_initial[4] = {'A','B','C','D'};
    write_alpha_display(disp_initial);

    // SET UP VOLTAGE READER  ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
    //Check if Two Point or Vref are burned into eFuse
    check_efuse();

    //Configure ADC
    if (unit == ADC_UNIT_1) {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(channel, atten);
    } else {
        adc2_config_channel_atten((adc2_channel_t)channel, atten);
    }

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);

    // Initiate alarm using timer API
    alarm_init();


    // Set Up Structure for Samples
    reset_struct();

}

// Main Function
void app_main(void) 
{
    // Create a FIFO queue for timer-based
    timer_queue = xQueueCreate(10, sizeof(timer_event_t));

    printf("\n<<<Setup>>>");
    init();				// Initialize stuff

    printf("\n[[[START]]]");
    // Run the tasks
    xTaskCreate(timer_evt_task, "timer_evt_task", 1024*2, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(wide_servo_sweep_task, "wide_servo_sweep_task", 1024*2, NULL, configMAX_PRIORITIES-1, NULL);
    // xTaskCreate(timer_evt_task, "timer_evt_task", 2048, NULL, 5, NULL);
    // xTaskCreate(wide_servo_sweep_task, "wide_servo_sweep_task", 2048, NULL, 4, NULL);

    // printf("\n<<<Setup>>>");
    // init();				// Initialize stuff

}
