// Buggy
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "driver/uart.h"
#include "esp_vfs_dev.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "driver/i2c.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "esp_log.h"
#include "driver/mcpwm.h"
#include <math.h>
#include "alphanumTable.h"
//Wheel Speed
#include "freertos/queue.h"
#include "driver/ledc.h"
#include "driver/pcnt.h"
#include "esp_attr.h"
#include "esp_types.h"			// timer
#include "driver/periph_ctrl.h" // timer
#include "driver/timer.h" 		// timer

static const char *TAG = "Roadtrip";


// Macros: steering servo
#define servoMinPw 750
#define servoMaxPw 2000
#define servoMaxDeg 90
#define servoPwFreq 50
#define servoPin 13

// Macros: motor driver
#define motorInitTime 5
#define motorMinPw 900
#define motorMaxPw 2100
#define motorMedianPw 1500
#define motorMaxVel 100
#define motorPwFreq 50
#define motorPin 12

// Macros: PID controller
#define Kp 5.0        //0.1
#define Ki 0.009        //0.1   <-- this one makes it go crazy!!!
#define Kd 0.15        //0.1
#define maxOut 100
#define minOut 0

// Macros: alphanumeric display
#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define SAMPLING_RATE   64          //Multisampling
#define SAMPLING_PERIOD 1        //Sampling period, in seconds

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

// Macros: LiDAR
#define lidarAddrRight 0x62
#define measureValue 0x04
#define measureRegister 0x00
#define allBytes 0x8f

// DEFINE: Pulse Counter (Wheel Speed) /////////////////////////////////////////////////////
#define PCNT_H_LIM_VAL      1000	// Upper Limit of pulse counter
#define PCNT_L_LIM_VAL     -10      // Lower Limit of pulse counter
#define PCNT_THRESH1_VAL    5
#define PCNT_THRESH0_VAL   -5
#define PCNT_INPUT_SIG_IO   4   // Pulse Input GPIO
#define PCNT_INPUT_CTRL_IO  5   // Control GPIO HIGH=count up, LOW=count down
//#define LEDC_OUTPUT_IO      18 // Output GPIO of a sample 1 Hz pulse generator
#define FILTER_VAL			50	// "PCNT signal filter value, counter in APB_CLK cycles. Any pulses lasting shorter than this will be ignored when the filter is enabled. "

// DEFINE: Timer (Wheel Speed) /////////////////////////////////////////////////////
#define TIMER_DIVIDER         16    //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // to seconds
#define TIMER_INTERVAL_SEC   (0.5)    // Sample Period for gathering pulses (reset every _blank_ seconds)
#define TEST_WITH_RELOAD      1     // Testing will be done with auto reload

#define WHEEL_CIRCUMFERENCE		0.22	// wheel circumference in meters
#define PULSES_PER_ROTATION		6.0		// number of black regions on the wheel


// Global Variables: servo and motor control
char inString[8];
int servoSetpoint = 0; // Set the servo setpoint to 0 by default
float motorSetpoint; // Set the motor setpoint to 0 by default
int motorSpeed;
bool servoFlag = true; // Initialize the servo
bool motorFlag = false;

// Global Variables: PID Controller
float error;
float prevError = 0;
float integral = 0;
float derivative = 0;
float timeStep = 100;
bool pidFlag = false;

// Global Variables: alphanumeric display
char inString[8];
char lastString[8];
bool writeAlphaFlag = false;
int distance;

// Global Variables: LiDAR
float distanceForward;

// GLOBAL VARIABLES: Pulse Counter (Wheel Speed) /////////////////////////////////////////////////////
// Global Variable for Counting Pulses
int16_t count = 0;  // pulse count (per sample time)
float wheel_speed;  // speed of the wheel calculated from pulses
float lastWheelSpeed = 0; // last wheel speed measurement

xQueueHandle pcnt_evt_queue;   // A queue to handle pulse counter events

/* A sample structure to pass events from the PCNT
 * interrupt handler to the main program.
 */
typedef struct {
    int unit;  // the PCNT unit that originated an interrupt
    uint32_t status; // information on the event type that caused the interrupt
} pcnt_evt_t;

// GLOBAL VARIABLES: Timer (Wheel Speed) /////////////////////////////////////////////////////
// A simple structure to pass "events" to main task
typedef struct {
    int flag;     // flag for enabling stuff in main code
} timer_event_t;

// Initialize queue handler for timer-based events
xQueueHandle timer_queue;


// Helper function declarations
int servoDegToPw(int angle);
int motorThrottle(int level);
void initializeMotorDriver(int time);

static void i2c_example_master_init();
int testConnection(uint8_t devAddr, int32_t timeout);
static void i2c_scanner();
int alpha_oscillator();
int no_blink();
int set_brightness_max(uint8_t val);

void writeToRegister(uint8_t lidarAddr, uint8_t reg, uint8_t data);
uint16_t readRegister(uint8_t lidarAddr, uint8_t reg);

// Task Function Definitions
void vTask_actuateServo();
void vTask_actuateMotor();
void vTask_readSerial();
void vTask_PIDController();
void vTask_writeToAlphanum();
void vTask_readLidar();

// Pulse Counter Function Definitions
static void IRAM_ATTR pcnt_example_intr_handler(void *arg);
static void pcnt_example_init(int unit);
void IRAM_ATTR timer_group0_isr(void *para);
static void alarm_init();
static void timer_evt_task(void *arg);
// void pulse_counter_task();
void init_wheelSpeed();


void app_main(void)
{
    // Initialize alphanumeric display
    i2c_example_master_init();
    i2c_scanner();
    xTaskCreate(vTask_writeToAlphanum, "writeToAlphanum", 4096, NULL, configMAX_PRIORITIES-2, NULL);

    // Create a FIFO queue for timer-based events
	timer_queue = xQueueCreate(10, sizeof(timer_event_t));

    // Initialize the motor driver
    initializeMotorDriver(motorInitTime);

    // Initialize Wheel Speed Stuff
    init_wheelSpeed();

    // WHEEL SPEED /////////////////////////////////////////////////////
	// Timer Task
	xTaskCreate(timer_evt_task, "timer_evt_task", 4096, NULL, configMAX_PRIORITIES, NULL);
	// Pulse Counting Task
	// xTaskCreate(pulse_counter_task, "pulse_counter_task", 4096, NULL, configMAX_PRIORITIES-1, NULL);

    // BUGGY /////////////////////////////////////////////////////
    // Set up main tasks
    xTaskCreate(vTask_PIDController, "PIDController", 4096, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(vTask_readSerial, "readSerial", 4096, NULL, configMAX_PRIORITIES-3, NULL);
    xTaskCreate(vTask_actuateServo, "actuateServo", 4096, NULL, configMAX_PRIORITIES-4, NULL);
    xTaskCreate(vTask_actuateMotor, "actuateMotor", 4096, NULL, configMAX_PRIORITIES-5, NULL);
    xTaskCreate(vTask_readLidar, "readLidar", 4096, NULL, configMAX_PRIORITIES-6, NULL);

    
}

// ~~~~~~~~~~ Task Function Declarations ~~~~~~~~~~

// readSerial: Reads serial monitor for user input to control steering and acceleration
void vTask_readSerial(){

    // Install UART driver for interrupt-driven reads and writes
    ESP_ERROR_CHECK( uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0));

    // Tell VFS to use UART driver
    esp_vfs_dev_uart_use_driver(UART_NUM_0);

    // Give a brief set of instructions before the program begins
    printf("Please send one integer at a time, preceded by an S for steering and an M for (drive) motor.\n\n");

    for(;;){
        // Grab the serial buffer once the user has pressed enter
        gets(inString);

        if(inString[0] != '\0'){ // If the first element of the string is non-zero...
            char firstChar[1];

            firstChar[0] = inString[0]; // obain the first character in the string...
            char *val = inString + 1; // place the rest of the string in the val variable, assuming it is simply a value 

            if(firstChar[0] == 'S'){ // If the first character is S...
                servoSetpoint = atoi(val); // convert the value into a number and set it as the servo setpoint...
                servoFlag = true; // raise the servo flag.
            }
            else if(firstChar[0] == 'M'){ // Otherwise, if the first character is M...
                motorSetpoint = atof(val); // convert the value into a number and set it as the motor setpoint...
                printf("~~~~~ USER SET SPEED IN m/s: %f\n", motorSetpoint);
            }
            else{ // Otherwise...
                printf("Give me a valid input...\n\n\n"); // Make the user ashamed that they had to get to the error checking code.
                // Add insult to injury:
                vTaskDelay(pdMS_TO_TICKS(1000));
                printf("       ...asshole. \n");
            }
        }

        // Delay to make the ESP happy
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// actuateServo: Takes in serial reading of user input, sets steering servo
void vTask_actuateServo(){
    
    // // Initialize GPIO pin
    // mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, servoPin);

    // // Create object with PWM configurations
    // mcpwm_config_t servoPwConfigs = {
    //     .frequency = servoPwFreq,
    //     .cmpr_a = 0,
    //     .counter_mode = MCPWM_UP_COUNTER,
    //     .duty_mode = MCPWM_DUTY_MODE_0,
    // };

    // // Initialize the PWM generator
    // mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &servoPwConfigs);

    for(;;){
        if(servoFlag){ // If the user has sent an angle directed to the servo...
            int convertedPw = servoDegToPw(servoSetpoint); // convert the input angle given in degrees to the corresponding PWM...
            ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_A, convertedPw)); // set the pin to the correct PWM...
            printf("Steering angle set to %d degrees\n", servoSetpoint); // alert the user that the change of angle has been successful...
            // printf("PWM Value: %d\n", convertedPw); // output the PWM value for debugging purposes...
            servoFlag = false; // lower the servo flag. 
        }

        // Delay to make the ESP happy
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// actuateServo: Takes in serial reading of user input, sets steering servo
void vTask_actuateMotor(){

    // Note: Because the PWM GPIO has been initialized during the ESC initialization process, it need not be included here.

    int lastSetpoint = 1;

    for(;;){
        if(motorFlag){ // If the user has sent throttle directed to the motor...
            int convertedPw = motorThrottle(motorSpeed); // convert the input throttle given in percent to the corresponding PWM...
            ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, convertedPw)); // set the pin to the correct PWM...
            printf("Motor throttle set to %d/100\n", motorSpeed); // alert the user that the change of throttle has been successful...
            if(motorSpeed == 0){
                if(lastSetpoint > 0){
                    ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, motorThrottle(-20)));
                }
                else if(lastSetpoint < 0){
                    ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, motorThrottle(20)));
                }
                ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, motorThrottle(0)));
                vTaskDelay(pdMS_TO_TICKS(500));
                vTaskDelay(pdMS_TO_TICKS(500));
                ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, convertedPw));
            }
            // printf("PWM Value: %d\n", convertedPw); // output the PWM value for debugging purposes...
            motorFlag = false; // lower the motor flag. 
            int lastSetpoint = motorSpeed;
        }

        // Delay to make the ESP happy
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void vTask_PIDController(){
    motorSetpoint = 0;  // Initialize motor setpoint at 0
    motorSpeed = 0;     // Initialize motor speed to 0

    for(;;){
        if(pidFlag){
            error = motorSetpoint - wheel_speed;
            integral += error * timeStep;
            derivative = (error - prevError) / timeStep / 1000;
            prevError = error;
            motorSpeed = (int) round((Kp * error) + (Ki * integral) + (Kd * derivative));
            if(motorSpeed > maxOut){
                motorSpeed = maxOut;
            }
            else if(motorSpeed < minOut){
                motorSpeed = minOut;
            }
            motorFlag = true;
            lastWheelSpeed = wheel_speed;
        }
        vTaskDelay(pdMS_TO_TICKS(timeStep));
    }

}

// writeToAlphanum: Takes an input and writes it to the alphanumeric display
void vTask_writeToAlphanum() {
    // Debug
    int ret;

    // Set up routines
    // Turn on alpha oscillator
    ret = alpha_oscillator();
    // Set display blink off
    ret = no_blink();
    ret = set_brightness_max(0xF);

    // Write to characters to buffer
    uint16_t displaybuffer[8];
    displaybuffer[0] = 0;
    displaybuffer[1] = 0;
    displaybuffer[2] = 0;
    displaybuffer[3] = 0;
    uint16_t decimalPoint = 0b0100000000000000;

    // Continually writes the same command
    while (1) {
      int ascii;
      int buffiterator = alphafonttable[32];
      size_t bufseclength = 8;
      if(writeAlphaFlag){
        for(int i = 0; i < 4; i++){
          displaybuffer[i] = 0b00000000000000000;
        }
        for(int i = 0; i < sizeof(inString)/sizeof(inString[0]); i++){
          ascii = inString[i];
          if(inString[i] != '.'){
            displaybuffer[buffiterator] = alphafonttable[ascii];
            buffiterator++;
          }
          else if(inString[i] == '.'){
            displaybuffer[buffiterator - 1] = (displaybuffer[buffiterator - 1] | decimalPoint);
          }
        }
        // Send commands characters to display over I2C
        i2c_cmd_handle_t cmd4 = i2c_cmd_link_create();
        i2c_master_start(cmd4);
        i2c_master_write_byte(cmd4, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
        i2c_master_write_byte(cmd4, (uint8_t)0x00, ACK_CHECK_EN);
        i2c_master_write(cmd4, displaybuffer, bufseclength, ACK_CHECK_EN);
        i2c_master_stop(cmd4);
        ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd4, 1000 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd4);
        writeAlphaFlag = false;
      }
      vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ~~~~~ LiDAR ~~~~~
void vTask_readLidar(){
    for(;;){
        writeToRegister(lidarAddrRight, measureRegister, measureValue);

        int dataFlag = 1;
        uint16_t registerData;
        while(dataFlag){
            registerData = readRegister(lidarAddrRight, 0x01);
            dataFlag = registerData & (1 << 15);
            vTaskDelay(pdMS_TO_TICKS(5));
        }
        int distanceForwardRaw = readRegister(lidarAddrRight, allBytes);
        distanceForward = (float) distanceForwardRaw / 100.0;
        printf("Distance Measured Forward: %.2f\n", distanceForward);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ~~~~~~~~~~ Helper function Definitions ~~~~~~~~~~

// ~~~~~ Servo ~~~~~

// servoDegToPw: Converts an input angle into the appropriate PWM to actuate the servo
int servoDegToPw(int angle){
    return (angle + servoMaxDeg) * (servoMaxPw - servoMinPw) / (2 * servoMaxDeg) + servoMinPw;
}

// ~~~~~ Motor ~~~~~

// motorThrottle: Converts a throttle percentage (range: 0-motorMaxVel) to a PWM to actuate the ECS
int motorThrottle(int level){
    return (level + motorMaxVel) * (motorMaxPw - motorMinPw) / (2 * motorMaxVel) + motorMinPw;
}

// initializeMotorDriver: Guides the user through the correct sequence to initialize the motor driver
void initializeMotorDriver(int time){

    // Initialize GPIO Pins
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, servoPin);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, motorPin);
    
    // Create object with PWM configurations
    mcpwm_config_t motorPwConfigs = {
        .frequency = motorPwFreq,
        .cmpr_a = 0,
        .cmpr_b = 0,
        .counter_mode = MCPWM_UP_COUNTER,
        .duty_mode = MCPWM_DUTY_MODE_0,
    };

    // Initialize the PWM generator
    mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &motorPwConfigs);

    // Print statement to instruct the user to turn on the buggy at the appropriate time]
    vTaskDelay(pdMS_TO_TICKS(1000));
    printf("\n\nInitializing MOTOR DRIVER...\n");
    sprintf(inString, "INIT");
    writeAlphaFlag = true;
    vTaskDelay(pdMS_TO_TICKS(1000));

    printf("Please power the buggy on in\n");
    for(int i = 3; i > 0; i--){
        printf("%d...\n", i);
        sprintf(inString, "IN %d", (uint8_t)i);
        writeAlphaFlag = true;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    sprintf(inString, "P ON");
    writeAlphaFlag = true;
    printf("POWER ON BUGGY\n");

    // Set PWM to the initial (median) value accepted by the ESC
    ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, motorMedianPw));

    // Wait for the required time
    vTaskDelay(pdMS_TO_TICKS(time * 1000));

    // Print statement to guide the user through the process
    printf("Motor Driver Initialized.\n");
    sprintf(inString, "REDY");
    writeAlphaFlag = true;
}

// ~~~~~ alphanumeric display ~~~~~

// Function to initiate i2c -- note the MSB declaration!
static void i2c_example_master_init(){
    // Debug
    // printf("\n>> i2c Config\n");
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
    conf.clk_flags = 0;                                     // <-- UNCOMMENT IF YOU GET ERRORS (see readme.md)
    err = i2c_param_config(i2c_master_port, &conf);           // Configure
    // if (err == ESP_OK) {printf("- parameters: ok\n");}

    // Install I2C driver
    err = i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                       I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
    // i2c_set_data_mode(i2c_master_port,I2C_DATA_MODE_LSB_FIRST,I2C_DATA_MODE_LSB_FIRST);
    // if (err == ESP_OK) {printf("- initialized: yes\n\n");}

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
    int err = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return err;
}

// Utility function to scan for i2c device
static void i2c_scanner() {
    int32_t scanTimeout = 1000;
    // printf("\n>> I2C scanning ..."  "\n");
    uint8_t count = 0;
    for (uint8_t i = 1; i < 127; i++) {
        // printf("0x%X%s",i,"\n");
        if (testConnection(i, scanTimeout) == ESP_OK) {
            // printf( "- Device found at address: 0x%X%s", i, "\n");
            count++;
        }
    }
    if (count == 0)
        // printf("- No I2C devices found!" "\n");
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
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  vTaskDelay(200 / portTICK_RATE_MS);
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
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd2, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd2);
  vTaskDelay(200 / portTICK_RATE_MS);
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
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd3, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd3);
  vTaskDelay(200 / portTICK_RATE_MS);
  return ret;
}

// I2C Read/Write Functions:

void writeToRegister(uint8_t lidarAddr, uint8_t reg, uint8_t data){
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (lidarAddr << 1) | WRITE_BIT, I2C_MASTER_ACK);
    i2c_master_write_byte(cmd, reg, I2C_MASTER_ACK); 
    i2c_master_write_byte(cmd, data, I2C_MASTER_ACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

uint16_t readRegister(uint8_t lidarAddr, uint8_t reg){
    uint8_t dataFirst; 
    uint8_t dataLast; 

    i2c_cmd_handle_t cmdFirst = i2c_cmd_link_create();
    i2c_cmd_handle_t cmdLast = i2c_cmd_link_create();

    i2c_master_start(cmdFirst);
    i2c_master_write_byte(cmdFirst, ( lidarAddr << 1 ) | WRITE_BIT, I2C_MASTER_ACK);
    i2c_master_write_byte(cmdFirst, reg, I2C_MASTER_ACK);
    i2c_master_stop(cmdFirst);
    i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmdFirst, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmdFirst);

    i2c_master_start(cmdLast);
    i2c_master_write_byte(cmdLast, ( lidarAddr << 1 ) | READ_BIT, I2C_MASTER_ACK);
    i2c_master_read_byte(cmdLast, &dataFirst , I2C_MASTER_ACK);
    i2c_master_read_byte(cmdLast, &dataLast , I2C_MASTER_NACK);
    i2c_master_stop(cmdLast);
    i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmdLast, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmdLast);
    

    uint16_t wholeData = (dataFirst << 8 | dataLast);
    return wholeData;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCTIONS: Pulse Counter (Wheel Speed) //////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////

/* Decode what PCNT's unit originated an interrupt
 * and pass this information together with the event type
 * the main program using a queue.
 */
static void IRAM_ATTR pcnt_example_intr_handler(void *arg)
{
    int pcnt_unit = (int)arg;
    pcnt_evt_t evt;
    evt.unit = pcnt_unit;
    /* Save the PCNT event type that caused an interrupt
       to pass it to the main program */
    pcnt_get_event_status(pcnt_unit, &evt.status);
    xQueueSendFromISR(pcnt_evt_queue, &evt, NULL);
}


/* Initialize PCNT functions:
 *  - configure and initialize PCNT
 *  - set up the input filter
 *  - set up the counter events to watch
 */
static void pcnt_example_init(int unit)
{
    /* Prepare configuration for the PCNT unit */
    pcnt_config_t pcnt_config = {
        // Set PCNT input signal and control GPIOs
        .pulse_gpio_num = PCNT_INPUT_SIG_IO,
        .ctrl_gpio_num = PCNT_INPUT_CTRL_IO,
        .channel = PCNT_CHANNEL_0,
        .unit = unit,
        // What to do on the positive / negative edge of pulse input?
        .pos_mode = PCNT_COUNT_INC,   // Count up on the positive edge
        .neg_mode = PCNT_COUNT_DIS,   // Keep the counter value on the negative edge
        // What to do when control input is low or high?
        .lctrl_mode = PCNT_MODE_REVERSE, // Reverse counting direction if low
        .hctrl_mode = PCNT_MODE_KEEP,    // Keep the primary counter mode if high
        // Set the maximum and minimum limit values to watch
        .counter_h_lim = PCNT_H_LIM_VAL,
        .counter_l_lim = PCNT_L_LIM_VAL,
    };
    /* Initialize PCNT unit */
    pcnt_unit_config(&pcnt_config);

    /* Configure and enable the input filter */
    pcnt_set_filter_value(unit, FILTER_VAL);
    pcnt_filter_enable(unit);

    /* Set threshold 0 and 1 values and enable events to watch */
    pcnt_set_event_value(unit, PCNT_EVT_THRES_1, PCNT_THRESH1_VAL);
    pcnt_event_enable(unit, PCNT_EVT_THRES_1);
    pcnt_set_event_value(unit, PCNT_EVT_THRES_0, PCNT_THRESH0_VAL);
    pcnt_event_enable(unit, PCNT_EVT_THRES_0);
    /* Enable events on zero, maximum and minimum limit values */
    pcnt_event_enable(unit, PCNT_EVT_ZERO);
    pcnt_event_enable(unit, PCNT_EVT_H_LIM);
    pcnt_event_enable(unit, PCNT_EVT_L_LIM);

    /* Initialize PCNT's counter */
    pcnt_counter_pause(unit);
    pcnt_counter_clear(unit);

    /* Install interrupt service and add isr callback handler */
    pcnt_isr_service_install(0);
    pcnt_isr_handler_add(unit, pcnt_example_intr_handler, (void *)unit);

    /* Everything is set up, now go to counting */
    pcnt_counter_resume(unit);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCTIONS: Timer (Wheel Speed) //////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////


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

// Initialize timer 0 in group 0 for TIMER_INTERVAL_SEC sec alarm interval & auto reload
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


////////////////////////////////////////////////////////////////////////////////////////////////////////
// TASKS: Wheel Speed //////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////

// TASK: Timer (TIMER_INTERVAL_SEC second sample period)
static void timer_evt_task(void *arg) {

	// int pulse_count_unit = PCNT_UNIT_0; // pulse counting unit handle
    int pcnt_unit = PCNT_UNIT_0;	// pulse counting unit handle

    /* Initialize PCNT event queue and PCNT functions */
	pcnt_evt_queue = xQueueCreate(10, sizeof(pcnt_evt_t));
	pcnt_example_init(pcnt_unit);

	pcnt_evt_t evt_pcnt;
	portBASE_TYPE res;

    // Create dummy structure to store structure from queue
    timer_event_t evt_timer;

    while (1) {

        res = xQueueReceive(pcnt_evt_queue, &evt_pcnt, 1000 / portTICK_PERIOD_MS);

        // Transfer from queue
        xQueueReceive(timer_queue, &evt_timer, portMAX_DELAY);

        // If the Calculate Speed event is triggered (1 second has passed)
        if (evt_timer.flag == 1) {
            pcnt_get_counter_value(pcnt_unit, &count); // get the current pulse count
            // Calculate the Speed
            // (distance/time) = (distance/rotation) * (rotation/pulses) * (pulses/sample) * (sample/time)
            wheel_speed = WHEEL_CIRCUMFERENCE * (1 / PULSES_PER_ROTATION) * count * (1 / TIMER_INTERVAL_SEC) ;
            printf("\nCount: %d pulses/sec,   Speed: %.4f m/s \n", count, wheel_speed); // Print results
            pcnt_counter_clear(pcnt_unit);	// reset pulse count
            evt_timer.flag = 0; // lower timer flag
            sprintf(inString, "%1.3f", wheel_speed);
            writeAlphaFlag = true;
            pidFlag = true;
        }
            

    }
}

// // TASK: Pulse Counter (Wheel Speed)
// void pulse_counter_task() {
// 	int pcnt_unit = PCNT_UNIT_0;	// pulse counting unit handle

// 	/* Initialize PCNT event queue and PCNT functions */
// 	pcnt_evt_queue = xQueueCreate(10, sizeof(pcnt_evt_t));
// 	pcnt_example_init(pcnt_unit);

// 	pcnt_evt_t evt;
// 	portBASE_TYPE res;
// 	while (1) {
// 		/* Wait for the event information passed from PCNT's interrupt handler.
// 		 * Once received, decode the event type and print it on the serial monitor.
// 		 */
// 		res = xQueueReceive(pcnt_evt_queue, &evt, 1000 / portTICK_PERIOD_MS);
// 		if (res == pdTRUE) {
// 			pcnt_get_counter_value(pcnt_unit, &count);
// 			// ESP_LOGI(TAG, "Event PCNT unit[%d]; cnt: %d", evt.unit, count);
// 			if (evt.status & PCNT_EVT_THRES_1) {
// 				// ESP_LOGI(TAG, "THRES1 EVT");
// 			}
// 			if (evt.status & PCNT_EVT_THRES_0) {
// 				// ESP_LOGI(TAG, "THRES0 EVT");
// 			}
// 			if (evt.status & PCNT_EVT_L_LIM) {
// 				// ESP_LOGI(TAG, "L_LIM EVT");
// 			}
// 			if (evt.status & PCNT_EVT_H_LIM) {
// 				// ESP_LOGI(TAG, "H_LIM EVT");
// 			}
// 			if (evt.status & PCNT_EVT_ZERO) {
// 				// ESP_LOGI(TAG, "ZERO EVT");
// 			}
// 		} else {
// 			pcnt_get_counter_value(pcnt_unit, &count);
// 			// ESP_LOGI(TAG, "Current counter value :%d", count);
// 		}
// 	}
// }


////////////////////////////////////////////////////////////////////////////////////////////////////////
// INITIALIZATION  /////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////

// Initialization
void init_wheelSpeed() {
	// Initiate alarm using timer API
	alarm_init();
}
