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

// Macros
#define servoMinPw 750
#define servoMaxPw 2000
#define servoMaxDeg 90
#define servoPwFreq 50
#define servoPin 13

#define motorInitTime 5
#define motorMinPw 900
#define motorMaxPw 2100
#define motorMedianPw 1500
#define motorMaxVel 100
#define motorPwFreq 50
#define motorPin 12

// Global Variable Definitons
char inString[8];
int servoSetpoint = 0; // Set the servo setpoint to 0 by default
int motorSetpoint; // Set the motor setpoint to 0 by default
bool servoFlag = true; // Initialize the servo
bool motorFlag = false;

// Helper function declarations
int servoDegToPw(int angle);
int motorThrottle(int level);
void initializeMotorDriver(int time);

// Task Function Definitions
void vTask_actuateServo();
void vTask_actuateMotor();
void vTask_readSerial();

void app_main(void)
{   
    // Initialize the motor driver
    initializeMotorDriver(motorInitTime);

    // Set up main tasks
    xTaskCreate(vTask_readSerial, "readSerial", 4096, NULL, 5, NULL);
    xTaskCreate(vTask_actuateServo, "actuateServo", 4096, NULL, 4, NULL);
    xTaskCreate(vTask_actuateMotor, "actuateMotor", 4096, NULL, 3, NULL);
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
                motorSetpoint = atoi(val); // convert the value into a number and set it as the motor setpoint...
                motorFlag = true; // raise the motor flag.
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
            int convertedPw = motorThrottle(motorSetpoint); // convert the input throttle given in percent to the corresponding PWM...
            ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, convertedPw)); // set the pin to the correct PWM...
            printf("Motor throttle set to %d/100\n", motorSetpoint); // alert the user that the change of throttle has been successful...
            if(motorSetpoint == 0){
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
            int lastSetpoint = motorSetpoint;
        }

        // Delay to make the ESP happy
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ~~~~~~~~~~ Helper function Definitions ~~~~~~~~~~

// servoDegToPw: Converts an input angle into the appropriate PWM to actuate the servo
int servoDegToPw(int angle){
    return (angle + servoMaxDeg) * (servoMaxPw - servoMinPw) / (2 * servoMaxDeg) + servoMinPw;
}

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
    vTaskDelay(pdMS_TO_TICKS(1000));

    printf("Please power the buggy on in\n");
    for(int i = 3; i > 0; i--){
        printf("%d...\n", i);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    printf("POWER ON BUGGY\n");

    // Set PWM to the initial (median) value accepted by the ESC
    ESP_ERROR_CHECK(mcpwm_set_duty_in_us(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_OPR_B, motorMedianPw));

    // Wait for the required time
    vTaskDelay(pdMS_TO_TICKS(time * 1000));

    // Print statement to guide the user through the process
    printf("Motor Driver Initialized.\n");
}