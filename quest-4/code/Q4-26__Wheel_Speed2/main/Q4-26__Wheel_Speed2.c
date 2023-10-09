


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/ledc.h"
#include "driver/pcnt.h"
#include "esp_attr.h"
#include "esp_log.h"
#include <stdio.h>				// timer
#include "esp_types.h"			// timer
#include "driver/periph_ctrl.h" // timer
#include "driver/timer.h" 		// timer

static const char *TAG = "Wheel_Speed";

/**
 * TEST CODE BRIEF
 *
 * Use PCNT module to count rising edges
 *
 * Functionality of  Pulse Counter GPIOs:
 *   - GPIO4 - pulse input pin,
 *   - GPIO5 - control input pin.
 *
 * GPIO5 is the control signal, you can leave it floating with internal pull up,
 * or connect it to ground. If left floating, the count value will be increasing.
 * If you connect GPIO5 to GND, the count value will be decreasing.
 *
 * An interrupt will be triggered when the counter value:
 *   - reaches 'thresh1' or 'thresh0' value,
 *   - reaches 'l_lim' value or 'h_lim' value,
 *   - will be reset to zero.
 */


// DEFINE: Pulse Counter (Wheel Speed) /////////////////////////////////////////////////////
#define PCNT_H_LIM_VAL      1000	// Upper Limit of pulse counter
#define PCNT_L_LIM_VAL     -10		// Lower Limit of pulse counter
#define PCNT_THRESH1_VAL    5
#define PCNT_THRESH0_VAL   -5
#define PCNT_INPUT_SIG_IO   4  // Pulse Input GPIO
#define PCNT_INPUT_CTRL_IO  5  // Control GPIO HIGH=count up, LOW=count down
//#define LEDC_OUTPUT_IO      18 // Output GPIO of a sample 1 Hz pulse generator

// DEFINE: Timer (Wheel Speed) /////////////////////////////////////////////////////
#define TIMER_DIVIDER         16    //  Hardware timer clock divider
#define TIMER_SCALE           (TIMER_BASE_CLK / TIMER_DIVIDER)  // to seconds
#define TIMER_INTERVAL_SEC   (1)    // Sample test interval for the first timer (how quick it counts up)
#define TEST_WITH_RELOAD      1     // Testing will be done with auto reload

#define WHEEL_CIRCUMFERENCE		0.62	// wheel circumference in meters
#define PULSES_PER_ROTATION		6.0		// number of black regions on the wheel


// GLOBAL VARIABLES: Pulse Counter (Wheel Speed) /////////////////////////////////////////////////////
// Global Variable for Counting Pulses
int16_t count = 0;

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
    pcnt_set_filter_value(unit, 100);
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

// Initialize timer 0 in group 0 for 1 sec alarm interval & auto reload
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

// TASK 1: Timer Task (1 second)
static void timer_evt_task(void *arg) {

	int pulse_count_unit = PCNT_UNIT_0; // pulse counting unit handle

	float wheel_speed;

    while (1) {

        // Create dummy structure to store structure from queue
        timer_event_t evt;

        // Transfer from queue
        xQueueReceive(timer_queue, &evt, portMAX_DELAY);

        // If the event is triggered (1 second has passed)
        if (evt.flag == 1) {
        	wheel_speed = WHEEL_CIRCUMFERENCE * (count/PULSES_PER_ROTATION);
            printf("\nCount: %d pulses/sec,   Speed: %.4f m/s", count, wheel_speed); // Print results
            pcnt_counter_clear(pulse_count_unit);	// reset pulse count
        }
    }
}

// TASK 1: Pulse Counter (light)
void pulse_counter() {
	int pcnt_unit = PCNT_UNIT_0;	// pulse counting unit handle

	/* Initialize PCNT event queue and PCNT functions */
	pcnt_evt_queue = xQueueCreate(10, sizeof(pcnt_evt_t));
	pcnt_example_init(pcnt_unit);

	pcnt_evt_t evt;
	portBASE_TYPE res;
	while (1) {
		/* Wait for the event information passed from PCNT's interrupt handler.
		 * Once received, decode the event type and print it on the serial monitor.
		 */
		res = xQueueReceive(pcnt_evt_queue, &evt, 1000 / portTICK_PERIOD_MS);
		if (res == pdTRUE) {
			pcnt_get_counter_value(pcnt_unit, &count);
			ESP_LOGI(TAG, "Event PCNT unit[%d]; cnt: %d", evt.unit, count);
			if (evt.status & PCNT_EVT_THRES_1) {
				ESP_LOGI(TAG, "THRES1 EVT");
			}
			if (evt.status & PCNT_EVT_THRES_0) {
				ESP_LOGI(TAG, "THRES0 EVT");
			}
			if (evt.status & PCNT_EVT_L_LIM) {
				ESP_LOGI(TAG, "L_LIM EVT");
			}
			if (evt.status & PCNT_EVT_H_LIM) {
				ESP_LOGI(TAG, "H_LIM EVT");
			}
			if (evt.status & PCNT_EVT_ZERO) {
				ESP_LOGI(TAG, "ZERO EVT");
			}
		} else {
			pcnt_get_counter_value(pcnt_unit, &count);
			ESP_LOGI(TAG, "Current counter value :%d", count);
		}
	}
}


////////////////////////////////////////////////////////////////////////////////////////////////////////
// INITIALIZATION & MAIN APP ///////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////

// Initialization
void init_setup() {
	// Initiate alarm using timer API
	alarm_init();
}

// Main App
void app_main(void)
{
	// Create a FIFO queue for timer-based events
	timer_queue = xQueueCreate(10, sizeof(timer_event_t));

	// Set Up Stuff
	init_setup();

	// WHEEL SPEED /////////////////////////////////////////////////////
	// Timer Task
	xTaskCreate(timer_evt_task, "timer_evt_task", 4096, NULL, configMAX_PRIORITIES, NULL);
	// Pulse Counting Task
	xTaskCreate(pulse_counter, "pulse_counter", 4096, NULL, configMAX_PRIORITIES-1, NULL);

}
