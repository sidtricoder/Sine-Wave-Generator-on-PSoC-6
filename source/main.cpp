/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for running a sine wave neural network
*              model on PSoC 6 using TensorFlow Lite for Microcontrollers.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2022-2024, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress does
* not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

/* ML/TensorFlow includes */
#include "tensorflow/lite/c/common.h"
#include "tensorflow/lite/core/c/common.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"

/* Our model */
#include "models/sine_model.h"

#include <cstdarg> // Required for va_list, va_start, va_end
#include <cstdio>  // Required for vsnprintf, printf

/*******************************************************************************
* Macros
*******************************************************************************/
/* Debug flag - set to 1 for debug output */
#define DEBUG 1

/* Some settings for our sine wave model */
#define PI 3.14159265f                   /* Pi constant */
#define SINE_FREQ 0.5f                   /* Frequency (Hz) of sinewave */
#define SINE_PERIOD ((1.0f / SINE_FREQ) * (1000000.0f))  /* Period (microseconds) */

/* PWM LED configuration */
#define PWM_FREQUENCY 1000000u    /* 1 MHz */
#define PWM_PERIOD 255            /* 255 for 8-bit resolution */

/* LED blink timer clock value in Hz  */
#define LED_BLINK_TIMER_CLOCK_HZ          (10000)

/* LED blink timer period value */
#define LED_BLINK_TIMER_PERIOD            (9999)


/*******************************************************************************
* Global Variables
*******************************************************************************/
bool timer_interrupt_flag = false;
bool led_blink_active_flag = true;

/* Variable for storing character read from terminal */
uint8_t uart_read_value;

/* Timer object used for blinking the LED */
cyhal_timer_t led_blink_timer;

/* PWM LED object */
cyhal_pwm_t pwm_led_obj;

/* Timer object used for tracking time */
cyhal_timer_t timer_obj;

/* TFLite globals */
namespace {
    const tflite::Model* model = nullptr;
    tflite::MicroInterpreter* interpreter = nullptr;
    TfLiteTensor* model_input = nullptr;
    TfLiteTensor* model_output = nullptr;

    /* Create an area of memory for input, output, and other TensorFlow arrays */
    constexpr int kTensorArenaSize = 10 * 1024;
    uint8_t tensor_arena[kTensorArenaSize];
} /* namespace */


/* TensorFlow Lite Micro utility function for printing */
// Remove extern "C" to match the TFLM header declaration (C++ linkage)
void MicroPrintf(const char* format, ...) {
  char print_buf[256]; // Reasonably sized buffer for formatted string
  va_list args;
  va_start(args, format);
  vsnprintf(print_buf, sizeof(print_buf), format, args);
  va_end(args);
  printf("%s", print_buf); // Route to standard printf
}

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
void timer_init(void);
static void isr_timer(void *callback_arg, cyhal_timer_event_t event);
void ml_init(void);
void pwm_init(void);
void micro_timer_init(void);
static uint32_t get_time_us(void);

// extern "C" is important for C++ to C linkage for DebugLog
extern "C" void DebugLog(const char* s) {
    printf("%s", s);
}

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function that initializes the system, sets up the TensorFlow
* Lite model, and runs the sine wave inference in a loop. The LED brightness
* varies based on the predicted sine wave value from the neural network model.
*
* Parameters:
*  none
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;

#if defined (CY_DEVICE_SECURE)
    cyhal_wdt_t wdt_obj;

    /* Clear watchdog timer so that it doesn't trigger a reset */
    result = cyhal_wdt_init(&wdt_obj, cyhal_wdt_get_max_timeout_ms());
    CY_ASSERT(CY_RSLT_SUCCESS == result);
    cyhal_wdt_free(&wdt_obj);
#endif /* #if defined (CY_DEVICE_SECURE) */

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize retarget-io to use the debug UART port */
    result = cy_retarget_io_init_fc(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
            CYBSP_DEBUG_UART_CTS,CYBSP_DEBUG_UART_RTS,CY_RETARGET_IO_BAUDRATE);

    /* retarget-io init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");

    printf("**************************************************************\r\n");
    printf("*   PSoC 6 TensorFlow Lite Sine Wave Model Demo              *\r\n");
    printf("**************************************************************\r\n\n");

    /* Initialize PWM for LED brightness control */
    pwm_init();
    
    /* Initialize timer for microsecond timing */
    micro_timer_init();
    
    /* Initialize the ML model */
    ml_init();
    
    /* Initialize timer for LED blinking (kept from original example) */
    timer_init();
    
    printf("Running sine wave inference...\r\n\n");
    printf("Press 'Enter' key to pause or resume the demo\r\n\n");

    for (;;)
    {
        /* Check if 'Enter' key was pressed */
        if (cyhal_uart_getc(&cy_retarget_io_uart_obj, &uart_read_value, 1)
             == CY_RSLT_SUCCESS)
        {
            if (uart_read_value == '\r')
            {
                /* Pause/resume demo */
                if (led_blink_active_flag)
                {
                    printf("Demo paused \r\n");
                }
                else
                {
                    printf("Demo resumed\r\n");
                }

                /* Move cursor to previous line */
                printf("\x1b[1F");

                led_blink_active_flag ^= 1;
            }
        }
        
        /* Only run inference when demo is active */
        if (led_blink_active_flag)
        {
            /* Get current timestamp and modulo with period */
            uint32_t timestamp = get_time_us() % (uint32_t)SINE_PERIOD;
            
            /* Calculate x value to feed to the model (convert timestamp to angle in radians) */
            float x_val = ((float)timestamp * 2.0f * PI) / SINE_PERIOD;
            
#if DEBUG
            uint32_t start_time = get_time_us();
#endif
            
            /* Copy value to input buffer (tensor) */
            model_input->data.f[0] = x_val;
            
            /* Run inference */
            TfLiteStatus invoke_status = interpreter->Invoke();
            if (invoke_status != kTfLiteOk) {
                printf("Invoke failed on input: %f\r\n", x_val);
            }
            
            /* Read predicted y value from output buffer (tensor) */
            float y_val = model_output->data.f[0];
            
            /* Translate to a PWM LED brightness (map from -1,1 to 0,100%) */
            float duty_cycle = (y_val + 1.0f) * 50.0f;
            
            /* Set PWM duty cycle to control LED brightness */
            cyhal_pwm_set_duty_cycle(&pwm_led_obj, duty_cycle, 100.0f);
            
            /* Print values */
            printf("Input: %f, Output: %f\r\n", x_val, y_val);
            
#if DEBUG
            printf("Time for inference (us): %lu\r\n", (get_time_us() - start_time));
#endif
            
            /* Small delay to avoid flooding the console */
            cyhal_system_delay_ms(100);
        }
        
        /* Handle LED blinking from original example (using timer interrupt) */
        if (timer_interrupt_flag)
        {
            timer_interrupt_flag = false;
            
            /* No need to toggle LED here since we're controlling it with PWM */
        }
    }
}


/*******************************************************************************
* Function Name: timer_init
********************************************************************************
* Summary:
* This function creates and configures a Timer object. The timer ticks
* continuously and produces a periodic interrupt on every terminal count
* event. The period is defined by the 'period' and 'compare_value' of the
* timer configuration structure 'led_blink_timer_cfg'. Without any changes,
* this application is designed to produce an interrupt every 1 second.
*
* Parameters:
*  none
*
* Return :
*  void
*
*******************************************************************************/
 void timer_init(void)
 {
    cy_rslt_t result;

    const cyhal_timer_cfg_t led_blink_timer_cfg =
    {
        .is_continuous = true,              /* Run timer indefinitely */
        .direction = CYHAL_TIMER_DIR_UP,    /* Timer counts up */
        .is_compare = false,                /* Don't use compare mode */
        .period = LED_BLINK_TIMER_PERIOD,   /* Defines the timer period */
        .compare_value = 0,                 /* Timer compare value, not used */
        .value = 0                          /* Initial value of counter */
    };

    /* Initialize the timer object. Does not use input pin ('pin' is NC) and
     * does not use a pre-configured clock source ('clk' is NULL). */
    result = cyhal_timer_init(&led_blink_timer, NC, NULL);

    /* timer init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

    /* Configure timer period and operation mode such as count direction,
       duration */
    cyhal_timer_configure(&led_blink_timer, &led_blink_timer_cfg);

    /* Set the frequency of timer's clock source */
    cyhal_timer_set_frequency(&led_blink_timer, LED_BLINK_TIMER_CLOCK_HZ);

    /* Assign the ISR to execute on timer interrupt */
    cyhal_timer_register_callback(&led_blink_timer, isr_timer, NULL);

    /* Set the event on which timer interrupt occurs and enable it */
    cyhal_timer_enable_event(&led_blink_timer, CYHAL_TIMER_IRQ_TERMINAL_COUNT,
                              7, true);

    /* Start the timer with the configured settings */
    cyhal_timer_start(&led_blink_timer);
 }


/*******************************************************************************
* Function Name: isr_timer
********************************************************************************
* Summary:
* This is the interrupt handler function for the timer interrupt.
*
* Parameters:
*    callback_arg    Arguments passed to the interrupt callback
*    event            Timer/counter interrupt triggers
*
* Return:
*  void
*******************************************************************************/
static void isr_timer(void *callback_arg, cyhal_timer_event_t event)
{
    (void) callback_arg;
    (void) event;

    /* Set the interrupt flag and process it from the main while(1) loop */
    timer_interrupt_flag = true;
}

/*******************************************************************************
* Function Name: ml_init
********************************************************************************
* Summary:
* Initializes the TensorFlow Lite Micro runtime and loads the sine model.
*
* Parameters:
*  none
*
* Return:
*  void
*
*******************************************************************************/
void ml_init(void)
{
    printf("Initializing TensorFlow Lite Micro...\r\n");
    
    /* Map the model into a usable data structure */
    model = tflite::GetModel(sine_model_tflite);
    if (model->version() != TFLITE_SCHEMA_VERSION) {
        printf("Model version does not match Schema version\r\n");
        CY_ASSERT(0);
    }
    
    /* Create a resolver for the operations - add only what's needed */
    static tflite::MicroMutableOpResolver<4> resolver;
    // Add operations used by the model
    resolver.AddFullyConnected();
    resolver.AddRelu();
    resolver.AddLogistic();
    resolver.AddMul();
    /* Build an interpreter to run the model */
    static tflite::MicroInterpreter static_interpreter(
        model, resolver, tensor_arena, kTensorArenaSize);
    interpreter = &static_interpreter;
    
    /* Allocate memory from the tensor_arena for the model's tensors */
    TfLiteStatus allocate_status = interpreter->AllocateTensors();
    if (allocate_status != kTfLiteOk) {
        printf("AllocateTensors() failed\r\n");
        CY_ASSERT(0);
    }
    
    /* Obtain pointers to the model's input and output tensors */
    model_input = interpreter->input(0);
    model_output = interpreter->output(0);
    
    /* Log model details */
#if DEBUG
    printf("Model loaded successfully\r\n");
    printf("Input tensor dimensions: %d\r\n", model_input->dims->size);
    printf("Input tensor shape: [%d", model_input->dims->data[0]);
    for (int i = 1; i < model_input->dims->size; i++) {
        printf(", %d", model_input->dims->data[i]);
    }
    printf("]\r\n");
    printf("Input tensor type: %d\r\n", model_input->type);
    
    printf("Output tensor dimensions: %d\r\n", model_output->dims->size);
    printf("Output tensor shape: [%d", model_output->dims->data[0]);
    for (int i = 1; i < model_output->dims->size; i++) {
        printf(", %d", model_output->dims->data[i]);
    }
    printf("]\r\n");
    printf("Output tensor type: %d\r\n", model_output->type);
    
    printf("TensorFlow Lite Micro initialization complete\r\n\n");
#endif
}

/*******************************************************************************
* Function Name: pwm_init
********************************************************************************
* Summary:
* Initializes the PWM for LED control.
*
* Parameters:
*  none
*
* Return:
*  void
*
*******************************************************************************/
void pwm_init(void)
{
    cy_rslt_t result;
    
    /* Initialize PWM on the USER_LED pin */
    result = cyhal_pwm_init(&pwm_led_obj, CYBSP_USER_LED, NULL);
    if (result != CY_RSLT_SUCCESS) {
        printf("PWM initialization failed with error code: %lu\r\n", result);
        CY_ASSERT(0);
    }
    
    /* Start the PWM */
    result = cyhal_pwm_start(&pwm_led_obj);
    if (result != CY_RSLT_SUCCESS) {
        printf("PWM start failed with error code: %lu\r\n", result);
        CY_ASSERT(0);
    }
    
    /* Set initial PWM duty cycle (0%) */
    result = cyhal_pwm_set_duty_cycle(&pwm_led_obj, 0, 100.0f);
    if (result != CY_RSLT_SUCCESS) {
        printf("PWM set duty cycle failed with error code: %lu\r\n", result);
        CY_ASSERT(0);
    }
    
    printf("PWM initialization complete\r\n");
}

/*******************************************************************************
* Function Name: micro_timer_init
********************************************************************************
* Summary:
* Initialize a timer for microsecond timing.
*
* Parameters:
*  none
*
* Return:
*  void
*
*******************************************************************************/
void micro_timer_init(void)
{
    cy_rslt_t result;
    
    /* Timer configuration structure */
    const cyhal_timer_cfg_t timer_cfg =
    {
        .is_continuous = true,              /* Run timer indefinitely */
        .direction = CYHAL_TIMER_DIR_UP,    /* Timer counts up */
        .is_compare = false,                /* Don't use compare mode */
        .period = 0xFFFFFFFF,               /* Counter period (maximum) */
        .compare_value = 0,                 /* Initial compare value */
        .value = 0                          /* Initial value of counter */
    };
    
    /* Initialize the timer object */
    result = cyhal_timer_init(&timer_obj, NC, NULL);
    if (result != CY_RSLT_SUCCESS) {
        printf("Timer initialization failed with error code: %lu\r\n", result);
        CY_ASSERT(0);
    }
    
    /* Configure timer */
    result = cyhal_timer_configure(&timer_obj, &timer_cfg);
    if (result != CY_RSLT_SUCCESS) {
        printf("Timer configuration failed with error code: %lu\r\n", result);
        CY_ASSERT(0);
    }
    
    /* Set the timer frequency to 1MHz for microsecond timing */
    result = cyhal_timer_set_frequency(&timer_obj, 1000000);
    if (result != CY_RSLT_SUCCESS) {
        printf("Timer set frequency failed with error code: %lu\r\n", result);
        CY_ASSERT(0);
    }
    
    /* Start the timer */
    result = cyhal_timer_start(&timer_obj);
    if (result != CY_RSLT_SUCCESS) {
        printf("Timer start failed with error code: %lu\r\n", result);
        CY_ASSERT(0);
    }
    
    printf("Timer initialization complete\r\n");
}

/*******************************************************************************
* Function Name: get_time_us
********************************************************************************
* Summary:
* Get current time in microseconds.
*
* Parameters:
*  none
*
* Return:
*  uint32_t Current time in microseconds
*
*******************************************************************************/
static uint32_t get_time_us(void)
{
    return cyhal_timer_read(&timer_obj);
}

/* [] END OF FILE */
