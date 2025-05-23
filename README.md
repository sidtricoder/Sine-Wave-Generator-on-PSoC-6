# PSoC 6 TensorFlow Lite Sine Wave Demo

This code example demonstrates running a TensorFlow Lite for Microcontrollers (TFLM) model on a PSoC&trade; 6 MCU. The model predicts the sine of an input value, and the output is used to control the brightness of an LED via PWM, visually representing the sine wave. The example also uses UART for displaying inference results and allowing user interaction (pause/resume).

## Requirements

- [ModusToolbox&trade;](https://www.infineon.com/modustoolbox) v3.1 or later (tested with v3.1)
- Board support package (BSP) minimum required version for:
   - PSoC&trade; 6 MCU: v4.2.0
- Programming language: C++
- Associated parts: All [PSoC&trade; 6 MCU](https://www.infineon.com/cms/en/product/microcontroller/32-bit-psoc-arm-cortex-microcontroller/psoc-6-32-bit-arm-cortex-m4-mcu) parts.

## Supported toolchains (make variable 'TOOLCHAIN')

- GNU Arm&reg; Embedded Compiler v11.3.1 (`GCC_ARM`) – Default value of `TOOLCHAIN`
- Arm&reg; Compiler v6.16 (`ARM`)
- IAR C/C++ Compiler v9.30.1 (`IAR`)

## Supported kits (make variable 'TARGET')

- [PSoC&trade; 6 Bluetooth&reg; LE Pioneer Kit](https://www.infineon.com/CY8CKIT-062-BLE) (`CY8CKIT-062-BLE`) – Tested Target
- [PSoC&trade; 6 Wi-Fi Bluetooth&reg; Pioneer Kit](https://www.infineon.com/CY8CKIT-062-WIFI-BT) (`CY8CKIT-062-WIFI-BT`)
- [PSoC&trade; 62S2 Wi-Fi Bluetooth&reg; Prototyping Kit](https://www.infineon.com/CY8CPROTO-062S2-43439) (`CY8CPROTO-062S2-43439`)

## Hardware setup

This example uses the board's default configuration. See the kit user guide to ensure that the board is configured correctly.

> **Note:** The PSoC&trade; 6 Bluetooth&reg; LE Pioneer Kit (CY8CKIT-062-BLE) and the PSoC&trade; 6 Wi-Fi Bluetooth&reg; Pioneer Kit (CY8CKIT-062-WIFI-BT) ship with KitProg2 installed. ModusToolbox&trade; requires KitProg3. Before using this code example, make sure that the board is upgraded to KitProg3. The tool and instructions are available in the [Firmware Loader](https://github.com/Infineon/Firmware-loader) GitHub repository. If you do not upgrade, you will see an error like "unable to find CMSIS-DAP device" or "KitProg firmware is out of date".

## Software setup

See the [ModusToolbox&trade; tools package installation guide](https://www.infineon.com/ModusToolboxInstallguide) for information about installing and configuring the tools package.
Install a terminal emulator if you don't have one. Instructions in this document use [Tera Term](https://teratermproject.github.io/index-en.html).

This example requires the TensorFlow Lite for Microcontrollers library. Ensure the TFLM component (`COMPONENT_ML_TFLM`) is included in your project. The `sine_model.tflite` file (converted to a C header, e.g., `models/sine_model.h`) must also be present in the project.

## Using the code example

### Create the project

The ModusToolbox&trade; tools package provides the Project Creator as both a GUI tool and a command line tool.

<details><summary><b>Use Project Creator GUI</b></summary>

1. Open the Project Creator GUI tool.

2. On the **Choose Board Support Package (BSP)** page, select a kit supported by this code example. See [Supported kits](#supported-kits-make-variable-target).

3. On the **Select Application** page:

   a. Select the **Applications(s) Root Path** and the **Target IDE**.

   b. Select this code example from the list by enabling its check box.

   c. (Optional) Change the suggested **New Application Name** and **New BSP Name**.

   d. Click **Create** to complete the application creation process.

</details>

<details><summary><b>Use Project Creator CLI</b></summary>

The 'project-creator-cli' tool can be used to create applications from a CLI terminal or from within batch files or shell scripts. This tool is available in the *{ModusToolbox&trade; install directory}/tools_{version}/project-creator/* directory.

Use a CLI terminal to invoke the 'project-creator-cli' tool. On Windows, use the command-line 'modus-shell' program provided in the ModusToolbox&trade; installation instead of a standard Windows command-line application. This shell provides access to all ModusToolbox&trade; tools.

The following example clones the "PSoC 6 TensorFlow Lite Sine Wave Demo" application with the desired name "SineWaveDemo" configured for the *CY8CKIT-062-BLE* BSP into the specified working directory, *C:/mtb_projects*:

   ```
   project-creator-cli --board-id CY8CKIT-062-BLE --app-id psoc6-tflm-sine-wave-demo --user-app-name SineWaveDemo --target-dir "C:/mtb_projects"
   ```

</details>

### Open the project

After the project has been created, you can open it in your preferred development environment.

<details><summary><b>Eclipse IDE</b></summary>

If you opened the Project Creator tool from the included Eclipse IDE, the project will open in Eclipse automatically.

</details>

<details><summary><b>Visual Studio (VS) Code</b></summary>

Launch VS Code manually, and then open the generated *{project-name}.code-workspace* file located in the project directory.

</details>

## Operation

1. Connect the board to your PC using the provided USB cable through the KitProg3 USB connector.

2. Open a terminal program and select the KitProg3 COM port. Set the serial port parameters to 8N1 and 115200 baud.

3. Program the board using one of the following:

   <details><summary><b>Using Eclipse IDE</b></summary>

      1. Select the application project in the Project Explorer.

      2. In the **Quick Panel**, scroll down, and click **\<Application Name> Program (KitProg3_MiniProg4)**.
   </details>

   <details><summary><b>Using CLI</b></summary>

     From the terminal, execute the `make program` command to build and program the application using the default toolchain to the default target:
      ```
      make program TOOLCHAIN=GCC_ARM
      ```
   </details>

4. After programming, the application starts automatically. Confirm that the sine wave inference results are displayed on the UART terminal, and the LED brightness varies smoothly, mimicking a sine wave.

## Design and implementation

This application runs a pre-trained TensorFlow Lite for Microcontrollers model that predicts the sine of an input value `x`. The `x` value is derived from a timer, creating a time-varying input. The model's prediction `y` (i.e., `sin(x)`) is then used to modulate the duty cycle of a PWM signal connected to an LED, causing its brightness to vary according to the sine wave.

### Resources and settings

**Table 1. Application resources**

 Resource  |  Alias/object     |    Purpose
 :-------- | :-------------    | :------------
 UART (HAL)| `cy_retarget_io_uart_obj` | UART HAL object used by Retarget-IO for debug/console.
 GPIO (HAL)    | `CYBSP_USER_LED`     | User LED for PWM output.
 PWM (HAL)     | `pwm_led_obj`        | PWM object for controlling LED brightness.
 Timer (HAL)   | `timer_obj`          | Timer for generating microsecond timestamps for sine input.
 TFLM Model    | `sine_model_tflite` (in `sine_model.h`) | Neural network model for sine prediction.

## Document history

Document title: *PSoC 6 TensorFlow Lite Sine Wave Demo*

 Version | Description of change
 ------- | ---------------------
 1.0.0   | Initial version for the TFLM sine wave demo.

 # Changes done to main.c to get main.cpp along with reason

## 1. File Extension and Language
-   **Change**: Rename `main.c` to `main.cpp`.
-   **Reason**: Switching to C++ is a crucial first step. It allows us to leverage C++ specific features such as namespaces (which TensorFlow Lite Micro uses extensively to organize its code and prevent naming collisions), classes (the TensorFlow Lite Micro library is C++ based, and we'll be interacting with its classes like `MicroInterpreter`), and easier integration with other C++ libraries. This change aligns our project with the language used by the TensorFlow Lite Micro framework itself.

---

## 2. Add Core Peripheral and TensorFlow Lite Micro Includes
**Add these lines at the top of your `main.cpp` file:**
```cpp
// Core PSoC 6 and Board Support Package Includes
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"

// TensorFlow Lite Micro Library Includes
#include "tensorflow/lite/c/common.h"
#include "tensorflow/lite/core/c/common.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"

// Model Specific Include
#include "models/sine_model.h"
```
**Reasoning for each include:**
-   `"cyhal.h"`: **Cypress Hardware Abstraction Layer.** This header provides the APIs to interact with the PSoC 6 microcontroller's peripherals (like GPIOs for LEDs, UART for communication, Timers for timing events, and PWM for LED brightness control) in a standardized way, abstracting away low-level register details.
-   `"cybsp.h"`: **Cypress Board Support Package.** This initializes your specific development board, configuring system clocks, relevant pins, and other hardware components essential for the board to function correctly before your application code runs.
-   `"cy_retarget_io.h"`: **Retarget I/O.** This utility is vital for debugging. It redirects standard input/output streams (like `printf`) to a physical interface, typically the UART, allowing you to send messages and diagnostic information to a serial terminal on your computer.
-   `"tensorflow/lite/c/common.h"`: **TensorFlow Lite C Common Headers.** Defines fundamental C data structures, enumerations (like `TfLiteStatus` for error checking), and functions that are used across the TensorFlow Lite Micro library.
-   `"tensorflow/lite/core/c/common.h"`: **TensorFlow Lite Core C Common Headers.** Provides additional core definitions and types for the TensorFlow Lite framework, ensuring robust interaction between different parts of the TFLM runtime.
-   `"tensorflow/lite/schema/schema_generated.h"`: **TensorFlow Lite Model Schema.** Contains C++ classes generated from the TensorFlow Lite model schema (which uses FlatBuffers). This is crucial for the interpreter to understand the structure and components of the `.tflite` model file you'll be using.
-   `"tensorflow/lite/micro/micro_interpreter.h"`: **TFLM Micro Interpreter.** Declares the `MicroInterpreter` class. This class is the heart of TFLM; it's responsible for loading your neural network model, allocating memory for its tensors (inputs, outputs, intermediate layers), and executing the inference.
-   `"tensorflow/lite/micro/micro_mutable_op_resolver.h"`: **TFLM Micro Mutable Op Resolver.** Declares the `MicroMutableOpResolver` class. This component allows you to register only the specific TensorFlow Lite operations (also known as kernels, like 'fully_connected', 'relu', etc.) that your particular model requires. This is key for microcontrollers as it helps minimize the final binary code size by excluding unused operations.
-   `"models/sine_model.h"`: **Custom Model Header.** This header file contains your actual sine wave neural network model data. The model, converted from a `.tflite` file (e.g., using `xxd`), is typically stored as a C character array representing the FlatBuffer data. This is how the compiled model is embedded directly into your application firmware.

---

## 3. Add C++ Standard Library Headers
**Add these C++ standard library includes:**
```cpp
#include <cstdarg>
#include <cstdio>
```
**Reasoning for each include:**
-   `<cstdarg>`: **C Standard Arguments.** This header is part of the C++ standard library and provides facilities for creating and working with functions that accept a variable number of arguments (variadic functions). This is used, for example, by `printf`-like functions, including the `MicroPrintf` utility we'll implement for TensorFlow Lite Micro logging.
-   `<cstdio>`: **C Standard Input/Output.** This header provides C-style input and output operations. We include it primarily for functions like `printf` (for general output to the console via retargeted I/O) and `vsnprintf` (used within `MicroPrintf` to format strings safely into a buffer).

---

## 4. Define Sine Wave, PWM, and Timer Macros
**Add these preprocessor definitions (macros):**
```cpp
// Debug flag - set to 1 for debug output, 0 to disable
#define DEBUG 1

// Some settings for our sine wave model
#define PI 3.14159265f
#define SINE_FREQ 0.5f
#define SINE_PERIOD ((1.0f / SINE_FREQ) * (1000000.0f))

// PWM LED configuration
#define PWM_FREQUENCY 1000000u // This is for configuring the PWM peripheral clock
#define PWM_PERIOD 255         // Used if PWM duty cycle is set by raw period counts

// LED blink timer clock value in Hz (from original main.c)
#define LED_BLINK_TIMER_CLOCK_HZ (10000)
// LED blink timer period value (from original main.c)
#define LED_BLINK_TIMER_PERIOD (9999)
```
**Reasoning for each define:**
-   `DEBUG 1`: This preprocessor macro acts as a compile-time switch. When `DEBUG` is `1`, conditional code blocks (e.g., `#if DEBUG ... #endif`) containing diagnostic `printf` statements or other debugging aids will be compiled into the program. Setting it to `0` will exclude this debug code, reducing binary size and potentially improving performance for a release version.
-   `PI 3.14159265f`: Defines the mathematical constant Pi (π) as a single-precision floating-point number. This is essential for trigonometric calculations, specifically for converting a time-based input into an angle in radians, which serves as the input to our sine wave model.
-   `SINE_FREQ 0.5f`: Defines the target frequency of the sine wave we want our model to predict, set to 0.5 Hertz (meaning one complete cycle every 2 seconds). This parameter dictates how rapidly the output sine wave will oscillate.
-   `SINE_PERIOD ((1.0f / SINE_FREQ) * (1000000.0f))`: Calculates and defines the period of the sine wave in microseconds. The formula `(1.0f / SINE_FREQ)` gives the period in seconds; this is then multiplied by `1,000,000.0f` to convert it to microseconds. This value is used in conjunction with a timer to generate the `x_val` input for the model, ensuring it covers one full sine wave cycle over this duration.
-   `PWM_FREQUENCY 1000000u`: Sets the desired clock frequency for the Pulse Width Modulation (PWM) peripheral to 1 MHz (1,000,000 Hertz). This high frequency is chosen for controlling LED brightness to ensure that any flickering is far too rapid to be perceived by the human eye, resulting in a smooth dimming effect. This is typically used when initializing the PWM peripheral itself.
-   `PWM_PERIOD 255`: Defines a period value, often used when PWM resolution is 8-bit (0-255). If the `cyhal_pwm_set_duty_cycle` function were to take raw counts for duty cycle based on this period, this macro would be directly used. However, the example uses a percentage-based approach for setting the duty cycle.
-   `LED_BLINK_TIMER_CLOCK_HZ (10000)`: Specifies the clock frequency (10,000 Hz) for the `led_blink_timer`. This timer and its configuration are retained from the original `main.c` example, primarily to keep the periodic timer interrupt (`isr_timer`) functional.
-   `LED_BLINK_TIMER_PERIOD (9999)`: Defines the period for the `led_blink_timer`. With a 10 kHz clock source, setting the period to 9999 (which means the timer counts from 0 to 9999, effectively 10,000 counts) results in the timer interrupt firing every 1 second (`(9999 + 1) / 10000 = 1s`).

---

## 5. Declare TensorFlow Lite Micro Globals
**Add these global variables, encapsulated in an anonymous namespace:**
```cpp
namespace {
    // Pointer to the TFLM model structure.
    // It will hold the actual model data loaded from the sine_model_tflite array.
    const tflite::Model* model = nullptr;

    // Pointer to the TFLM interpreter object.
    // This object is responsible for running the model inference.
    tflite::MicroInterpreter* interpreter = nullptr;

    // Pointer to the model's input tensor.
    // Used to feed data (the x_val) into the model.
    TfLiteTensor* model_input = nullptr;

    // Pointer to the model's output tensor.
    // Used to retrieve the prediction (the y_val) from the model.
    TfLiteTensor* model_output = nullptr;

    // Defines the size of the memory arena (10KB here).
    // `constexpr` ensures this is a compile-time constant.
    // This arena is used by TFLM for all runtime memory needs.
    constexpr int kTensorArenaSize = 10 * 1024;

    // The actual static memory buffer (byte array).
    // TFLM uses this for tensor data, activations, and scratch buffers,
    // avoiding dynamic memory allocation (heap) which is often problematic
    // on microcontrollers. The size must be sufficient for the model.
    uint8_t tensor_arena[kTensorArenaSize];
} // namespace
```
-   **Reason**: These global variables are fundamental for TensorFlow Lite Micro.
    -   The **anonymous namespace** (`namespace {}`) limits the scope of these variables to the current file (`main.cpp`), preventing potential naming conflicts with global variables in other files if your project grows larger.

---

## 6. Implement MicroPrintf Utility
**Add this utility function for TFLM logging:**
```cpp
// TensorFlow Lite Micro utility function for printing debug messages.
// TFLM can be configured to call this function for its internal logging.
void MicroPrintf(const char* format, ...) {
    // A reasonably sized local buffer to format the string into.
    char print_buf[256];

    // Standard C variadic arguments handling:
    // Declare a va_list to hold the variable arguments.
    va_list args;
    // Initialize `args` to point to the first variable argument
    // (the one after `format`).
    va_start(args, format);
    // Safely format the string. `vsnprintf` takes the buffer, its size,
    // the format string, and the va_list. It prevents buffer overflows
    // by not writing more characters than the buffer size allows.
    vsnprintf(print_buf, sizeof(print_buf), format, args);
    // Clean up the va_list.
    va_end(args);

    // Route the formatted string to the standard printf, which is (or should be)
    // retargeted to a UART or other debug output on the PSoC 6.
    printf("%s", print_buf);
}
```
-   **Reason**: TensorFlow Lite Micro often uses a function like `MicroPrintf` for its internal logging. By providing our own implementation, we can route these logs to our desired output (e.g., the UART via `printf`). This function takes a format string and variable arguments, formats them into a temporary buffer, and then prints the buffer. This is crucial for debugging and understanding the TFLM interpreter's state or errors.

---

## 7. Add PWM and Microsecond Timer Objects
**Declare these global HAL objects for hardware interaction:**
```cpp
// This cyhal_pwm_t object will represent the PWM peripheral instance
// configured to control the brightness of the user LED (CYBSP_USER_LED).
// We'll initialize and use this object to vary the LED's duty cycle
// based on the neural network model's output.
cyhal_pwm_t pwm_led_obj;

// This cyhal_timer_t object will be configured as a high-resolution timer.
// Its purpose is to provide precise microsecond-level timing, which is
// essential for generating the time-varying input (x_val) to the sine
// wave model and for measuring inference duration if DEBUG is enabled.
cyhal_timer_t timer_obj;
```
-   **Reason**: These objects are handles to the PSoC 6 hardware peripherals, managed by the Cypress Hardware Abstraction Layer (HAL).

---

## 8. Add ML and Hardware Initialization Function Prototypes
**Add these function prototypes (forward declarations) after includes and globals:**
```cpp
// Prototype for the function (from original main.c) that initializes
// the 1-second blink timer and its interrupt.
void timer_init(void);

// Prototype for the Interrupt Service Routine (ISR) (from original main.c)
// for the 1-second blink timer. `static` limits its linkage to this file.
static void isr_timer(void *callback_arg, cyhal_timer_event_t event);

// Prototype for our new function that will initialize all
// TensorFlow Lite Micro components (model, interpreter, tensors).
void ml_init(void);

// Prototype for our new function that will initialize the PWM
// peripheral for controlling the LED brightness.
void pwm_init(void);

// Prototype for our new function that will initialize the
// high-resolution timer for microsecond-level timing.
void micro_timer_init(void);

// Prototype for a helper function to read the current value
// from the microsecond timer. `static` limits its linkage to this file.
static uint32_t get_time_us(void);
```
-   **Reason**: These forward declarations inform the compiler about the existence and signature (return type and parameters) of these functions before they are fully defined later in the file. This allows functions (like `main`) to call them even if their definitions appear later in the source code.

---

## 9. Use C++ Linkage for DebugLog
**Add this function, ensuring C linkage for TFLM compatibility:**
```cpp
// extern "C" tells the C++ compiler to use C linkage for this function.
// This prevents C++ name mangling, ensuring that if TFLM expects to call
// a C function named "DebugLog", it can find this implementation.
extern "C" void DebugLog(const char* s) {
    // The function body simply prints the input string `s`
    // using the retargeted printf.
    printf("%s", s);
}
```
-   **Reason**: TensorFlow Lite Micro's internal logging mechanisms might expect a C-style function named `DebugLog`. The `extern "C"` linkage specification is crucial here. C++ compilers perform "name mangling" (modifying function names to include information about their parameters for overloading), while C compilers do not. `extern "C"` ensures that `DebugLog` has a simple, unmangled C-style name in the compiled object code, making it callable from C code or from C++ code that expects C linkage.

---

## 10. Update Main Function and Loop for Inference
**The `main()` function is where the primary application logic resides. It initializes hardware and then enters an infinite loop for continuous operation.**
```cpp
int main(void)
{
    cy_rslt_t result; // Variable to store HAL function results for error checking.

#if defined (CY_DEVICE_SECURE)
    // This block is specific to PSoC 6 devices with security features.
    // It initializes and then frees the watchdog timer (WDT) to prevent
    // an unintended device reset during startup or debugging.
    cyhal_wdt_t wdt_obj;
    result = cyhal_wdt_init(&wdt_obj, cyhal_wdt_get_max_timeout_ms());
    CY_ASSERT(CY_RSLT_SUCCESS == result); // Assert if WDT init fails.
    cyhal_wdt_free(&wdt_obj);
#endif

    // Initialize the device and board peripherals.
    // This typically sets up system clocks, and configures pins according
    // to the board's design (e.g., for UART, LEDs).
    result = cybsp_init();
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0); // If board init fails, halt execution.
    }

    // Enable global interrupts. This allows interrupt service routines (ISRs)
    // like `isr_timer` to be executed when their corresponding hardware
    // events occur.
    __enable_irq();

    // Initialize retarget-io to use the debug UART port.
    // This redirects printf/scanf to the UART, allowing communication
    // with a serial terminal for debugging and output.
    // CYBSP_DEBUG_UART_TX, RX, CTS, RTS are board-specific pins.
    result = cy_retarget_io_init_fc(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
                                    CYBSP_DEBUG_UART_CTS, CYBSP_DEBUG_UART_RTS,
                                    CY_RETARGET_IO_BAUDRATE);
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0); // If retarget-io init fails, halt.
    }

    // The original main.c initialized CYBSP_USER_LED GPIO here.
    // In main.cpp, this direct GPIO initialization is effectively handled
    // within pwm_init() which configures the same pin for PWM output.

    // ANSI escape sequence to clear the terminal screen and move cursor to home.
    printf("\x1b[2J\x1b[;H");

    printf("**************************************************************\r\n");
    printf("*   PSoC 6 TensorFlow Lite Sine Wave Model Demo              *\r\n");
    printf("**************************************************************\r\n\n");

    // Initialize PWM for LED brightness control.
    pwm_init();
    
    // Initialize a timer for microsecond-resolution timing.
    micro_timer_init();
    
    // Initialize the ML model, interpreter, and tensors.
    ml_init();
    
    // Initialize the 1-second timer for periodic interrupts (kept from original example).
    // This also starts the timer.
    timer_init(); 
    
    printf("Running sine wave inference...\r\n\n");
    printf("Press 'Enter' key to pause or resume the demo\r\n\n");

    // Infinite loop for continuous operation.
    for (;;)
    {
        // Check if the 'Enter' key was pressed via UART.
        // cyhal_uart_getc attempts to read 1 byte with a timeout of 1ms (non-blocking).
        if (cyhal_uart_getc(&cy_retarget_io_uart_obj, &uart_read_value, 1)
             == CY_RSLT_SUCCESS)
        {
            // If a character was successfully read:
            if (uart_read_value == '\r') // Check if it's the Enter key (carriage return).
            {
                // Toggle the demo's active state.
                if (led_blink_active_flag)
                {
                    printf("Demo paused \r\n");
                }
                else
                {
                    printf("Demo resumed\r\n");
                }
                // ANSI escape sequence to move the cursor up one line.
                // This is to overwrite the "Demo paused/resumed" message line
                // on the next print, keeping the console cleaner.
                printf("\x1b[1F"); 
                led_blink_active_flag ^= 1; // Toggle the flag.
            }
        }
        
        // Only run inference and LED control if the demo is active.
        if (led_blink_active_flag)
        {
            // Generate input for the sine model:
            // 1. Get current time in microseconds from our high-resolution timer.
            // 2. Modulo with SINE_PERIOD to create a repeating cycle over the defined period.
            uint32_t timestamp = get_time_us() % (uint32_t)SINE_PERIOD;
            
            // 3. Convert the timestamp (which is now in [0, SINE_PERIOD-1])
            //    into an angle in radians from 0 to 2*PI. This is the x_val.
            float x_val = ((float)timestamp * 2.0f * PI) / SINE_PERIOD;
            
            #if DEBUG
            // If DEBUG is enabled, record the start time for inference profiling.
            uint32_t start_time_us = get_time_us();
            #endif
            
            // Feed the input value (x_val) to the model's input tensor.
            // Assumes the model has one float input.
            model_input->data.f[0] = x_val;
            
            // Run model inference.
            TfLiteStatus invoke_status = interpreter->Invoke();
            
            // Check if inference was successful.
            if (invoke_status != kTfLiteOk) {
                MicroPrintf("Invoke failed on input: %f\r\n", (double)x_val);
            }
            
            // Retrieve the prediction from the model's output tensor.
            // Assumes the model has one float output (the sine value).
            float y_val = model_output->data.f[0];
            
            // Translate the model's output (y_val, typically in [-1, 1])
            // to a PWM duty cycle percentage (0% to 100%).
            // (y_val + 1.0f) maps the range to [0, 2].
            // Multiplying by 50.0f maps it to [0, 100].
            float duty_cycle_percent = (y_val + 1.0f) * 50.0f;
            
            // Set the PWM duty cycle to control LED brightness.
            // The `100.0f` here is a reference value, indicating that
            // `duty_cycle_percent` is indeed a percentage from 0 to 100.
            cyhal_pwm_set_duty_cycle(&pwm_led_obj, duty_cycle_percent, 100.0f);
            
            #if DEBUG
            // If DEBUG is enabled, print input, output, duty cycle, and inference time.
            // Note: Using MicroPrintf for consistency if TFLM logs also use it.
            // Casting floats to double for printf is a common practice but check your printf capabilities.
            MicroPrintf("Input: %.3f, Output: %.3f, Duty: %.1f%%, Timestamp: %lu us, Inference: %lu us\r\n",
                        (double)x_val, (double)y_val, (double)duty_cycle_percent, timestamp, (get_time_us() - start_time_us));
            #endif
            
            // Small delay to make LED changes observable and to avoid flooding the console.
            cyhal_system_delay_ms(100);
        }
        
        // Check if the 1-second timer interrupt has occurred.
        if (timer_interrupt_flag)
        {
            // Clear the flag.
            timer_interrupt_flag = false;
            
            // The original main.c toggled an LED here.
            // In this main.cpp, the CYBSP_USER_LED is controlled by PWM based on
            // the model output. This interrupt flag is still processed but
            // doesn't directly control the LED in the same way. It could be
            // used for other periodic 1-second tasks if needed.
        }
    }
}
```

---

## 11. Add New Initialization Function Implementations
**Implement the following functions to set up TensorFlow Lite Micro, PWM, and the microsecond timer.**

### `ml_init()` - TensorFlow Lite Micro Initialization
```cpp
void ml_init(void)
{
    printf("Initializing TensorFlow Lite Micro...\r\n");
    
    // 1. Get a pointer to the TFLM model data.
    // `sine_model_tflite` is the byte array defined in `models/sine_model.h`.
    model = tflite::GetModel(sine_model_tflite);

    // 2. Validate the model's schema version.
    // This ensures the model format is compatible with the TFLM library version.
    if (model->version() != TFLITE_SCHEMA_VERSION) {
        MicroPrintf("Model version does not match Schema version\r\n");
        CY_ASSERT(0); // Halt if versions mismatch.
    }
    
    // 3. Create an operator resolver.
    // This resolver will map model operations (like "fully_connected") to their
    // actual C++ kernel implementations. We use MicroMutableOpResolver to add
    // only the operations our specific model needs, saving memory.
    // The template argument `<4>` indicates the maximum number of ops we expect to register.
    static tflite::MicroMutableOpResolver<4> resolver;

    // 4. Register the operations used by the sine_model.
    // These must match the operations present in your trained model.
    resolver.AddFullyConnected(); // For dense layers.
    resolver.AddRelu();           // For ReLU activation functions.
    resolver.AddLogistic();       // For Logistic/Sigmoid activation (if used).
    resolver.AddMul();            // For element-wise multiplication (if used).
    // Check your model architecture to confirm these are the correct ops.

    // 5. Build an interpreter to run the model.
    // The interpreter takes the model, resolver, tensor arena, and arena size.
    // `static_interpreter` is declared static so it persists throughout the
    // application's lifetime, as `interpreter` will point to it.
    static tflite::MicroInterpreter static_interpreter(
        model, resolver, tensor_arena, kTensorArenaSize);
    interpreter = &static_interpreter; // Assign to our global pointer.
    
    // 6. Allocate memory from the `tensor_arena` for the model's tensors.
    // This is a critical step; TFLM calculates memory needs and allocates
    // from the provided arena.
    TfLiteStatus allocate_status = interpreter->AllocateTensors();
    if (allocate_status != kTfLiteOk) {
        MicroPrintf("AllocateTensors() failed\r\n");
        CY_ASSERT(0); // Halt if allocation fails (arena might be too small).
    }
    
    // 7. Obtain pointers to the model's input and output tensors.
    // These pointers allow us to write data to the input tensor and read
    // data from the output tensor. Index 0 is used for single input/output models.
    model_input = interpreter->input(0);
    model_output = interpreter->output(0);
    
    #if DEBUG
    // If DEBUG is enabled, print details about the model's tensors.
    // This is useful for verifying the model loaded correctly.
    MicroPrintf("Model loaded successfully\r\n");
    MicroPrintf("Input tensor dimensions: %d\r\n", model_input->dims->size);
    MicroPrintf("Input tensor shape: [%d", model_input->dims->data[0]);
    for (int i = 1; i < model_input->dims->size; i++) {
        MicroPrintf(", %d", model_input->dims->data[i]);
    }
    MicroPrintf("]\r\n");
    MicroPrintf("Input tensor type: %d (Note: Check TfLiteType enum, e.g., kTfLiteFloat32 is 1)\r\n", model_input->type);
    
    MicroPrintf("Output tensor dimensions: %d\r\n", model_output->dims->size);
    MicroPrintf("Output tensor shape: [%d", model_output->dims->data[0]);
    for (int i = 1; i < model_output->dims->size; i++) {
        MicroPrintf(", %d", model_output->dims->data[i]);
    }
    MicroPrintf("]\r\n");
    MicroPrintf("Output tensor type: %d\r\n", model_output->type);
    #endif
    
    printf("TensorFlow Lite Micro initialization complete\r\n\n");
}
```

### `pwm_init()` - PWM Initialization for LED Control
```cpp
void pwm_init(void)
{
    cy_rslt_t result; // For storing HAL function results.
    
    printf("Initializing PWM for LED control...\r\n");

    // 1. Initialize PWM on the USER_LED pin (CYBSP_USER_LED).
    // `&pwm_led_obj` is our global PWM object.
    // `CYBSP_USER_LED` is the board-specific pin for the user LED.
    // `NULL` for the clock argument means the HAL will use a suitable clock.
    result = cyhal_pwm_init(&pwm_led_obj, CYBSP_USER_LED, NULL);
    if (result != CY_RSLT_SUCCESS) {
        MicroPrintf("PWM initialization failed with error code: %lu\r\n", result);
        CY_ASSERT(0); // Halt on failure.
    }
    
    // 2. Set initial PWM duty cycle to 0% (LED off).
    // The `100.0f` as the third argument indicates that the second argument (0)
    // is a percentage value (0 out of 100).
    result = cyhal_pwm_set_duty_cycle(&pwm_led_obj, 0.0f, 100.0f);
    if (result != CY_RSLT_SUCCESS) {
        MicroPrintf("PWM set duty cycle failed with error code: %lu\r\n", result);
        CY_ASSERT(0);
    }

    // 3. Configure the PWM frequency (optional if already set by init or default is fine).
    // This step is often part of cyhal_pwm_init or a separate frequency setting function.
    // The example main.cpp doesn't explicitly call set_frequency here after init,
    // but it defines PWM_FREQUENCY. If needed:
    // result = cyhal_pwm_set_frequency(&pwm_led_obj, PWM_FREQUENCY);
    // if (result != CY_RSLT_SUCCESS) { /* ... error handling ... */ }

    // 4. Start the PWM peripheral.
    // This enables the PWM output on the configured pin.
    result = cyhal_pwm_start(&pwm_led_obj);
    if (result != CY_RSLT_SUCCESS) {
        MicroPrintf("PWM start failed with error code: %lu\r\n", result);
        CY_ASSERT(0);
    }
    
    printf("PWM initialization complete\r\n");
}
```

### `micro_timer_init()` - Microsecond Timer Initialization
```cpp
void micro_timer_init(void)
{
    cy_rslt_t result; // For storing HAL function results.

    printf("Initializing microsecond timer...\r\n");
    
    // 1. Define the timer configuration structure.
    const cyhal_timer_cfg_t timer_cfg =
    {
        .is_continuous = true,          // Timer runs continuously after starting.
        .direction = CYHAL_TIMER_DIR_UP,// Timer counts upwards.
        .is_compare = false,            // Compare mode not used for this timer.
        .period = 0xFFFFFFFF,           // Set to maximum period for a 32-bit counter.
        .compare_value = 0,             // Compare value not used.
        .value = 0                      // Initial counter value.
    };
    
    // 2. Initialize the timer object.
    // `&timer_obj` is our global timer object.
    // `NC` (Not Connected) for the pin argument as this timer is not tied to a GPIO.
    // `NULL` for the clock argument, HAL will select a suitable clock.
    result = cyhal_timer_init(&timer_obj, NC, NULL);
    if (result != CY_RSLT_SUCCESS) {
        MicroPrintf("Microsecond Timer initialization failed: %lu\r\n", result);
        CY_ASSERT(0);
    }
    
    // 3. Configure the timer with the defined settings.
    result = cyhal_timer_configure(&timer_obj, &timer_cfg);
    if (result != CY_RSLT_SUCCESS) {
        MicroPrintf("Microsecond Timer configuration failed: %lu\r\n", result);
        CY_ASSERT(0);
    }
    
    // 4. Set the timer frequency to 1MHz.
    // This makes each timer tick correspond to 1 microsecond.
    result = cyhal_timer_set_frequency(&timer_obj, 1000000);
    if (result != CY_RSLT_SUCCESS) {
        MicroPrintf("Microsecond Timer set frequency failed: %lu\r\n", result);
        CY_ASSERT(0);
    }
    
    // 5. Start the timer.
    result = cyhal_timer_start(&timer_obj);
    if (result != CY_RSLT_SUCCESS) {
        MicroPrintf("Microsecond Timer start failed: %lu\r\n", result);
        CY_ASSERT(0);
    }
    
    printf("Microsecond timer initialization complete\r\n");
}
```

### `get_time_us()` - Get Current Time in Microseconds
```cpp
// Static helper function to read the current value of the microsecond timer.
// `static` limits its scope to this file.
static uint32_t get_time_us(void)
{
    // `cyhal_timer_read` returns the current counter value of the specified timer.
    // Since we configured the timer to tick every microsecond, this value
    // directly represents the time in microseconds (modulo the timer's period).
    return cyhal_timer_read(&timer_obj);
}
```

---

## 12. Retain and Adapt Timer Interrupt Logic (from original `main.c`)
**These functions set up a 1-second periodic interrupt, largely kept for compatibility or other potential periodic tasks.**

### `timer_init()` - 1-Second Blink Timer Initialization
```cpp
 void timer_init(void)
 {
    cy_rslt_t result; // For HAL function results.

    // 1. Define configuration for the LED blink timer (1-second interrupt).
    const cyhal_timer_cfg_t led_blink_timer_cfg =
    {
        .is_continuous = true,              // Run timer indefinitely
        .direction = CYHAL_TIMER_DIR_UP,    // Timer counts up 
        .is_compare = false,                // Don't use compare mode 
        .period = LED_BLINK_TIMER_PERIOD,   // Defines the timer period
        .compare_value = 0,                 // Timer compare value, not used
        .value = 0                          // Initial value of counter
    };

    // 2. Initialize the timer object (`led_blink_timer` is a global from main.c, ensure it's declared).
    // Assuming `led_blink_timer` is declared globally like `cyhal_timer_t led_blink_timer;`
    result = cyhal_timer_init(&led_blink_timer, NC, NULL);
    if (result != CY_RSLT_SUCCESS)
    {
        MicroPrintf("1s Timer init failed: %lu\r\n", result);
        CY_ASSERT(0);
    }

    // 3. Configure timer period and operation mode.
    cyhal_timer_configure(&led_blink_timer, &led_blink_timer_cfg);

    // 4. Set the frequency of timer's clock source.
    // With LED_BLINK_TIMER_CLOCK_HZ = 10000 and period = 9999,
    // interrupt occurs every (9999+1)/10000 = 1 second.
    cyhal_timer_set_frequency(&led_blink_timer, LED_BLINK_TIMER_CLOCK_HZ);

    // 5. Assign the ISR (`isr_timer`) to execute on timer interrupt.
    cyhal_timer_register_callback(&led_blink_timer, isr_timer, NULL);

    // 6. Set the event on which timer interrupt occurs (terminal count) and enable it.
    // Priority 7 is a common default; adjust if needed for your interrupt scheme.
    cyhal_timer_enable_event(&led_blink_timer, CYHAL_TIMER_IRQ_TERMINAL_COUNT,
                              7, true);

    // 7. Start the timer with the configured settings.
    cyhal_timer_start(&led_blink_timer);
 }
```
*(Note: Ensure `led_blink_timer` and `timer_interrupt_flag` are declared globally as they were in `main.c` if you port this logic directly. The provided `main.cpp` attachment uses these.)*

### `isr_timer()` - Interrupt Service Routine for 1-Second Timer
```cpp
// This is the interrupt handler function for the 1-second timer interrupt.
// `static` limits its scope to this file.
static void isr_timer(void *callback_arg, cyhal_timer_event_t event)
{
    // Suppress unused parameter warnings.
    (void) callback_arg;
    (void) event;

    // Set the global flag. This flag will be checked in the main loop
    // to indicate that 1 second has elapsed.
    timer_interrupt_flag = true;
}
```
-   **Reason**: This timer and ISR are retained from the original `main.c`. While the primary LED control is now via PWM driven by the ML model, this 1-second interrupt mechanism can be useful for other periodic tasks or simply to show the evolution from the simpler blinky example. If it has no other purpose, it could be removed to save resources.

---

## Summary
By following these detailed steps, you've successfully upgraded the basic `main.c` LED blinker to a sophisticated `main.cpp` application. This new version runs a TensorFlow Lite Micro neural network model in real-time on your PSoC 6, predicting a sine wave and visualizing its output by varying LED brightness using PWM. This project serves as an excellent foundation for more complex TinyML applications, demonstrating model inference, hardware interaction, and real-time processing on microcontrollers.

