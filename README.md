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
