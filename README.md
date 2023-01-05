# PID Motor Control for STM32

This library allows you to easily control motors using a PID control loop on an STM32 microcontroller.

## Features
- Functions for both forward and backward movement
- Stop function
- Modifiable Kp, Ki, and Kd constants for PID control

## Prerequisites
- TIM1, TIM2, TIM3, TIM5, and TIM14 handles initialized
- System clock configured
- GPIO initialized

## Usage
To control the motors, use the following functions:
- `pidr()` and `pidg()` for forward movement
- `pidr_arriere()` and `pidg_arriere()` for backward movement
- `stop()` to stop the motors

## Implementation Details
The PID control loop is implemented using the Kp, Ki, and Kd constants for each motor. The motor speeds and tick counts are updated using the `HAL_TIM_IC_CaptureCallback()` and `HAL_TIM_PeriodElapsedCallback()` functions.

