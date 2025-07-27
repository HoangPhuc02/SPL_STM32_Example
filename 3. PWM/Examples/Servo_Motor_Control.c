/*
 * =============================================================================
 * Project: STM32F103 PWM Examples
 * File: Servo_Motor_Control.c
 * Description: PWM example for controlling servo motors using TIM2
 * Author: PWM Driver Team
 * Date: July 2025
 * =============================================================================
 * 
 * OVERVIEW:
 * This example demonstrates PWM generation for servo motor control.
 * Servo motors require specific PWM parameters:
 * - Frequency: 50Hz (20ms period)
 * - Pulse width: 1ms to 2ms (for 0° to 180° rotation)
 * - Signal: 5V logic levels
 * 
 * HARDWARE SETUP:
 * - Connect servo signal wire to PA0 (TIM2_CH1)
 * - Connect servo VCC to 5V power supply
 * - Connect servo GND to common ground
 * - Ensure STM32 and servo share common ground
 * 
 * SERVO SPECIFICATIONS:
 * - Standard servo: 1ms = 0°, 1.5ms = 90°, 2ms = 180°
 * - Frequency: 50Hz (20ms period)
 * - Pulse width range: 1ms - 2ms
 * 
 * =============================================================================
 */

#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"

/* =============================================================================
 * SERVO PWM CONFIGURATION
 * =============================================================================
 */
#define SERVO_FREQUENCY     50      // 50Hz for servo control
#define TIMER_CLOCK         72000000 // 72MHz APB1 timer clock
#define PRESCALER           72      // Prescaler: 72MHz/72 = 1MHz timer clock
#define PERIOD              20000   // 20ms period: 1MHz/20000 = 50Hz

/* Servo pulse width definitions (in microseconds and timer ticks) */
#define PULSE_WIDTH_MIN_US  1000    // 1ms = 1000µs (0 degrees)
#define PULSE_WIDTH_MID_US  1500    // 1.5ms = 1500µs (90 degrees)  
#define PULSE_WIDTH_MAX_US  2000    // 2ms = 2000µs (180 degrees)

/* Convert microseconds to timer ticks (1MHz timer = 1µs per tick) */
#define PULSE_WIDTH_MIN     PULSE_WIDTH_MIN_US  // 1000 ticks
#define PULSE_WIDTH_MID     PULSE_WIDTH_MID_US  // 1500 ticks
#define PULSE_WIDTH_MAX     PULSE_WIDTH_MAX_US  // 2000 ticks

/* Servo angle to pulse width conversion */
#define SERVO_MIN_ANGLE     0       // Minimum angle (degrees)
#define SERVO_MAX_ANGLE     180     // Maximum angle (degrees)

/* Movement parameters */
#define ANGLE_STEP          5       // Degrees per step
#define MOVE_DELAY          50000   // Delay between movements

/* =============================================================================
 * FUNCTION PROTOTYPES
 * =============================================================================
 */
void Servo_GPIO_Config(void);
void Servo_Timer_Config(void);
void Servo_SetAngle(uint16_t angle);
uint16_t Servo_AngleToPulseWidth(uint16_t angle);
void Delay(uint32_t count);

/* =============================================================================
 * MAIN FUNCTION
 * =============================================================================
 */
int main(void)
{
    uint16_t current_angle = 90;    // Start at center position
    uint8_t sweep_direction = 1;    // 1 = forward, 0 = backward
    
    // Initialize servo PWM
    Servo_GPIO_Config();
    Servo_Timer_Config();
    
    // Move to center position initially
    Servo_SetAngle(90);
    Delay(1000000); // Wait 1 second for servo to reach position
    
    /* =================================================================
     * MAIN LOOP - SERVO SWEEP DEMONSTRATION
     * =================================================================
     * This loop demonstrates various servo control patterns:
     * 1. Continuous sweeping from 0° to 180° and back
     * 2. Step-by-step movement with position feedback
     * 3. Specific angle positioning
     */
    while(1)
    {
        /* =========================================================
         * SERVO SWEEP PATTERN
         * =========================================================
         * Continuously sweep servo from 0° to 180° and back
         */
        
        // Set servo to current angle
        Servo_SetAngle(current_angle);
        
        // Wait for servo to move
        Delay(MOVE_DELAY);
        
        // Update angle based on sweep direction
        if(sweep_direction == 1) // Moving toward 180°
        {
            current_angle += ANGLE_STEP;
            if(current_angle >= SERVO_MAX_ANGLE)
            {
                current_angle = SERVO_MAX_ANGLE;
                sweep_direction = 0; // Change direction
                Delay(500000); // Pause at end position
            }
        }
        else // Moving toward 0°
        {
            if(current_angle >= ANGLE_STEP)
            {
                current_angle -= ANGLE_STEP;
            }
            else
            {
                current_angle = SERVO_MIN_ANGLE;
                sweep_direction = 1; // Change direction
                Delay(500000); // Pause at end position
            }
        }
        
        /* Alternative control patterns (uncomment to use):
         
        // Pattern 1: Fixed positions
        Servo_SetAngle(0);    Delay(1000000);  // 0°
        Servo_SetAngle(45);   Delay(1000000);  // 45°
        Servo_SetAngle(90);   Delay(1000000);  // 90°
        Servo_SetAngle(135);  Delay(1000000);  // 135°
        Servo_SetAngle(180);  Delay(1000000);  // 180°
        
        // Pattern 2: Quick oscillation
        for(int i = 0; i < 10; i++) {
            Servo_SetAngle(45);  Delay(200000);
            Servo_SetAngle(135); Delay(200000);
        }
        */
    }
}

/* =============================================================================
 * SERVO GPIO CONFIGURATION
 * =============================================================================
 * Configures PA0 as alternate function for TIM2 Channel 1 PWM output.
 * PA0 is used because it's TIM2_CH1 and easy to access on most boards.
 */
void Servo_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // Enable GPIOA clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    
    /* Configure PA0 (TIM2_CH1) for servo PWM output */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;     // Alternate function push-pull
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;   // High speed for clean signals
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/* =============================================================================
 * SERVO TIMER CONFIGURATION
 * =============================================================================
 * Configures TIM2 for 50Hz PWM generation suitable for servo control.
 * 
 * Timer Configuration:
 * - Input Clock: 72MHz (APB1)
 * - Prescaler: 72 → Timer Clock: 1MHz (1µs resolution)
 * - Period: 20000 → PWM Frequency: 50Hz (20ms period)
 * - Resolution: 1µs (perfect for servo timing requirements)
 */
void Servo_Timer_Config(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    
    // Enable TIM2 clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    
    /* =================================================================
     * TIMER BASE CONFIGURATION FOR SERVO PWM
     * =================================================================
     * Calculation for 50Hz servo PWM:
     * - Desired frequency: 50Hz (20ms period)
     * - Timer resolution needed: 1µs (for precise servo timing)
     * - Prescaler: 72MHz / 72 = 1MHz (1µs per tick)
     * - Period: 1MHz / 50Hz = 20000 ticks = 20ms
     */
    TIM_TimeBaseStructure.TIM_Period = PERIOD - 1;        // 20000-1 = 19999
    TIM_TimeBaseStructure.TIM_Prescaler = PRESCALER - 1;  // 72-1 = 71
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    
    /* =================================================================
     * PWM OUTPUT COMPARE CONFIGURATION
     * =================================================================
     * PWM Mode 1: Output HIGH when counter < compare value
     * Compare value determines pulse width (servo position)
     */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;           // PWM Mode 1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = PULSE_WIDTH_MID;            // Start at center (90°)
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;   // Active high
    TIM_OC1Init(TIM2, &TIM_OCInitStructure);
    
    // Enable preload for smooth operation
    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM2, ENABLE);
    
    // Start timer
    TIM_Cmd(TIM2, ENABLE);
}

/* =============================================================================
 * SET SERVO ANGLE
 * =============================================================================
 * Sets servo to specified angle (0° to 180°).
 * 
 * Parameters:
 *   angle - Desired angle in degrees (0-180)
 *          0° = 1ms pulse width
 *          90° = 1.5ms pulse width  
 *          180° = 2ms pulse width
 * 
 * The function converts angle to appropriate pulse width and updates
 * the timer compare register to generate the correct PWM signal.
 */
void Servo_SetAngle(uint16_t angle)
{
    uint16_t pulse_width;
    
    // Limit angle to valid range
    if(angle > SERVO_MAX_ANGLE)
        angle = SERVO_MAX_ANGLE;
    
    // Convert angle to pulse width
    pulse_width = Servo_AngleToPulseWidth(angle);
    
    // Update timer compare register
    TIM_SetCompare1(TIM2, pulse_width);
}

/* =============================================================================
 * CONVERT ANGLE TO PULSE WIDTH
 * =============================================================================
 * Converts servo angle (0°-180°) to PWM pulse width (1000-2000 µs).
 * 
 * Linear interpolation formula:
 * pulse_width = min_pulse + (angle / max_angle) * (max_pulse - min_pulse)
 * 
 * Example calculations:
 * - 0°:   1000 + (0/180) * (2000-1000) = 1000µs = 1ms
 * - 90°:  1000 + (90/180) * (2000-1000) = 1500µs = 1.5ms  
 * - 180°: 1000 + (180/180) * (2000-1000) = 2000µs = 2ms
 */
uint16_t Servo_AngleToPulseWidth(uint16_t angle)
{
    uint32_t pulse_width;
    
    // Linear interpolation: pulse = min + (angle/180) * (max - min)
    pulse_width = PULSE_WIDTH_MIN + 
                  ((uint32_t)angle * (PULSE_WIDTH_MAX - PULSE_WIDTH_MIN)) / SERVO_MAX_ANGLE;
    
    return (uint16_t)pulse_width;
}

/* =============================================================================
 * SIMPLE DELAY FUNCTION
 * =============================================================================
 */
void Delay(uint32_t count)
{
    volatile uint32_t i;
    for(i = 0; i < count; i++)
    {
        // Empty loop
    }
}

/* =============================================================================
 * SERVO MOTOR THEORY AND TIMING
 * =============================================================================
 * 
 * SERVO MOTOR BASICS:
 * 
 * Servo motors are closed-loop control systems that use PWM signals for
 * position control. They contain:
 * - DC motor for movement
 * - Potentiometer for position feedback  
 * - Control circuit for position regulation
 * 
 * PWM SIGNAL REQUIREMENTS:
 * 
 * 1. FREQUENCY: 50Hz (20ms period)
 *    - Standard for most hobby servos
 *    - Some servos work with 50-333Hz range
 *    - Higher frequency = faster response, more power consumption
 * 
 * 2. PULSE WIDTH: 1ms to 2ms
 *    - 1.0ms = 0° (fully counter-clockwise)
 *    - 1.5ms = 90° (center position)
 *    - 2.0ms = 180° (fully clockwise)
 *    - Dead band: 0ms to 1ms and 2ms to 20ms (no movement)
 * 
 * 3. VOLTAGE LEVELS:
 *    - Logic level: 3.3V or 5V (STM32 outputs 3.3V - usually sufficient)
 *    - Power supply: 4.8V to 6V for motor operation
 * 
 * TIMING PRECISION:
 * 
 * Servo position accuracy depends on PWM timing precision:
 * - 1µs resolution gives ~0.18° accuracy (1000µs range / 180° = 5.56µs/°)
 * - Our 1MHz timer provides 1µs resolution - excellent for servos
 * - Jitter should be < 1µs for stable positioning
 * 
 * SERVO TYPES:
 * 
 * 1. STANDARD SERVO (0°-180°):
 *    - Most common type
 *    - 1ms-2ms pulse width range
 *    - 180° rotation limit
 * 
 * 2. CONTINUOUS ROTATION SERVO:
 *    - Modified for continuous rotation
 *    - 1.5ms = stop, <1.5ms = CCW, >1.5ms = CW
 *    - Speed control instead of position
 * 
 * 3. DIGITAL SERVO:
 *    - Higher update rates (up to 333Hz)
 *    - Better holding torque
 *    - More precise positioning
 * 
 * PRACTICAL CONSIDERATIONS:
 * 
 * 1. POWER SUPPLY:
 *    - Separate 5V supply for servo motor
 *    - Common ground between STM32 and servo
 *    - Adequate current capacity (servos can draw 1-2A)
 * 
 * 2. SIGNAL QUALITY:
 *    - Clean PWM signal (minimal jitter)
 *    - Proper signal levels (3.3V usually OK)
 *    - Short signal wires to reduce noise
 * 
 * 3. MECHANICAL LIMITS:
 *    - Don't force servo beyond mechanical stops
 *    - Allow time for servo to reach position
 *    - Consider servo speed ratings
 * 
 * 4. SOFTWARE CONSIDERATIONS:
 *    - Smooth angle transitions to avoid jerky movement
 *    - Appropriate delays between position changes
 *    - Error handling for invalid angles
 * 
 * DEBUGGING TIPS:
 * 
 * 1. Use oscilloscope to verify PWM timing
 * 2. Check power supply voltage and current
 * 3. Verify ground connections
 * 4. Test with known-good servo
 * 5. Start with center position (1.5ms) for testing
 * 
 * =============================================================================
 */
