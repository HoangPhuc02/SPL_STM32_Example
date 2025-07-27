/*
 * =============================================================================
 * Project: STM32F103 PWM Examples
 * File: Motor_Speed_Control.c
 * Description: PWM example for DC motor speed control with direction
 * Author: PWM Driver Team
 * Date: July 2025
 * =============================================================================
 * 
 * OVERVIEW:
 * This example demonstrates PWM-based speed control for DC motors using
 * an H-bridge driver (like L298N). The system provides:
 * - Variable speed control (0% to 100%)
 * - Direction control (forward/reverse)
 * - Smooth acceleration and deceleration
 * - Motor braking functionality
 * 
 * HARDWARE SETUP:
 * - H-Bridge: L298N or similar
 * - Motor: 6V-12V DC motor
 * - Power: External power supply for motor
 * 
 * CONNECTIONS:
 * STM32 -> L298N H-Bridge:
 * - PA6 (TIM3_CH1) -> ENA (Enable A - PWM speed control)
 * - PA1 -> IN1 (Direction control bit 1)
 * - PA2 -> IN2 (Direction control bit 2)
 * - GND -> GND (Common ground)
 * 
 * L298N -> Motor:
 * - OUT1, OUT2 -> Motor terminals
 * - +12V -> External power supply positive
 * - GND -> External power supply negative (and STM32 GND)
 * 
 * =============================================================================
 */

#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"

/* =============================================================================
 * MOTOR CONTROL CONFIGURATION
 * =============================================================================
 */
#define PWM_FREQUENCY       1000    // 1kHz PWM frequency (good for motors)
#define TIMER_CLOCK         72000000 // 72MHz APB1 timer clock
#define PRESCALER           72      // Prescaler: 72MHz/72 = 1MHz
#define PERIOD              1000    // Period: 1MHz/1000 = 1kHz

/* Motor control pins */
#define MOTOR_PWM_PIN       GPIO_Pin_6  // PA6 (TIM3_CH1) - Speed control
#define MOTOR_DIR1_PIN      GPIO_Pin_1  // PA1 - Direction control 1
#define MOTOR_DIR2_PIN      GPIO_Pin_2  // PA2 - Direction control 2

/* Motor direction definitions */
typedef enum {
    MOTOR_STOP = 0,     // Motor stopped (brake)
    MOTOR_FORWARD,      // Motor forward
    MOTOR_REVERSE,      // Motor reverse
    MOTOR_COAST         // Motor coast (free running)
} MotorDirection_t;

/* Speed control parameters */
#define SPEED_MIN           0       // Minimum speed (0%)
#define SPEED_MAX           100     // Maximum speed (100%)
#define ACCELERATION_STEP   2       // Speed change per step (2%)
#define ACCELERATION_DELAY  20000   // Delay between speed changes

/* =============================================================================
 * FUNCTION PROTOTYPES
 * =============================================================================
 */
void Motor_GPIO_Config(void);
void Motor_PWM_Config(void);
void Motor_SetSpeed(uint8_t speed_percent);
void Motor_SetDirection(MotorDirection_t direction);
void Motor_SmoothSpeedChange(uint8_t target_speed);
uint16_t Motor_PercentToCompare(uint8_t percent);
void Delay(uint32_t count);

/* =============================================================================
 * GLOBAL VARIABLES
 * =============================================================================
 */
uint8_t current_speed = 0;              // Current motor speed (0-100%)
MotorDirection_t current_direction = MOTOR_STOP; // Current direction

/* =============================================================================
 * MAIN FUNCTION
 * =============================================================================
 */
int main(void)
{
    // Initialize motor control system
    Motor_GPIO_Config();
    Motor_PWM_Config();
    
    // Start with motor stopped
    Motor_SetDirection(MOTOR_STOP);
    Motor_SetSpeed(0);
    
    /* =================================================================
     * MAIN LOOP - MOTOR CONTROL DEMONSTRATION
     * =================================================================
     * This demonstrates various motor control patterns:
     * 1. Forward acceleration and deceleration
     * 2. Reverse operation
     * 3. Smooth speed transitions
     * 4. Emergency stop functionality
     */
    while(1)
    {
        /* =========================================================
         * FORWARD OPERATION CYCLE
         * =========================================================
         */
        
        // Set forward direction
        Motor_SetDirection(MOTOR_FORWARD);
        Delay(100000); // Small delay for direction change
        
        // Accelerate to 80% speed
        Motor_SmoothSpeedChange(80);
        Delay(2000000); // Run at 80% for 2 seconds
        
        // Accelerate to full speed
        Motor_SmoothSpeedChange(100);
        Delay(2000000); // Run at full speed for 2 seconds
        
        // Decelerate to 50%
        Motor_SmoothSpeedChange(50);
        Delay(1000000); // Run at 50% for 1 second
        
        // Stop motor (brake)
        Motor_SmoothSpeedChange(0);
        Motor_SetDirection(MOTOR_STOP);
        Delay(1000000); // Wait 1 second
        
        /* =========================================================
         * REVERSE OPERATION CYCLE
         * =========================================================
         */
        
        // Set reverse direction
        Motor_SetDirection(MOTOR_REVERSE);
        Delay(100000); // Small delay for direction change
        
        // Accelerate to 60% reverse speed
        Motor_SmoothSpeedChange(60);
        Delay(2000000); // Run in reverse for 2 seconds
        
        // Stop motor
        Motor_SmoothSpeedChange(0);
        Motor_SetDirection(MOTOR_STOP);
        Delay(1000000); // Wait 1 second
        
        /* =========================================================
         * SPEED RAMPING DEMONSTRATION
         * =========================================================
         */
        
        Motor_SetDirection(MOTOR_FORWARD);
        Delay(100000);
        
        // Ramp up from 0% to 100% in steps
        for(uint8_t speed = 0; speed <= 100; speed += 10)
        {
            Motor_SetSpeed(speed);
            Delay(300000); // 300ms at each speed
        }
        
        // Ramp down from 100% to 0% in steps
        for(uint8_t speed = 100; speed > 0; speed -= 10)
        {
            Motor_SetSpeed(speed);
            Delay(300000); // 300ms at each speed
        }
        
        Motor_SetSpeed(0);
        Motor_SetDirection(MOTOR_STOP);
        Delay(2000000); // Wait 2 seconds before next cycle
    }
}

/* =============================================================================
 * MOTOR GPIO CONFIGURATION
 * =============================================================================
 * Configures GPIO pins for motor control:
 * - PA6: PWM output for speed control (TIM3_CH1)
 * - PA1: Direction control pin 1
 * - PA2: Direction control pin 2
 */
void Motor_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    
    // Enable GPIOA clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    
    /* Configure PA6 (TIM3_CH1) for PWM output */
    GPIO_InitStructure.GPIO_Pin = MOTOR_PWM_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;     // Alternate function push-pull
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    /* Configure PA1 and PA2 for direction control */
    GPIO_InitStructure.GPIO_Pin = MOTOR_DIR1_PIN | MOTOR_DIR2_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;    // Output push-pull
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    // Initialize direction pins to safe state (motor stopped)
    GPIO_ResetBits(GPIOA, MOTOR_DIR1_PIN);
    GPIO_ResetBits(GPIOA, MOTOR_DIR2_PIN);
}

/* =============================================================================
 * MOTOR PWM CONFIGURATION
 * =============================================================================
 * Configures TIM3 Channel 1 for motor speed control PWM.
 * Uses 1kHz frequency which is suitable for most DC motors.
 */
void Motor_PWM_Config(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    
    // Enable TIM3 clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    
    /* =================================================================
     * TIMER BASE CONFIGURATION
     * =================================================================
     * 1kHz PWM frequency is ideal for motor control:
     * - Low enough to be efficient (minimal switching losses)
     * - High enough to avoid audible noise
     * - Good balance between performance and efficiency
     */
    TIM_TimeBaseStructure.TIM_Period = PERIOD - 1;        // 1000-1 = 999
    TIM_TimeBaseStructure.TIM_Prescaler = PRESCALER - 1;  // 72-1 = 71
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
    
    /* =================================================================
     * PWM OUTPUT COMPARE CONFIGURATION
     * =================================================================
     */
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;           // PWM Mode 1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;                          // Start with 0% duty cycle
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;   // Active high
    TIM_OC1Init(TIM3, &TIM_OCInitStructure);
    
    // Enable preload for smooth operation
    TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM3, ENABLE);
    
    // Start timer
    TIM_Cmd(TIM3, ENABLE);
}

/* =============================================================================
 * SET MOTOR SPEED
 * =============================================================================
 * Sets motor speed as percentage (0% to 100%).
 * 
 * Parameters:
 *   speed_percent - Desired speed (0-100%)
 *                  0% = motor stopped
 *                  100% = full speed
 * 
 * Note: This function only sets PWM duty cycle. Direction must be
 * set separately using Motor_SetDirection().
 */
void Motor_SetSpeed(uint8_t speed_percent)
{
    uint16_t compare_value;
    
    // Limit speed to valid range
    if(speed_percent > SPEED_MAX)
        speed_percent = SPEED_MAX;
    
    // Convert percentage to compare value
    compare_value = Motor_PercentToCompare(speed_percent);
    
    // Update PWM duty cycle
    TIM_SetCompare1(TIM3, compare_value);
    
    // Update current speed tracking
    current_speed = speed_percent;
}

/* =============================================================================
 * SET MOTOR DIRECTION
 * =============================================================================
 * Controls motor direction using H-bridge logic.
 * 
 * H-Bridge Truth Table (L298N):
 * IN1 | IN2 | Motor Action
 * ----|-----|-------------
 *  0  |  0  | Stop (brake)
 *  0  |  1  | Forward
 *  1  |  0  | Reverse  
 *  1  |  1  | Stop (brake)
 * 
 * Parameters:
 *   direction - Motor direction (MOTOR_STOP, MOTOR_FORWARD, MOTOR_REVERSE, MOTOR_COAST)
 */
void Motor_SetDirection(MotorDirection_t direction)
{
    switch(direction)
    {
        case MOTOR_STOP:
            // Brake: Both inputs LOW (active braking)
            GPIO_ResetBits(GPIOA, MOTOR_DIR1_PIN);
            GPIO_ResetBits(GPIOA, MOTOR_DIR2_PIN);
            break;
            
        case MOTOR_FORWARD:
            // Forward: IN1=0, IN2=1
            GPIO_ResetBits(GPIOA, MOTOR_DIR1_PIN);
            GPIO_SetBits(GPIOA, MOTOR_DIR2_PIN);
            break;
            
        case MOTOR_REVERSE:
            // Reverse: IN1=1, IN2=0
            GPIO_SetBits(GPIOA, MOTOR_DIR1_PIN);
            GPIO_ResetBits(GPIOA, MOTOR_DIR2_PIN);
            break;
            
        case MOTOR_COAST:
            // Coast: Disable PWM (motor freewheels)
            // This would require disabling PWM output
            Motor_SetSpeed(0);
            break;
            
        default:
            // Invalid direction - stop motor for safety
            GPIO_ResetBits(GPIOA, MOTOR_DIR1_PIN);
            GPIO_ResetBits(GPIOA, MOTOR_DIR2_PIN);
            break;
    }
    
    current_direction = direction;
}

/* =============================================================================
 * SMOOTH SPEED CHANGE
 * =============================================================================
 * Gradually changes motor speed to target value for smooth operation.
 * This prevents sudden torque changes that could damage gears or cause
 * current spikes.
 * 
 * Parameters:
 *   target_speed - Target speed percentage (0-100%)
 */
void Motor_SmoothSpeedChange(uint8_t target_speed)
{
    // Limit target speed
    if(target_speed > SPEED_MAX)
        target_speed = SPEED_MAX;
    
    // Gradually change speed
    while(current_speed != target_speed)
    {
        if(current_speed < target_speed)
        {
            // Accelerating
            current_speed += ACCELERATION_STEP;
            if(current_speed > target_speed)
                current_speed = target_speed;
        }
        else
        {
            // Decelerating
            if(current_speed >= ACCELERATION_STEP)
                current_speed -= ACCELERATION_STEP;
            else
                current_speed = 0;
                
            if(current_speed < target_speed)
                current_speed = target_speed;
        }
        
        // Apply new speed
        Motor_SetSpeed(current_speed);
        
        // Wait before next step
        Delay(ACCELERATION_DELAY);
    }
}

/* =============================================================================
 * CONVERT PERCENTAGE TO COMPARE VALUE
 * =============================================================================
 * Converts speed percentage (0-100%) to timer compare value (0-PERIOD).
 * 
 * Formula: compare = (percentage * PERIOD) / 100
 */
uint16_t Motor_PercentToCompare(uint8_t percent)
{
    return (uint16_t)((uint32_t)percent * PERIOD / 100);
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
 * DC MOTOR CONTROL THEORY
 * =============================================================================
 * 
 * PWM MOTOR CONTROL PRINCIPLES:
 * 
 * 1. SPEED CONTROL:
 *    - Motor speed is proportional to average voltage
 *    - PWM varies average voltage by changing duty cycle
 *    - Higher duty cycle = higher average voltage = faster speed
 *    - Motor inertia smooths out PWM switching
 * 
 * 2. DIRECTION CONTROL:
 *    - Requires H-bridge to reverse current flow
 *    - Four switches arranged in "H" configuration
 *    - Diagonal switch pairs turned on for each direction
 * 
 * 3. BRAKING METHODS:
 *    - Active braking: Short motor terminals (energy dissipated as heat)
 *    - Regenerative braking: Feed energy back to supply
 *    - Coasting: Disconnect motor (freewheeling)
 * 
 * H-BRIDGE OPERATION:
 * 
 * H-bridge allows bidirectional current flow through motor:
 * 
 *     +V ----[S1]----+----[S2]---- +V
 *                     |
 *                   MOTOR
 *                     |
 *     GND ---[S3]----+----[S4]---- GND
 * 
 * Forward:  S1=ON, S2=OFF, S3=OFF, S4=ON  (current: +V -> S1 -> Motor -> S4 -> GND)
 * Reverse:  S1=OFF, S2=ON, S3=ON, S4=OFF  (current: +V -> S2 -> Motor -> S3 -> GND)
 * Brake:    S1=OFF, S2=OFF, S3=ON, S4=ON  (motor terminals shorted to GND)
 * 
 * PWM FREQUENCY SELECTION:
 * 
 * Motor PWM frequency considerations:
 * - Too low (< 100Hz): Audible noise, torque ripple
 * - Optimal (1-10kHz): Good efficiency, quiet operation  
 * - Too high (> 50kHz): Switching losses, EMI issues
 * 
 * Typical frequencies:
 * - Small DC motors: 1-5kHz
 * - Large motors: 100Hz-1kHz
 * - Servo motors: 50Hz (position control)
 * 
 * MOTOR CHARACTERISTICS:
 * 
 * 1. BACK EMF:
 *    - Rotating motor generates voltage opposing applied voltage
 *    - Limits current at steady-state speed
 *    - Provides speed feedback (can be measured)
 * 
 * 2. STARTING CURRENT:
 *    - High current when motor starts (no back EMF)
 *    - May require current limiting or soft start
 *    - Consider fuse/protection circuit
 * 
 * 3. TORQUE-SPEED RELATIONSHIP:
 *    - Torque decreases with increasing speed
 *    - Maximum torque at stall (0 RPM)
 *    - Maximum speed at no load
 * 
 * PRACTICAL CONSIDERATIONS:
 * 
 * 1. POWER SUPPLY:
 *    - Adequate current capacity for motor + spikes
 *    - Voltage rating matching motor requirements
 *    - Decoupling capacitors near H-bridge
 * 
 * 2. PROTECTION:
 *    - Flyback diodes for inductive load protection
 *    - Current sensing for overcurrent protection
 *    - Thermal protection for H-bridge
 * 
 * 3. CONTROL ALGORITHMS:
 *    - Open loop: Simple PWM control (this example)
 *    - Closed loop: Speed feedback with encoder/tachometer
 *    - PID control: Precise speed/position control
 * 
 * 4. SAFETY:
 *    - Emergency stop functionality
 *    - Watchdog timer for fault detection
 *    - Gradual speed changes to prevent damage
 * 
 * =============================================================================
 */
