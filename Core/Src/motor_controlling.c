#include "motor_controlling.h"
#include <stdlib.h>
#include <math.h>  // Add this for fabs() function

#define MAX_PWM 44999
#define MAX_OUTPUT 140.0f  // ✅ Changed from 127.0f to 100.0f (percentage)
#define LOW_PASS_ALPHA 0.2f

void MotorController_Setup(MotorController *mc,
                          TIM_HandleTypeDef *enc_tim,
                          TIM_HandleTypeDef *pwm_tim,
                          uint32_t ch_fwd, uint32_t ch_bwd,
                          float kp, float ki, float kd) {
    mc->encoder_timer = enc_tim;
    mc->pwm_timer = pwm_tim;
    mc->pwm_channel_forward = ch_fwd;
    mc->pwm_channel_backward = ch_bwd;
    
    mc->kp = kp;
    mc->ki = ki;
    mc->kd = kd;
    
    mc->last_count = __HAL_TIM_GET_COUNTER(mc->encoder_timer);
    mc->integral = 0;
    mc->previous_error = 0;
    mc->target_rpm = 0;
    mc->current_rpm = 0;
    mc->is_enabled = 0;
    
    // Stop PWM
    __HAL_TIM_SET_COMPARE(mc->pwm_timer, mc->pwm_channel_forward, 0);
    __HAL_TIM_SET_COMPARE(mc->pwm_timer, mc->pwm_channel_backward, 0);
}

void MotorController_Enable(MotorController *mc) {
    mc->is_enabled = 1;
    HAL_TIM_PWM_Start(mc->pwm_timer, mc->pwm_channel_forward);
    HAL_TIM_PWM_Start(mc->pwm_timer, mc->pwm_channel_backward);
}

void MotorController_Disable(MotorController *mc) {
    mc->is_enabled = 0;
    __HAL_TIM_SET_COMPARE(mc->pwm_timer, mc->pwm_channel_forward, 0);
    __HAL_TIM_SET_COMPARE(mc->pwm_timer, mc->pwm_channel_backward, 0);
    HAL_TIM_PWM_Stop(mc->pwm_timer, mc->pwm_channel_forward);
    HAL_TIM_PWM_Stop(mc->pwm_timer, mc->pwm_channel_backward);
}

void MotorController_SetTargetRPM(MotorController *mc, float rpm) {
    mc->target_rpm = rpm;
}

// PID controller update function
void MotorController_Update(MotorController *mc, uint32_t dt_ms) {
    // Read encoder
    uint32_t curr = __HAL_TIM_GET_COUNTER(mc->encoder_timer);
    int32_t diff = (int32_t)((curr - mc->last_count + 22500) % 45000) - 22500;
    mc->last_count = curr;
    
    // Calculate RPM
    float new_rpm = ((float)diff * 60000.0f) / (515 * dt_ms);
    
    // ✅ Faster filtering
    mc->current_rpm = mc->current_rpm * 0.6f + new_rpm * 0.4f;
    
    if (!mc->is_enabled) return;
    
    // ✅ Calculate error
    float error = mc->target_rpm - mc->current_rpm;
    float dt_sec = dt_ms / 1000.0f;
    
    // ✅ MODIFIED: Much more aggressive integral handling for steady-state error
    float abs_error = fabs(error);
    float variable_ki = mc->ki;
    
    // ✅ BOOST INTEGRAL GAIN for large persistent errors
    // We need much stronger integral action to overcome steady-state error
    if (abs_error > 10.0f) {
        // For large errors, boost integral gain dramatically
        variable_ki = mc->ki * 3.0f;  // 300% of normal Ki
    } else if (abs_error >= 1.0f) {
        // For moderate errors, use normal Ki with slight boost
        variable_ki = mc->ki * 1.5f;  // 150% of normal Ki
    } else {
        // Very close to target, use normal Ki
        variable_ki = mc->ki;
    }
    
    // Calculate derivative with filtering
    float derivative = (error - mc->previous_error) / dt_sec;
    
    // Pre-calculate terms
    float p_term = mc->kp * error;
    float d_term = mc->kd * derivative;
    
    // Calculate what output would be without integral
    float output_without_integral = p_term + d_term;
    
    // ✅ AGGRESSIVE: Added feed-forward bias term to overcome steady-state error
    // This is a critical addition for DC motors that have friction and load
    static float bias_term = 0.0f;
    
    // Gradually build up bias term if we're consistently under target
    if (error > 5.0f && mc->current_rpm > 0) {
        // We're consistently under target, increase bias
        bias_term += 0.05f;  // Small increment per cycle
        if (bias_term > 20.0f) bias_term = 20.0f;  // Limit bias
    } else if (error < -5.0f) {
        // We're overshooting, reduce bias
        bias_term -= 0.1f;  // Faster reduction if overshooting
        if (bias_term < 0.0f) bias_term = 0.0f;
    }
    
    // Calculate final output with bias term
    float i_term = variable_ki * mc->integral;
    float output = p_term + i_term + d_term + bias_term;  // Add bias term
    
    // Apply output limits
    if (output > 100.0f) output = 100.0f;
    if (output < -100.0f) output = -100.0f;
    
    mc->previous_error = error;
    Motor_SetDutyCycle(mc, (int8_t)output);
    
    // ✅ FIXED: Steady-state error compensation
    float integral_change = 0.0f;
    
    // ✅ MUCH MORE AGGRESSIVE: Faster persistent error detection and response
    static uint8_t persistent_error_counter = 0;
    
    if (abs_error > 3.0f) {
        persistent_error_counter++;
        
        // If error persists for a while, increase integral action SIGNIFICANTLY
        if (persistent_error_counter > 30) {  // Reduced from 100 to 30 (300ms)
            // MUCH stronger integral action for persistent errors
            integral_change = error * dt_sec * 3.0f;  // 300% stronger (was 1.5f)
        } else {
            // Normal integration
            integral_change = error * dt_sec;
        }
    } else {
        persistent_error_counter = 0;
        
        // NEVER reduce integral when we have persistent error below target
        if (abs_error <= 1.0f || fabs(output_without_integral) >= 95.0f) {
            if (fabs(mc->integral) > 0.1f) {
                // Even slower decay rate
                float decay_rate = 0.05f * dt_sec;  // Reduced from 0.1f
                integral_change = -decay_rate * mc->integral;
            }
        } else {
            // Standard integration for small errors
            integral_change = error * dt_sec;
        }
    }
    
    // Apply variable Ki to integration
    mc->integral += integral_change * variable_ki;
    
    // ✅ MODIFIED: Much higher integral limits
    if (mc->integral > 100.0f) {      // Increased from 80.0f
        mc->integral = 100.0f;
    } else if (mc->integral < -100.0f) {
        mc->integral = -100.0f;
    }
}

float MotorController_GetRPM(MotorController *mc) {
    return mc->current_rpm;
}

// Set the duty cycle for the motor
void Motor_SetDutyCycle(MotorController *mc, int8_t duty_cycle) {
    // ✅ Use percentage-based (0-100%) 
    if (duty_cycle > 100) duty_cycle = 100;
    if (duty_cycle < -100) duty_cycle = -100;
    
    // ✅ Remove minimum threshold - let small values pass through
    // No minimum threshold - direct control
    
    // ✅ Calculate PWM value using percentage (0-100%)
    uint32_t pwm_value = (abs(duty_cycle) * MAX_PWM) / 100;
    
    if (duty_cycle > 0) {
        // Forward
        __HAL_TIM_SET_COMPARE(mc->pwm_timer, mc->pwm_channel_forward, pwm_value);
        __HAL_TIM_SET_COMPARE(mc->pwm_timer, mc->pwm_channel_backward, 0);
    } else if (duty_cycle < 0) {
        // Backward
        __HAL_TIM_SET_COMPARE(mc->pwm_timer, mc->pwm_channel_forward, 0);
        __HAL_TIM_SET_COMPARE(mc->pwm_timer, mc->pwm_channel_backward, pwm_value);
    } else {
        // Stop
        __HAL_TIM_SET_COMPARE(mc->pwm_timer, mc->pwm_channel_forward, 0);
        __HAL_TIM_SET_COMPARE(mc->pwm_timer, mc->pwm_channel_backward, 0);
    }
}
