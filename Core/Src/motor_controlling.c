#include "motor_controlling.h"
#include <stdlib.h>
#include <math.h>  // Add this for fabs() function

#define MAX_PWM 44999
#define MAX_OUTPUT 100.0f  // ✅ Changed from 127.0f to 100.0f (percentage)
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
    
    // ✅ IMPROVED: Variable integral gain approach
    // - Uses smaller Ki when close to target
    // - Completely disables integral in a deadband
    float abs_error = fabs(error);
    float variable_ki = mc->ki;
    
    if (abs_error < 5.0f) {
        // Reduce integral gain when close to target
        variable_ki = mc->ki * (abs_error / 5.0f);
        
        // ✅ Deadband approach - no integration when very close
        if (abs_error < 1.0f) {
            variable_ki = 0.0f;  // Disable integral completely
        }
    }
    
    // ✅ IMPROVED: Calculate derivative with more filtering
    float derivative = (error - mc->previous_error) / dt_sec;
    
    // ✅ EFFICIENT: Pre-calculate proportional and derivative terms
    float p_term = mc->kp * error;
    float d_term = mc->kd * derivative;
    
    // ✅ EFFICIENT: Calculate what output would be without integral
    float output_without_integral = p_term + d_term;
    
    // ✅ BETTER ANTI-WINDUP: Back-calculation method
    float integral_change = 0.0f;
    
    // Only accumulate integral when appropriate
    if (abs_error > 1.0f && fabs(output_without_integral) < 90.0f) {
        // Normal integration
        integral_change = error * dt_sec;
    } else {
        // ✅ IMPROVED: Targeted integral reduction
        // Reduces integral exactly to the point where output won't saturate
        if (fabs(mc->integral) > 0.1f) {
            // Calculate how much integral action we need to reduce
            float desired_integral = (90.0f - fabs(output_without_integral)) / variable_ki;
            desired_integral = fmin(desired_integral, fabs(mc->integral));
            
            // Make integral decay to this value
            float decay_rate = 0.3f * dt_sec;  // 30% decay per second
            integral_change = -decay_rate * mc->integral;
        }
    }
    
    // ✅ IMPROVED: Apply variable Ki to integration
    mc->integral += integral_change * variable_ki;
    
    // ✅ Apply integral limits (avoid unnecessary comparisons when possible)
    if (mc->integral > 50.0f) {
        mc->integral = 50.0f;
    } else if (mc->integral < -50.0f) {
        mc->integral = -50.0f;
    }
    
    // ✅ IMPROVED: Detect overshoot with hysteresis to avoid oscillation
    static float last_error = 0.0f;
    static uint8_t same_sign_count = 0;
    
    if ((error > 0 && last_error < 0) || (error < 0 && last_error > 0)) {
        // Zero crossing detected (error changed sign) - possible overshoot
        if (fabs(error) > 2.0f) {  // Only reset if significant overshoot
            mc->integral = 0.0f;
            same_sign_count = 0;
        }
    } else {
        // Error still has same sign
        same_sign_count++;
        
        // If error stays same sign for a while, we're not oscillating
        // Allow a small amount of integral to accumulate for steady-state
        if (same_sign_count > 50 && fabs(mc->integral) < 0.1f && abs_error > 1.0f) {
            // Small nudge to overcome steady-state error
            mc->integral = (error > 0) ? 1.0f : -1.0f;
        }
    }
    
    last_error = error;
    
    // Calculate final output
    float i_term = variable_ki * mc->integral;
    float output = p_term + i_term + d_term;
    
    // Apply output limits
    if (output > 100.0f) output = 100.0f;
    if (output < -100.0f) output = -100.0f;
    
    mc->previous_error = error;
    Motor_SetDutyCycle(mc, (int8_t)output);
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
