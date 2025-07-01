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
    
    // ✅ Simple but fast PID
    float error = mc->target_rpm - mc->current_rpm;
    float dt_sec = dt_ms / 1000.0f;
    
    // ✅ Always integrate for steady-state accuracy
    mc->integral += error * dt_sec;
    if (mc->integral > 800) mc->integral = 800;
    if (mc->integral < -800) mc->integral = -800;
    
    float derivative = (error - mc->previous_error) / dt_sec;
    
    // ✅ Simple PID calculation
    float output = mc->kp * error + mc->ki * mc->integral + mc->kd * derivative;
    
    // ✅ Full range output
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
