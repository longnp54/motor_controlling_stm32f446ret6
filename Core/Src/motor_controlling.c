#include "motor_controlling.h"
#include <stdlib.h>

#define MAX_PWM 44999
#define MAX_OUTPUT 140.0f
#define LOW_PASS_ALPHA 0.2f

//Initialize the motor controller structure
typedef struct {
    TIM_HandleTypeDef *encoder_timer;
    TIM_HandleTypeDef *pwm_timer;
    uint32_t pwm_channel_forward;
    uint32_t pwm_channel_backward;
    
    float kp, ki, kd;
    float target_rpm;
    float current_rpm;
    float integral;
    float previous_error;
    int32_t last_count;
    uint8_t is_enabled;
} MotorController;

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
    // 1.Read encoder from CCR register
    uint32_t curr = __HAL_TIM_GET_COUNTER(mc->encoder_timer);
    
    //Processing diff automatically
    int32_t diff = (int32_t)((curr - mc->last_count + 22500) % 45000) - 22500;
    
    mc->last_count = curr;
    
    float new_rpm = ((float)diff * 60000.0f) / (515 * dt_ms);
    
    // Low-pass filter
    mc->current_rpm = mc->current_rpm * 0.8f + new_rpm * 0.2f;
    
    if (!mc->is_enabled) return;
    
    // 2.Calculate PID
    float error = mc->target_rpm - mc->current_rpm;
    
    if (fabs(error) > 1.0f) {
        mc->integral += error * dt_ms;
        if (mc->integral > 5000) mc->integral = 5000;
        if (mc->integral < -5000) mc->integral = -5000;
    }
    
    float derivative = (error - mc->previous_error) / dt_ms;
    float output = mc->kp * error + mc->ki * mc->integral + mc->kd * derivative;
    
    if (output > MAX_OUTPUT) output = MAX_OUTPUT;
    if (output < -MAX_OUTPUT) output = -MAX_OUTPUT;
    
    mc->previous_error = error;
    
    if (fabs(error) > 0.5f) {
        Motor_SetDutyCycle(mc, (int8_t)output);
    }
}

float MotorController_GetRPM(MotorController *mc) {
    return mc->current_rpm;
}

// Set the duty cycle for the motor
void Motor_SetDutyCycle(MotorController *mc, int8_t duty_cycle) {
    // Clamp
    if (duty_cycle > 140) duty_cycle = 140;
    if (duty_cycle < -140) duty_cycle = -140;
    
    // Tính PWM
    uint32_t pwm_value = (abs(duty_cycle) * MAX_PWM) / 140;
    
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
