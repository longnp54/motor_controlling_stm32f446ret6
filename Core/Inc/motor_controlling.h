#ifndef MOTOR_CONTROLLING_H
#define MOTOR_CONTROLLING_H

#include "main.h"

#define ENCODER_CPR 515

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

// Function prototypes
void MotorController_Setup(MotorController *mc, TIM_HandleTypeDef *enc_tim, 
                          TIM_HandleTypeDef *pwm_tim, uint32_t ch_fwd, 
                          uint32_t ch_bwd, float kp, float ki, float kd);
void MotorController_Enable(MotorController *mc);
void MotorController_Disable(MotorController *mc);
void MotorController_SetTargetRPM(MotorController *mc, float rpm);
void MotorController_Update(MotorController *mc, uint32_t dt_ms);
float MotorController_GetRPM(MotorController *mc);
void Motor_SetDutyCycle(MotorController *mc, int8_t duty_cycle);

#endif
