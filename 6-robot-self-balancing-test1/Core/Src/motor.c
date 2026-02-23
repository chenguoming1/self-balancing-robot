#include "motor.h"
#include "main.h"
#include <stdlib.h>   // abs()

/* ── Internal helper ─────────────────────────────────────────────────── */
static void set_motor(uint32_t pwm_ch,
                      GPIO_TypeDef *in1_port, uint16_t in1_pin,
                      GPIO_TypeDef *in2_port, uint16_t in2_pin,
                      int32_t speed)
{
    /* Clamp to ±MOTOR_PWM_MAX */
    if (speed >  (int32_t)MOTOR_PWM_MAX) speed =  (int32_t)MOTOR_PWM_MAX;
    if (speed < -(int32_t)MOTOR_PWM_MAX) speed = -(int32_t)MOTOR_PWM_MAX;

    uint32_t duty = (uint32_t)abs((int)speed);

    if (speed > 0) {
        /* Forward */
        HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_RESET);
    } else if (speed < 0) {
        /* Reverse */
        HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_SET);
    } else {
        /* Brake: both IN pins HIGH = short brake on TB6612 */
        HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_SET);
        duty = 0;
    }

    __HAL_TIM_SET_COMPARE(&htim1, pwm_ch, duty);
}

/* ── Public API ──────────────────────────────────────────────────────── */

void Motor_Set(int32_t left_speed, int32_t right_speed)
{
    /* Motor A (Left):  AIN1=PB13  AIN2=PB12  PWMA=TIM1_CH4 (PA11) */
    set_motor(MOTOR_A_PWM_CH,
              MOTOR_A_IN1_PORT, MOTOR_A_IN1_PIN,
              MOTOR_A_IN2_PORT, MOTOR_A_IN2_PIN,
              left_speed);

    /* Motor B (Right): BIN1=PB14  BIN2=PB15  PWMB=TIM1_CH1 (PA8) */
    set_motor(MOTOR_B_PWM_CH,
              MOTOR_B_IN1_PORT, MOTOR_B_IN1_PIN,
              MOTOR_B_IN2_PORT, MOTOR_B_IN2_PIN,
              right_speed);
}

void Motor_Stop(void)
{
    Motor_Set(0, 0);
}
