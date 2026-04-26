#include "control.h"
#include "mpu6050.h"
#include "encoder.h"
#include "motor.h"
#include "adc.h"
#include "led.h"
#include "key.h"
#include "oled.h"
#include "SR04.h"
#include "PS2.h"
#include "usart2.h"

#define SPEED_Y 100   /* max forward/backward speed */
#define SPEED_Z 80    /* max turn speed */
#define LEFT_ENCODER_SIGN  (1)
#define RIGHT_ENCODER_SIGN (-1)
#define BALANCE_OUTPUT_SIGN (1)

int   Balance_Pwm, Velocity_Pwm, Turn_Pwm, Turn_Kp;

float Mechanical_angle = 0.0f;
float Target_Speed     = 0.0f;
float Turn_Speed       = 0.0f;

float balance_UP_KP = BLC_KP;
float balance_UP_KD = BLC_KD;
float velocity_KP   = SPD_KP;
float velocity_KI   = SPD_KI;
float Turn_Kd       = TURN_KD;
float Turn_KP       = TURN_KP;
static u8 Motor_Armed = 0;
static float velocity_encoder = 0.0f;
static float velocity_integral = 0.0f;

/* Called from HAL_GPIO_EXTI_Callback when MPU6050 INT fires (PB5 falling) */
void MPU6050_Control_IRQ(void)
{
    static u8 Voltage_Counter = 0;

    mpu_dmp_get_data(&pitch, &roll, &yaw);
    MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz);
    /* Match the working robot convention: both wheels should report the
     * same sign for the same physical travel direction. If this sign is
     * wrong, the velocity loop becomes positive feedback and the motors run
     * away as soon as they are armed.
     */
    Encoder_Left  = LEFT_ENCODER_SIGN * Read_Encoder(2);
    Encoder_Right = RIGHT_ENCODER_SIGN * Read_Encoder(3);
    Led_Flash(100);

    if (++Voltage_Counter == 20) {
        Voltage_Counter = 0;
        Voltage = Get_battery_volt();
    }

    /* Safety latch:
     * Boot with motor output disabled. Hold KEY long enough to arm.
     * This lets us validate IMU/encoder direction without runaway wheels.
     */
    if (!Motor_Armed) {
        if (KEY_Press(200)) {
            Motor_Armed = 1;
        } else {
            Moto1 = 0;
            Moto2 = 0;
            Set_Pwm(0, 0);
            return;
        }
    }

    if (KEY_Press(100)) {
        if (++CTRL_MODE >= 101) CTRL_MODE = 97;
        Mode_Change = 1;
    }

    Get_RC();

    if (Target_Speed >  SPEED_Y) Target_Speed =  SPEED_Y;
    if (Target_Speed < -SPEED_Y) Target_Speed = -SPEED_Y;
    if (Turn_Speed   >  SPEED_Z) Turn_Speed   =  SPEED_Z;
    if (Turn_Speed   < -SPEED_Z) Turn_Speed   = -SPEED_Z;

    Balance_Pwm  = balance_UP(pitch, Mechanical_angle, gyroy);
    Velocity_Pwm = velocity(Encoder_Left, Encoder_Right, Target_Speed);
    Turn_Pwm     = Turn_UP(gyroz, Turn_Speed);

    Moto1 = Balance_Pwm - Velocity_Pwm + Turn_Pwm;
    Moto2 = Balance_Pwm - Velocity_Pwm - Turn_Pwm;

    Xianfu_Pwm();
    Turn_Off(pitch, 12);
    Set_Pwm(Moto1, Moto2);
}

int balance_UP(float Angle, float Mechanical_balance, float Gyro)
{
    float Bias    = Angle - Mechanical_balance;
    int   balance = BALANCE_OUTPUT_SIGN *
                    (int)(balance_UP_KP * Bias + balance_UP_KD * Gyro);
    return balance;
}

int velocity(int encoder_left, int encoder_right, int target_speed)
{
    float Velocity;
    float Encoder_Least;

    Encoder_Least     = (float)(encoder_left + encoder_right);
    velocity_encoder *= 0.8f;
    velocity_encoder += Encoder_Least * 0.2f;
    velocity_integral += velocity_encoder;
    velocity_integral -= (float)target_speed;

    if (velocity_integral >  10000.0f) velocity_integral =  10000.0f;
    if (velocity_integral < -10000.0f) velocity_integral = -10000.0f;

    Velocity = velocity_encoder * velocity_KP + velocity_integral * velocity_KI;

    if (pitch < -40.0f || pitch > 40.0f) velocity_integral = 0.0f;

    return (int)Velocity;
}

int Turn_UP(int gyro_Z, int RC)
{
    return (int)(Turn_Kd * gyro_Z + Turn_KP * RC);
}

void Tracking(void)
{
    TkSensor  = 0;
    TkSensor += (u8)(C1 << 3);
    TkSensor += (u8)(C2 << 2);
    TkSensor += (u8)(C3 << 1);
    TkSensor += (u8)C4;
}

void Get_RC(void)
{
    static u8    SR04_Counter = 0;
    static float RATE_VEL    = 1.0f;
    float        RATE_TURN   = 1.6f;
    float        LY, RX;
    int          Yuzhi = 2;

    switch (CTRL_MODE) {
        case 97:
            if (++SR04_Counter >= 20) {
                SR04_Counter = 0;
                SR04_StartMeasure();
            }
            break;

        case 98:
            if (Bluetooth_ManualControlActive()) {
                break;
            }

            if (!Fore && !Back) Target_Speed = 0.0f;
            if (Fore)  Target_Speed--;
            if (Back)  Target_Speed++;
            if (!Left && !Right) Turn_Speed = 0.0f;
            if (Left)  Turn_Speed -= 30.0f;
            if (Right) Turn_Speed += 30.0f;
            if (!Left && !Right) Turn_Kd = TURN_KD;
            else                 Turn_Kd = 0.0f;
            break;

        case 99:
            Tracking();
            switch (TkSensor) {
                case 15: Target_Speed = 0;   Turn_Speed =   0; break;
                case  9: Target_Speed--;      Turn_Speed =   0; break;
                case  2: Target_Speed--;      Turn_Speed =  15; break;
                case  4: Target_Speed--;      Turn_Speed = -15; break;
                case  8: Target_Speed = -10;  Turn_Speed = -80; break;
                case  1: Target_Speed = -10;  Turn_Speed =  80; break;
            }
            break;

        case 100:
            if (PS2_Plugin) {
                LY = (float)(PS2_LY - 128);
                RX = (float)(PS2_RX - 128);
                if (LY > -Yuzhi && LY < Yuzhi) LY = 0;
                if (RX > -Yuzhi && RX < Yuzhi) RX = 0;
                if      (Target_Speed > -LY / RATE_VEL) Target_Speed--;
                else if (Target_Speed < -LY / RATE_VEL) Target_Speed++;
                Turn_Speed = RX / RATE_TURN;
            } else {
                Target_Speed = 0; Turn_Speed = 0;
            }
            break;
    }
}

void Control_ResetState(void)
{
    velocity_encoder = 0.0f;
    velocity_integral = 0.0f;
    Target_Speed = 0.0f;
    Turn_Speed = 0.0f;
}
