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

/* Called from HAL_GPIO_EXTI_Callback when MPU6050 INT fires (PB5 falling) */
void MPU6050_Control_IRQ(void)
{
    static u8 Voltage_Counter = 0;

    mpu_dmp_get_data(&pitch, &roll, &yaw);
    MPU_Get_Gyroscope(&gyrox, &gyroy, &gyroz);
    Encoder_Left  =  Read_Encoder(2);
    Encoder_Right = -Read_Encoder(3);
    Led_Flash(100);

    if (++Voltage_Counter == 20) {
        Voltage_Counter = 0;
        Voltage = Get_battery_volt();
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
    int   balance = (int)(balance_UP_KP * Bias + balance_UP_KD * Gyro);
    return balance;
}

int velocity(int encoder_left, int encoder_right, int gyro_Z)
{
    static float Velocity, Encoder_Least, Encoder;
    static float Encoder_Integral;

    Encoder_Least     = (float)(Encoder_Left + Encoder_Right);
    Encoder          *= 0.8f;
    Encoder          += Encoder_Least * 0.2f;
    Encoder_Integral += Encoder;
    Encoder_Integral -= (float)gyro_Z;

    if (Encoder_Integral >  10000.0f) Encoder_Integral =  10000.0f;
    if (Encoder_Integral < -10000.0f) Encoder_Integral = -10000.0f;

    Velocity = Encoder * velocity_KP + Encoder_Integral * velocity_KI;

    if (pitch < -40.0f || pitch > 40.0f) Encoder_Integral = 0.0f;

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
            if (!Fore && !Back) Target_Speed = 0;
            if (Fore)  Target_Speed--;
            if (Back)  Target_Speed++;
            if (!Left && !Right) Turn_Speed = 0;
            if (Left)  Turn_Speed -= 30;
            if (Right) Turn_Speed += 30;
            if (!Left && !Right) Turn_Kd = -0.6f;
            else                 Turn_Kd =  0.0f;
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
