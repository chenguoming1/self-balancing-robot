#include "usart2.h"
#include "control.h"
#include "ring_buffer.h"

u8 Fore = 0, Back = 0, Left = 0, Right = 0;
extern float pitch;
extern short gyroy;
extern int Encoder_Left;
extern int Encoder_Right;
extern int Moto1;
extern int Moto2;
extern float Voltage;

static uint8_t uart2_rx_byte;
static RingBuffer_t uart2_rx_rb;
static char bt_line[32];
static uint8_t bt_line_len = 0;
static uint8_t bt_line_active = 0;
static uint8_t bt_debug_stream = 0;
static uint8_t bt_manual_control = 0;
static char tx_buf[160];

#define MANUAL_DRIVE_MIN_EFFECTIVE  18.0f
#define MANUAL_DRIVE_SCALE           0.8f
#define MANUAL_DRIVE_MAX            60.0f
#define MANUAL_TURN_MIN_EFFECTIVE   10.0f
#define MANUAL_TURN_SCALE            0.6f
#define MANUAL_TURN_MAX             30.0f

static float parse_bt_float(const char *s)
{
    float value = 0.0f;
    float frac_scale = 0.1f;
    int sign = 1;
    uint8_t saw_dot = 0;

    if (*s == '-') {
        sign = -1;
        s++;
    } else if (*s == '+') {
        s++;
    }

    while (*s) {
        if (*s == '.') {
            if (saw_dot) {
                break;
            }
            saw_dot = 1;
        } else if (*s >= '0' && *s <= '9') {
            if (!saw_dot) {
                value = value * 10.0f + (float)(*s - '0');
            } else {
                value += (float)(*s - '0') * frac_scale;
                frac_scale *= 0.1f;
            }
        } else {
            break;
        }
        s++;
    }

    return (float)sign * value;
}

static char *append_str(char *dst, const char *src)
{
    while (*src) {
        *dst++ = *src++;
    }
    return dst;
}

static char *append_int(char *dst, int value)
{
    char tmp[16];
    int i = 0;
    unsigned int mag;

    if (value < 0) {
        *dst++ = '-';
        mag = (unsigned int)(-value);
    } else {
        mag = (unsigned int)value;
    }

    do {
        tmp[i++] = (char)('0' + (mag % 10U));
        mag /= 10U;
    } while (mag > 0U);

    while (i > 0) {
        *dst++ = tmp[--i];
    }

    return dst;
}

static char *append_float100(char *dst, float value)
{
    return append_int(dst, (int)(value * 100.0f));
}

static void build_debug_line(void)
{
    char *p = tx_buf;

    p = append_str(p, "dbg pc=");
    p = append_float100(p, pitch);
    p = append_str(p, " gy=");
    p = append_int(p, gyroy);
    p = append_str(p, " gz=");
    p = append_int(p, gyroz);
    p = append_str(p, " el=");
    p = append_int(p, Encoder_Left);
    p = append_str(p, " er=");
    p = append_int(p, Encoder_Right);
    p = append_str(p, " bp=");
    p = append_int(p, Balance_Pwm);
    p = append_str(p, " vp=");
    p = append_int(p, Velocity_Pwm);
    p = append_str(p, " tp=");
    p = append_int(p, Turn_Pwm);
    p = append_str(p, " m1=");
    p = append_int(p, Moto1);
    p = append_str(p, " m2=");
    p = append_int(p, Moto2);
    p = append_str(p, " a=");
    p = append_float100(p, Mechanical_angle);
    p = append_str(p, " p=");
    p = append_float100(p, balance_UP_KP);
    p = append_str(p, " d=");
    p = append_float100(p, balance_UP_KD);
    p = append_str(p, " w=");
    p = append_float100(p, velocity_KI);
    p = append_str(p, " k=");
    p = append_float100(p, Turn_Kd);
    p = append_str(p, " vb=");
    p = append_float100(p, Voltage);
    p = append_str(p, "\r\n");
    *p = '\0';
}

static float limit_abs(float value, float limit)
{
    if (value > limit) {
        return limit;
    }
    if (value < -limit) {
        return -limit;
    }
    return value;
}

static float shape_manual_drive(float raw_cmd)
{
    float mag;

    if (raw_cmd == 0.0f) {
        return 0.0f;
    }

    mag = MANUAL_DRIVE_MIN_EFFECTIVE + (fabsf(raw_cmd) * MANUAL_DRIVE_SCALE);
    mag = limit_abs(mag, MANUAL_DRIVE_MAX);
    return (raw_cmd > 0.0f) ? mag : -mag;
}

static float shape_manual_turn(float raw_cmd)
{
    float mag;

    if (raw_cmd == 0.0f) {
        return 0.0f;
    }

    mag = MANUAL_TURN_MIN_EFFECTIVE + (fabsf(raw_cmd) * MANUAL_TURN_SCALE);
    mag = limit_abs(mag, MANUAL_TURN_MAX);
    return (raw_cmd > 0.0f) ? mag : -mag;
}

static uint8_t is_ascii_tune_start(uint8_t ch)
{
    switch (ch) {
        case 'p':
        case 'd':
        case 'a':
        case 'v':
        case 'w':
        case 't':
        case 'r':
        case 'k':
        case 'q':
        case 'g':
        case 'l':
        case 's':
        case '?':
            return 1;
        default:
            return 0;
    }
}

static void apply_ascii_tune_command(const char *line)
{
    float value;

    if (line[0] == '\0') {
        return;
    }

    if (line[0] == '?') {
        Uart2SendStr(
            "cmds: p<kp> d<kd> a<angle> v<velkp> w<velki> t<speed> r<turn> k<turnkd> q<turnkp> g l0 l1 s\r\n");
        return;
    }

    if (line[0] == 'g') {
        build_debug_line();
        Uart2SendStr(tx_buf);
        return;
    }

    if (line[0] == 's') {
        Fore = 0;
        Back = 0;
        Left = 0;
        Right = 0;
        bt_manual_control = 0;
        Target_Speed = 0.0f;
        Turn_Speed = 0.0f;
        Control_ResetState();
        Uart2SendStr("ok stop\r\n");
        return;
    }

    if (line[0] == 'l') {
        bt_debug_stream = (line[1] == '1') ? 1u : 0u;
        Uart2SendStr(bt_debug_stream ? "ok log_on\r\n" : "ok log_off\r\n");
        return;
    }

    value = parse_bt_float(&line[1]);

    switch (line[0]) {
        case 'p':
            balance_UP_KP = value;
            Uart2SendStr("ok balance_kp\r\n");
            break;
        case 'd':
            balance_UP_KD = value;
            Uart2SendStr("ok balance_kd\r\n");
            break;
        case 'a':
            Mechanical_angle = value;
            Control_ResetState();
            Uart2SendStr("ok angle_trim\r\n");
            break;
        case 'v':
            velocity_KP = value;
            Uart2SendStr("ok velocity_kp\r\n");
            break;
        case 'w':
            velocity_KI = value;
            Control_ResetState();
            Uart2SendStr("ok velocity_ki\r\n");
            break;
        case 't':
            bt_manual_control = 1;
            Target_Speed = shape_manual_drive(value);
            Uart2SendStr("ok target_speed\r\n");
            break;
        case 'r':
            bt_manual_control = 1;
            Turn_Speed = shape_manual_turn(value);
            Uart2SendStr("ok turn_speed\r\n");
            break;
        case 'k':
            Turn_Kd = value;
            Uart2SendStr("ok turn_kd\r\n");
            break;
        case 'q':
            Turn_KP = value;
            Uart2SendStr("ok turn_kp\r\n");
            break;
        default:
            Uart2SendStr("err cmd\r\n");
            break;
    }
}

/* Configure USART2 baud rate and enable RX interrupt */
void uart2_init(u32 bound)
{
    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = bound;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart2);
    rb_init(&uart2_rx_rb);
    bt_line_len = 0;
    bt_line_active = 0;
    HAL_UART_Receive_IT(&huart2, &uart2_rx_byte, 1);
}

/* Called from HAL_UART_RxCpltCallback in stm32f1xx_it.c */
void USART2_RxCallback(void)
{
    rb_push(&uart2_rx_rb, uart2_rx_byte);
    HAL_UART_Receive_IT(&huart2, &uart2_rx_byte, 1);
}

void Bluetooth_Process(void)
{
    uint8_t byte;

    while (rb_pop(&uart2_rx_rb, &byte)) {
        if (!bt_line_active && (byte == '\r' || byte == '\n')) {
            continue;
        }

        if (bt_line_active || is_ascii_tune_start(byte)) {
            if (byte == '\r' || byte == '\n') {
                bt_line[bt_line_len] = '\0';
                apply_ascii_tune_command(bt_line);
                bt_line_len = 0;
                bt_line_active = 0;
                continue;
            }

            if (bt_line_len < (sizeof(bt_line) - 1u)) {
                bt_line[bt_line_len++] = (char)byte;
                bt_line_active = 1;
            } else {
                bt_line_len = 0;
                bt_line_active = 0;
                Uart2SendStr("err overflow\r\n");
            }
            continue;
        }

        BluetoothCMD((int)byte);
    }
}

void Bluetooth_DebugTask(void)
{
    static uint32_t last_log_ms = 0;

    if (!bt_debug_stream) {
        return;
    }

    if ((HAL_GetTick() - last_log_ms) < 100U) {
        return;
    }

    last_log_ms = HAL_GetTick();
    build_debug_line();
    Uart2SendStr(tx_buf);
}

void BluetoothCMD(int cmd)
{
    bt_manual_control = 0;
    switch (cmd) {
        case 90: Fore=0; Back=0; Left=0; Right=0; break; /* stop */
        case 65: Fore=1; Back=0; Left=0; Right=0; break; /* forward */
        case 72: Fore=1; Back=0; Left=1; Right=0; break; /* forward-left */
        case 66: Fore=1; Back=0; Left=0; Right=1; break; /* forward-right */
        case 71: Fore=0; Back=0; Left=1; Right=0; break; /* left */
        case 67: Fore=0; Back=0; Left=0; Right=1; break; /* right */
        case 69: Fore=0; Back=1; Left=0; Right=0; break; /* backward */
        case 70: Fore=0; Back=1; Left=0; Right=1; break; /* backward-right */
        case 68: Fore=0; Back=1; Left=1; Right=0; break; /* backward-left */
        default: Fore=0; Back=0; Left=0; Right=0; break;
    }
}

u8 Bluetooth_ManualControlActive(void)
{
    return bt_manual_control;
}

void Uart2SendByte(char byte)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&byte, 1, 10);
}

void Uart2SendBuf(char *buf, u16 len)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)buf, len, 100);
}

void Uart2SendStr(char *str)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)str, (uint16_t)strlen(str), 100);
}
