#include "usart2.h"

u8 Fore = 0, Back = 0, Left = 0, Right = 0;

static uint8_t uart2_rx_byte;

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
    HAL_UART_Receive_IT(&huart2, &uart2_rx_byte, 1);
}

/* Called from HAL_UART_RxCpltCallback in stm32f1xx_it.c */
void USART2_RxCallback(void)
{
    BluetoothCMD((int)uart2_rx_byte);
    HAL_UART_Receive_IT(&huart2, &uart2_rx_byte, 1);
}

void BluetoothCMD(int cmd)
{
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
