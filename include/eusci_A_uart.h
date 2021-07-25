#ifndef eusci_A_uart_h
#define eusci_A_uart_h  1

/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>

#define SYNCC         16

/**
 *  Initialize back-channel UART using default P1.2, P1.3.
 */
void init_UART (uint32_t moduleInstance, const eUSCI_UART_ConfigV1 *config);

/**
 *  Initialize back-channel UART using default P1.2, P1.3 with receive interrupt.
 *  You must implement void EUSCIA0_IRQHandler (void)
 */
void init_UART_w_recv_isr (uint32_t UART, const eUSCI_UART_ConfigV1 *config);

/**
 *  Transmit nul-terminated string across UART.
 *  adds a carriage-return if \n character sent.
 */
void uart_tx_str (uint32_t moduleInstance, const char *str);

#endif
