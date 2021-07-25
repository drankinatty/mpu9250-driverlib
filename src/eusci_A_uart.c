#include "eusci_A_uart.h"

/* 3MHz SMCLK eUSCI at 115200 baud */
const eUSCI_UART_ConfigV1 uartConfig =
{
  EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
  1,                                       // BRDIV  = 1
  10,                                      // UCxBRF = 10
  0,                                       // UCxBRS = 0
  EUSCI_A_UART_NO_PARITY,                  // No Parity
  EUSCI_A_UART_LSB_FIRST,                  // LSB First
  EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
  EUSCI_A_UART_MODE,                       // UART mode
  EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION,  // Oversampling
  EUSCI_A_UART_8_BIT_LEN                   // 8 bit data length
};

/* 12MHz SMCLK eUSCI at 115200 baud */
/*
const eUSCI_UART_ConfigV1 uartConfig =
{
  EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
  6,                                       // BRDIV  = 6
  8,                                       // UCxBRF = 8
  32,                                      // UCxBRS = 32
  EUSCI_A_UART_NO_PARITY,                  // No Parity
  EUSCI_A_UART_LSB_FIRST,                  // LSB First
  EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
  EUSCI_A_UART_MODE,                       // UART mode
  EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION,  // Oversampling
  EUSCI_A_UART_8_BIT_LEN                   // 8 bit data length
};
*/

char received[SYNCC] = {0};

/**
 *  Initialize back-channel UART using default P1.2, P1.3.
 */
void init_UART (uint32_t moduleInstance, const eUSCI_UART_ConfigV1 *config)
{
    /* Selecting P1.2 and P1.3 in UART mode */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin (GPIO_PORT_P1,
                             GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);

    /* Configuring UART Module */
    MAP_UART_initModule (moduleInstance, config);

    /* Enable UART module */
    MAP_UART_enableModule (moduleInstance);
}

/**
 *  Initialize back-channel UART using default P1.2, P1.3 with receive interrupt.
 */
void init_UART_w_recv_isr (uint32_t moduleInstance, const eUSCI_UART_ConfigV1 *config)
{
    /* initialize UART */
    init_UART (moduleInstance, config);

    /* Enabling interrupts */
    MAP_UART_enableInterrupt (moduleInstance, EUSCI_A_UART_RECEIVE_INTERRUPT);

    /* Enable interrupt handler */
    switch (moduleInstance) {
      case EUSCI_A0_BASE: MAP_Interrupt_enableInterrupt (INT_EUSCIA0); break;
      case EUSCI_A1_BASE: MAP_Interrupt_enableInterrupt (INT_EUSCIA1); break;
      case EUSCI_A2_BASE: MAP_Interrupt_enableInterrupt (INT_EUSCIA2); break;
      case EUSCI_A3_BASE: MAP_Interrupt_enableInterrupt (INT_EUSCIA3); break;
    }
}


/**
 *  Transmit nul-terminated string across UART.
 *  adds a carriage-return if \n character sent.
 */
void uart_tx_str (uint32_t moduleInstance, const char *str)
{
    int i = 0;

    for (; str[i]; i++) {
        if (str[i] == '\n')
            MAP_UART_transmitData (moduleInstance, '\r');
        MAP_UART_transmitData (moduleInstance, str[i]);
    }
}

/* EUSCI A0 UART ISR - Echoes data back to PC host */
// void EUSCIA0_IRQHandler (void)
// {
//     uint32_t status = MAP_UART_getEnabledInterruptStatus (EUSCI_A0_BASE);
//
//     if (status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
//     {
//         uint_fast8_t recvd = MAP_UART_receiveData (EUSCI_A0_BASE);
//
//         if (recvd == '\r') {    /* echo serial input */
//             MAP_UART_transmitData (EUSCI_A0_BASE, '\n');
//             MAP_UART_transmitData (EUSCI_A0_BASE, '\r');
//         }
// //        else {    /* SMCLK min of 12MHz for reliable tx/rx simultaneously */
// //            MAP_UART_transmitData (EUSCI_A0_BASE, recvd);
// //        }
//
//     /* add input handler here */
//     }
// }
