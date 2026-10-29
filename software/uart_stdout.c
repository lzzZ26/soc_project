/*
 * uart_stdout.c - UART0-based stdio retarget for MES50HP SoC
 *
 * UART0 is connected directly to the CMSDK APB subsystem (no GPIO mux).
 *   SIMULATION build : BAUDDIV=16, CTRL[6]=HSTM -> 1 bit per PCLK (fast sim)
 *   BOARD build      : BAUDDIV=5208, 9600 baud @ 50MHz
 */
#include "CMSDK_CM0.h"

#ifdef SIMULATION
#define UART_BAUDDIV 16
#define UART_CTRL    0x43     /* TXEN | RXEN | HSTM */
#else
/* CMSDK baud = PCLK / BAUDDIV.  50e6 / 5208 = 9600 */
#define UART_BAUDDIV 5208
#define UART_CTRL    0x03     /* TXEN | RXEN */
#endif

void UartStdOutInit(void)
{
    CMSDK_UART0->BAUDDIV = UART_BAUDDIV;
    CMSDK_UART0->CTRL    = UART_CTRL;
}

unsigned char UartPutc(unsigned char my_ch)
{
    while (CMSDK_UART0->STATE & 1);   /* wait while TX buffer full */
    CMSDK_UART0->DATA = my_ch;
    return my_ch;
}

unsigned char UartGetc(void)
{
    while ((CMSDK_UART0->STATE & 2) == 0);
    return (unsigned char)CMSDK_UART0->DATA;
}

void UartEndSimulation(void)
{
    UartPutc(0x04);
    while (1);
}
