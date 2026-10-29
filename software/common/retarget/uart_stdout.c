/*
 * UART0 (CP2102 USB-UART) retarget
 */
#include "CMSDK_CM0.h"

void UartStdOutInit(void)
{
  CMSDK_UART0->BAUDDIV = 326;  // 50MHz / (16 * 9600)
  CMSDK_UART0->CTRL    = 0x03; // TXEN=1, RXEN=1, normal mode
  
  // Enable UART0_TX on P1[1] via alternate function
  CMSDK_GPIO1->ALTFUNCSET = (1u << 1);
}

unsigned char UartPutc(unsigned char c)
{
  while ((CMSDK_UART0->STATE & 1));
  CMSDK_UART0->DATA = c;
  return c;
}

unsigned char UartGetc(void)
{
  while ((CMSDK_UART0->STATE & 2) == 0);
  return CMSDK_UART0->DATA;
}

void UartEndSimulation(void)
{
  UartPutc(0x04);
  while (1);
}
