/*
 * StelVideo: video signal generator for the Stellaris LaunchPad.
 * Copyright (C) 2013 Boris Gjenero <boris.gjenero@gmail.com>
 *
 * StelVideo is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * StelVideo is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with StelVideo.  If not, see <http://www.gnu.org/licenses/>.
 */

/* #define PART_LM4F120H5QR */

#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "inc/hw_uart.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"

#include "StelVideo.h"

void uart_init(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);

    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    UARTConfigSetExpClk(UART0_BASE, SysCtlClockGet(), 38400,
                        (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE |
                         UART_CONFIG_PAR_NONE));
}

void uart_run(RoteTerm *term)
{
    UARTCharPut(UART0_BASE, '>');

    while (1) {
        char c;
        c = UARTCharGet(UART0_BASE);
        UARTCharPut(UART0_BASE, c);
        rote_vt_inject(term, &c, 1);
    }
}
