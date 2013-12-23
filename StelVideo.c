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
#include "inc/hw_ints.h"
#include "inc/hw_ssi.h"
#include "driverlib/cpu.h"
#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"
#include "driverlib/ssi.h"
#include "driverlib/udma.h"

/*** Horizontal timings, using 80 MHz timer clock ***/

/*
 * 640  480     60 Hz   31.475 kHz:
 * ModeLine "640x480" 25.18 640 656 752 800 480 490 492 525 -HSync -VSync
 */

/* Number of timer cycles per horizontal cycle */
#define H_TOTAL 2542
/* Length of horizontal sync pulse */
#define H_SYNCLEN 305
/* Cycles between end of horizontal sync and start of image */
#define H_BACKPORCH 153
/* Define if sync is normally high */
#define H_SYNCINVERT

/*** Pixel timings, which are independent ***/

/* Pixel clock in MHz */
#define PIXEL_CLOCK 20000000
/* Pixels, must be multiple of 16 and more than 8*16 */
#define H_PIXELS (31*16)


/*** Vertical timings, in terms of line numbers, like a modeline ***/

#define V_DISPLINES 480
#define V_SYNCSTART 490
#define V_SYNCEND 492
#define V_TOTAL 525
#define V_SYNCINVERT

/*** Frame buffer ***/

/* Include an image from an external file for testing and demonstration. */
/* #include "fb.h" */

/*** Text Mode ***/

/* Next line of pixels for text mode is decoded into alternating buffers. */
unsigned char linebuf[2][H_PIXELS/8];

/* hexdump -ve '8/1 "0x%02x," "\n"' 8x16.chr */
#include "text_font.h"

unsigned char text_screen[(H_PIXELS/8) * (V_DISPLINES/16)];

/*** Interrupt handler at start of line ***/

void Timer0AIntHandler(void)
{
    /* States for vertical state machine */
    static enum {
        VERTM_IMAGE = 0,    /* Outputting lines of the image */
        VERTM_PRESYNC = 1,  /* After image, waiting for vertical sync start */
        VERTM_SYNC = 2,     /* Outputting vsync, waiting for its end */
        VERTM_POSTSYNC = 3  /* Waiting for end of cycle and start of image */
    } vertstate = VERTM_PRESYNC;

    /* Current line */
    static int vertidx = V_DISPLINES;

    TimerIntClear(TIMER0_BASE, TIMER_CAPA_EVENT);

    switch (vertstate) {
    case VERTM_IMAGE:
        if (vertidx < V_DISPLINES) {
            /* unsigned char *ucSourceBuffer = &fb[H_PIXELS/8*vertidx]; */
            unsigned char *ucSourceBuffer = &linebuf[vertidx&1][0];
            /* Set up timer triggered DMA for initial FIFO fill */
            uDMAChannelTransferSet(UDMA_CHANNEL_TMR0B | UDMA_PRI_SELECT,
                                   UDMA_MODE_AUTO, ucSourceBuffer,
                                   (void *)(SSI0_BASE + SSI_O_DR), 8);
            uDMAChannelEnable(UDMA_CHANNEL_TMR0B);

            /* Set up SSI controlled triggered DMA for the rest, but don't
             * enable yet. Timer interrupt triggered by completion of
             * above DMA transfer will enable it. */
            uDMAChannelTransferSet(UDMA_CHANNEL_SSI0TX | UDMA_PRI_SELECT,
                                   UDMA_MODE_BASIC, ucSourceBuffer+16,
                                   (void *)(SSI0_BASE + SSI_O_DR),
                                   H_PIXELS/16 - 8);
        } else {
            vertstate = VERTM_PRESYNC;
        }
        break;
    case VERTM_PRESYNC:
        if (vertidx < V_SYNCSTART) break;
        vertstate = VERTM_SYNC;
#ifdef V_SYNCINVERT
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 0);
#else
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, GPIO_PIN_3);
#endif
        break;
    case VERTM_SYNC:
        if (vertidx < V_SYNCSTART) break;
        vertstate = VERTM_POSTSYNC;
#ifdef V_SYNCINVERT
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, GPIO_PIN_3);
#else
        GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 0);
#endif
        break;
    case VERTM_POSTSYNC:
        if (vertidx < (V_TOTAL-1)) break;
        vertstate = VERTM_IMAGE;
        vertidx = -1;
        break;
    }
    vertidx++;

    /* Text mode character generator */
    if (vertstate == VERTM_IMAGE) {
        /* Current line of text. */
        unsigned char *charsrc = &text_screen[(vertidx >> 4) * (H_PIXELS/8)];
        /* Offset into font array corresponding to pixel line in font. */
        const unsigned char *fontsrc = &text_font[vertidx & 15];
        /* Which of two line buffers is currently being written. */
        unsigned char *dest = &linebuf[vertidx&1][0];
        int i;

        /* Make use of time before DMA triggering, but make sure CPU
         * is idle before DMA starts so that there is no jitter.
         */
#define CHARGEN_B4WFI 12
        for (i = 0; i < CHARGEN_B4WFI; i += 2) {
            dest[i] = fontsrc[charsrc[i + 1] << 4];
            dest[i + 1] = fontsrc[charsrc[i] << 4];
        }

        if (vertidx != 0) {
            CPUwfi();
        }

        for (i = CHARGEN_B4WFI; i < H_PIXELS/8; i += 2) {
            dest[i] = fontsrc[charsrc[i + 1] << 4];
            dest[i + 1] = fontsrc[charsrc[i] << 4];
        }
    }
}

/*** Interrupt handler for when timer triggered DMA completes ***/

void Timer0BIntHandler(void)
{
    /* Start DMA for the rest of the line */
    uDMAChannelEnable(UDMA_CHANNEL_SSI0TX);
}

volatile int timersync = 0;

/*** Interrupt handler used once to synchronize timers ***/

void Timer0BIntSync(void)
{
    if (timersync) {
        TimerLoadSet(TIMER0_BASE, TIMER_B, H_TOTAL-1);
        TimerIntDisable(TIMER0_BASE, TIMER_TIMB_TIMEOUT);
        TimerIntClear(TIMER0_BASE, TIMER_TIMB_TIMEOUT);
        TimerIntUnregister(TIMER0_BASE, TIMER_B);
        TimerIntRegister(TIMER0_BASE, TIMER_B, Timer0BIntHandler);
    } else {
        /* Handling spurious interrupts, maybe only needed
         * if device was not reset
         */
        TimerIntClear(TIMER0_BASE,
                      TIMER_CAPB_EVENT | TIMER_CAPB_MATCH
                      | TIMER_TIMB_TIMEOUT | TIMER_RTC_MATCH
                      | TIMER_CAPA_EVENT | TIMER_CAPA_MATCH
                      | TIMER_TIMA_TIMEOUT);
    }
}

#pragma DATA_ALIGN(ucDMAControlTable, 1024)
unsigned char ucDMAControlTable[1024];

/*** Entry point ***/

void main(void)
{
    SysCtlDelay(2000000);
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ |
                   SYSCTL_OSC_MAIN);

    /*** Initialize DMA controller ***/

    SysCtlPeripheralEnable(SYSCTL_PERIPH_UDMA);
    uDMAEnable();
    uDMAControlBaseSet(&ucDMAControlTable[0]);

    /*** Set up DMA for initial timer triggered load ***/

    uDMAChannelAttributeDisable(UDMA_CHANNEL_TMR0B, UDMA_ATTR_USEBURST |
                                UDMA_ATTR_ALTSELECT | UDMA_ATTR_REQMASK);
    uDMAChannelAttributeEnable(UDMA_CHANNEL_TMR0B, UDMA_ATTR_HIGH_PRIORITY);
    uDMAChannelAssign(UDMA_CH19_TIMER0B);
    uDMAChannelControlSet(UDMA_CHANNEL_TMR0B | UDMA_PRI_SELECT,
                          UDMA_SIZE_16 | UDMA_SRC_INC_16 |
                          UDMA_DST_INC_NONE | UDMA_ARB_8);

    /*** Set up DMA for the rest under SSI control ***/

    uDMAChannelAttributeDisable(UDMA_CHANNEL_SSI0TX, UDMA_ATTR_USEBURST |
                                UDMA_ATTR_ALTSELECT | UDMA_ATTR_REQMASK);
    uDMAChannelAttributeEnable(UDMA_CHANNEL_SSI0TX, UDMA_ATTR_HIGH_PRIORITY);
    uDMAChannelAssign(UDMA_CH11_SSI0TX);
    uDMAChannelControlSet(UDMA_CHANNEL_SSI0TX | UDMA_PRI_SELECT,
                          UDMA_SIZE_16 | UDMA_SRC_INC_16 |
                          UDMA_DST_INC_NONE | UDMA_ARB_4);

    /*** Set up SSI0 for graphic output ***/

    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    GPIOPinConfigure(GPIO_PA5_SSI0TX);
    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5);
    GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_5, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);
    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5);

    SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_TI,
                          SSI_MODE_MASTER, PIXEL_CLOCK, 16);

    SSIEnable(SSI0_BASE);

    SSIDMAEnable(SSI0_BASE, SSI_DMA_TX);

    /*** Set up PB3 for vertical sync ***/

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

#ifdef V_SYNCINVERT
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, GPIO_PIN_3);
#else
    GPIOPinWrite(GPIO_PORTB_BASE, GPIO_PIN_3, 0);
#endif
    GPIOPinTypeGPIOOutput(GPIO_PORTB_BASE, GPIO_PIN_3);
    GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_3 | GPIO_PIN_6,
                     GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);

    /*** Set up timer 0 for horizontal timing ***/

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);

    TimerConfigure(TIMER0_BASE, TIMER_CFG_SPLIT_PAIR |
                   TIMER_CFG_A_PWM | TIMER_CFG_B_PERIODIC_UP);
    TimerLoadSet(TIMER0_BASE, TIMER_A, H_TOTAL-1);
    TimerEnable(TIMER0_BASE, TIMER_BOTH);

    /* Timer 0A outputs horizontal sync,
     * starting at reload and ending at match
     */

#ifdef H_SYNCINVERT
    TimerControlLevel(TIMER0_BASE, TIMER_A, true);
#else
    TimerControlLevel(TIMER0_BASE, TIMER_A, false);
#endif

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    GPIOPinConfigure(GPIO_PB6_T0CCP0);
    GPIOPinTypeTimer(GPIO_PORTB_BASE, GPIO_PIN_6);

    TimerMatchSet(TIMER0_BASE, TIMER_A, H_TOTAL-H_SYNCLEN-1);

    /* Timer 0A triggers an interrupt at the start of the sync pulse.
     * It is used for setting up DMA, keeping track of vertical position,
     * and vertical sync.
     */

    TimerIntRegister(TIMER0_BASE, TIMER_A, Timer0AIntHandler);
    IntPrioritySet(INT_TIMER0A, 0x20);
    IntMasterEnable();
    /* Note that in PWM mode the timeout interrupt can't be used,
     * and capture interrupts are needed.
     */
    TimerIntEnable(TIMER0_BASE, TIMER_CAPA_EVENT);
    /* This should happen at the start of the sync signal. According to
     * documentation, that should always be POS_EDGE, but with inverted
     * sync POS_EDGE happens at the match instead, as if the
     * documentation is wrong.
     */
    TimerControlEvent(TIMER0_BASE, TIMER_A, TIMER_EVENT_NEG_EDGE);
    IntEnable(INT_TIMER0A);

    /* Use timer 0B interrupt once to synchronize A and B timers and set
     * horizontal position of picture. The interrupt will uninstall itself
     * and set up the timer for DMA triggering.
     */

    TimerLoadSet(TIMER0_BASE, TIMER_B, H_SYNCLEN+H_BACKPORCH);
    TimerIntRegister(TIMER0_BASE, TIMER_B, Timer0BIntSync);
    IntPrioritySet(INT_TIMER0B, 0x00);
    TimerIntEnable(TIMER0_BASE, TIMER_TIMB_TIMEOUT);
    IntEnable(INT_TIMER0B);

    TimerSynchronize(TIMER0_BASE, 0);
    TimerSynchronize(TIMER0_BASE, TIMER_0A_SYNC | TIMER_0B_SYNC);
    timersync = 1;

    /*** Temporary main ***/

    /* Fill text screen */
    int i;
    for (i = 0; i < sizeof(text_screen); i++) {
        text_screen[i] = i & 0xFF;
    }

    /* This just indicates that the device is running and provides a
     * very crude indication of the amount of free CPU time.
     */

#define RED_LED   GPIO_PIN_1
#define BLUE_LED  GPIO_PIN_2
#define GREEN_LED GPIO_PIN_3

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, RED_LED | BLUE_LED | GREEN_LED);

    while(1) {
        GPIOPinWrite(GPIO_PORTF_BASE,
                     RED_LED | BLUE_LED | GREEN_LED, RED_LED | GREEN_LED);
        SysCtlDelay(2000000);
        GPIOPinWrite(GPIO_PORTF_BASE,
                     RED_LED | BLUE_LED | GREEN_LED, BLUE_LED);
        SysCtlDelay(2000000);
    }
}
