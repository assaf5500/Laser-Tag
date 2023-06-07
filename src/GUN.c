/*----------------------------------------------------------------------------
 *  Demo Application for SimpliciTI
 *
 *  L. Friedman
 *  Texas Instruments, Inc.
 *----------------------------------------------------------------------------
 */
/******************************************************************************************

  Copyright 2007-2009 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights granted under
  the terms of a software license agreement between the user who downloaded the software,
  his/her employer (which must be your employer) and Texas Instruments Incorporated (the
  "License"). You may not use this Software unless you agree to abide by the terms of the
  License. The License limits your use, and you acknowledge, that the Software may not be
  modified, copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio frequency
  transceiver, which is integrated into your product. Other than for the foregoing purpose,
  you may not use, reproduce, copy, prepare derivative works of, modify, distribute,
  perform, display or sell this Software and/or its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE PROVIDED “AS IS”
  WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY
  WARRANTY OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
  IN NO EVENT SHALL TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER LEGAL EQUITABLE
  THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES INCLUDING BUT NOT LIMITED TO ANY
  INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST
  DATA, COST OF PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY
  THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
 **************************************************************************************************/

#include "bsp.h"
#include "mrfi.h"
#include "nwk_types.h"
#include "nwk_api.h"
#include "bsp_leds.h"
#include "bsp_buttons.h"
#include <msp430.h>
#include "app_remap_led.h"

#include "mrfi_defs.h"






static void linkFrom(void);

void toggleLED(uint8_t);

void print(char msg[]);

void print_counter(void);


//static          uint8_t  sRxTid = 0;
static          linkID_t sLinkID2 = 0;
static volatile uint8_t  sSemaphore = 0;
/*~~~~~~~~~~~~    OUR GLOBAL VARIABLES    ~~~~~~~~~~~~~~~~~*/


uint8_t mode=0;
uint8_t maxHits,hit_counter=0;
int ii,z,index=0;//,floop;
/* ADC */
int degC, irFlag=0,delay;
volatile long temp;
int results[0];
extern int tempOffset;

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/



/* Rx callback handler */
static uint8_t sRxCallback(linkID_t);

void main (void)
{

    BSP_Init();

    /* If an on-the-fly device address is generated it must be done before the
     * call to SMPL_Init(). If the address is set here the ROM value will not
     * be used. If SMPL_Init() runs before this IOCTL is used the IOCTL call
     * will not take effect. One shot only. The IOCTL call below is conformal.
     */
#ifdef I_WANT_TO_CHANGE_DEFAULT_ROM_DEVICE_ADDRESS_PSEUDO_CODE
    {
        addr_t lAddr;

        createRandomAddress(&lAddr);
        SMPL_Ioctl(IOCTL_OBJ_ADDR, IOCTL_ACT_SET, &lAddr);
    }
#endif /* I_WANT_TO_CHANGE_DEFAULT_ROM_DEVICE_ADDRESS_PSEUDO_CODE */

    /* This call will fail because the join will fail since there is no Access Point
     * in this scenario. But we don't care -- just use the default link token later.
     * We supply a callback pointer to handle the message returned by the peer.
     */
    SMPL_Init(sRxCallback);



    /* turn on LEDs. */
    if (!BSP_LED2_IS_ON())
    {
        toggleLED(2);
    }
    if (!BSP_LED1_IS_ON())
    {
        toggleLED(1);
    }

    //    /* wait for a button press... */
    //    do {
    //        if (BSP_BUTTON1() || BSP_BUTTON2())
    //        {
    //            break;
    //        }
    //    } while (1);


    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    WDTCTL = WDTPW | WDTHOLD;      // stop watchdog timer

    P1DIR |= 0x01;                  // configure P1.0 as output

    P2DIR &= ~0x01;         // make pin P1.2 input (ECHO)
    P2SEL &= 0x0;
    P2IE |= 0x01;           // enable interupt on ECHO pin
    P2DIR |= 0x0E;         // make pin P2.1,P2.2,P2.3 output (External LED's)

    /* Timer A */
    TACCTL0 = CCIE;                 // timer A config
    TACCR0 = 6000;


    /*  UART Interrupt Configuration    */
    BCSCTL1 = CALBC1_1MHZ;                              // setting the DCOCLK to 1 MHz
    DCOCTL = CALDCO_1MHZ;                               // setting the DCOCLK to 1 MHz

    IFG2 &= 0xFC;                                       //  "Reset" Flag interrupt Rx & Tx
    IE2 |= UCA0RXIE;                                    // Interrupt Rx & Tx Enable
    for(z=4;z>0;z--)
    {
        P2OUT ^= 0x02;              // toggle P1.0
        P2OUT ^= 0x02;              // toggle P1.0
        P2OUT ^= 0x02;              // toggle P1.0
        for(ii=10000; ii>0; ii--);     // delay
        P2OUT ^= 0x04;              // toggle P1.0
        P2OUT ^= 0x04;              // toggle P1.0
        P2OUT ^= 0x04;              // toggle P1.0
        for(ii=10000; ii>0; ii--);     // delay
        P2OUT ^= 0x08;              // toggle P1.0
        P2OUT ^= 0x08;              // toggle P1.0
        P2OUT ^= 0x08;              // toggle P1.0
        for(ii=10000; ii>0; ii--);     // delay
    }
    P2OUT &= 0xF1;
    __bis_SR_register(GIE);
    //    print("Temp: ");

    /*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
    /* never coming back... */
    linkFrom();

    /* but in case we do... */
    while (1) ;
}

static void linkFrom()
{
    uint8_t     msg[2];//, tid = 0;

    /* Turn off one LED so we can tell the device is now listening.
     * Received messages will toggle the other LED.
     */
    toggleLED(1);//Green LED off

    /* listen for link forever... */
    while (1)
    {
        if (SMPL_SUCCESS == SMPL_LinkListen(&sLinkID2))
        {
            break;
        }
        /* Implement fail-to-link policy here. otherwise, listen again. */
    }
    toggleLED(1);
    toggleLED(2);
    for(delay=10000; delay>0; delay--);
    toggleLED(1);
    toggleLED(2);
    for(delay=10000; delay>0; delay--);
    /* turn off LEDs. */
    if (BSP_LED2_IS_ON())
    {
        toggleLED(2);
    }
    if (BSP_LED1_IS_ON())
    {
        toggleLED(1);
    }

    /* turn on LED1 on the peer in response to receiving a frame. */
    //    *msg = 0x01;

    /* turn on RX. default is RX off. */
    SMPL_Ioctl( IOCTL_OBJ_RADIO, IOCTL_ACT_RADIO_RXON, 0);

    while (1)
    {
        /* Wait for a frame to be received. The Rx handler, which is running in
         * ISR thread, will post to this semaphore allowing the application to
         * send the reply message in the user thread.
         */
        if (sSemaphore)
        {
            if(mode == 2)//server wants hit_counter update -> send "SCORE"
            {
                toggleLED(1);
                *(msg+1) = hit_counter;
                *msg = 0x01;
            }
            else if(mode == 3)//server sent "RESET" -> send "READY"
            {
                *msg = 0x02;
                *(msg+1) = 0;
            }
            else if(mode == 0xFF)//GOT "KILL" -> SEND "DEAD"
            {
                *msg = 0xFF;
                *(msg+1) = 0xFF;
            }

            //            *(msg+1) = ++tid;
            SMPL_Send(sLinkID2, msg, 2);

            /* Reset semaphore. This is not properly protected and there is a race
             * here. In theory we could miss a message. Good enough for a demo, though.
             */
            sSemaphore = 0;
            if(mode == 0xFF){ _BIS_SR(LPM4_bits); }
        }
        if ((P2IFG & 0x01) && (irFlag == 1))
        {
            hit_counter++;
            if(hit_counter > maxHits)
            {
                for(z=4;z>0;z--)
                {
                    P2OUT ^= 0x02;              // toggle P1.0
                    P2OUT ^= 0x02;              // toggle P1.0
                    P2OUT ^= 0x02;              // toggle P1.0
                    for(ii=10000; ii>0; ii--);     // delay
                    P2OUT ^= 0x04;              // toggle P1.0
                    P2OUT ^= 0x04;              // toggle P1.0
                    P2OUT ^= 0x04;              // toggle P1.0
                    for(ii=10000; ii>0; ii--);     // delay
                    P2OUT ^= 0x08;              // toggle P1.0
                    P2OUT ^= 0x08;              // toggle P1.0
                    P2OUT ^= 0x08;              // toggle P1.0
                    for(ii=10000; ii>0; ii--);     // delay
                    P2OUT &= 0xF1;
                }
                hit_counter = hit_counter;
            }
            P2OUT &= 0xFD;
            P2OUT ^= 0x02;              // toggle P1.0
            for(ii=20000; ii>0; ii--);     // delay
            P2OUT &= 0xFD;
            P1OUT ^= 0x01;              // toggle P1.0
            P2IFG &= 0;
            //            irFlag = 0;
            //            TACTL = TASSEL_1 + MC_1;        // choose clock (ACLK) + UP mode
        }
    }
}

void toggleLED(uint8_t which)
{
    if (1 == which)
    {
        BSP_TOGGLE_LED1();
    }
    else if (2 == which)
    {
        BSP_TOGGLE_LED2();
    }
    return;
}

/* handle received messages */
static uint8_t sRxCallback(linkID_t port)
{
    uint8_t msg[2], len;


    /* is the callback for the link ID we want to handle? */
    if (port == sLinkID2)
    {
        /* yes. go get the frame. we know this call will succeed. */
        if ((SMPL_SUCCESS == SMPL_Receive(sLinkID2, msg, &len)) && len)
        {
            mode = msg[0];
            if(mode == 2)//"HIT_COUNT" - server wants hit_counter update
            {
                /* Post to the semaphore to let application know so it sends the reply */
                sSemaphore = 1;
            }
            else if(mode == 3)//"RESET" - we have a winner but they want to play again
            {
                P2OUT |= 0x08;
                hit_counter = 0;
                irFlag = 0;
                sSemaphore = 1;
            }
            else if(mode == 1)//"START" - communication established, let's start play!
            {
                maxHits = msg[1];
                irFlag = 1;/* enable IR interrupt enable *///~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
                P2OUT &= 0xF1;
                toggleLED(1);
                P2OUT ^= 0x04;              // toggle P1.0
                for(delay=10000; delay>0; delay--);
                toggleLED(1);
                P2OUT ^= 0x04;              // toggle P1.0
                for(delay=10000; delay>0; delay--);
                toggleLED(1);
                P2OUT ^= 0x04;              // toggle P1.0
                for(delay=10000; delay>0; delay--);
                toggleLED(1);
                P2OUT ^= 0x04;              // toggle P1.0
                for(delay=10000; delay>0; delay--);
                toggleLED(2);
                P2OUT ^= 0x04;              // toggle P1.0
                for(delay=10000; delay>0; delay--);
                toggleLED(2);
                P2OUT ^= 0x04;              // toggle P1.0
                for(delay=10000; delay>0; delay--);
                toggleLED(2);
                P2OUT ^= 0x04;              // toggle P1.0
                for(delay=10000; delay>0; delay--);
                toggleLED(2);
                P2OUT &= 0xFB;              // toggle P1.0
                for(delay=10000; delay>0; delay--);
            }
            else if(mode == 0xFF)//"KILL" - "GameOver" im done.
            {
                sSemaphore = 1;
            }
            else{}//for future purposes

            /* drop frame. we're done with it. */
            return 1;
        }
    }
    /* keep frame for later handling */
    return 0;
}

#pragma vector = TIMERA0_VECTOR    // interrupt "handler"
__interrupt void Timer_A (void)
{
    //    //    P1OUT ^= 0x01;              // toggle P1.0
    //    irFlag = 1;
    //    TACTL = TASSEL_1 + MC_0;        // choose clock (ACLK) + STOP mode
}

BSP_ISR_FUNCTION( BSP_GpioPort1Isr, PORT2_VECTOR )
{
    if(P2IFG & 0x01)
    {
//        P1OUT ^= 0x01;              // toggle P1.0
        P2IFG &= 0;
    }


    MRFI_GpioIsr();
}
