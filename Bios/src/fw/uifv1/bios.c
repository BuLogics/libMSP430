/**
* \ingroup MODULBIOS
*
* \file bios.c
*
* \brief Handle hardware dependent functions
*
*/
/*
 * Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/ 
 * 
 * 
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permitted provided that the following conditions 
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright 
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the 
 *    documentation and/or other materials provided with the   
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

//!<ul>
//!<li>system init</li>
//!<li>data receiving and sending from/to TUSB</li>
//!<li>LED handling</li>
//!<li>global timer</li>
//!<li>rx/tx with handshake or DMA</li>
//!</ul>

#include "hw_compiler_specific.h"
#include "bios.h"
#include "../v3_0p.h"
#include "../stream.h"
#include <stdlib.h>
#include <string.h>

#define CORE_VERSION 0x000B

typedef short (*HalFuncInOut)(unsigned short);

//! \brief Identifier for safecore
//! \details this words indicate a valid bios
const unsigned short identify_[3] @ "IDENTIFY" = {0x000F,0x2112,0xFEDF};
REQUIRED(identify_)
//! \brief version of bios code 
const unsigned short core_version_ @ "COREVERSION" = CORE_VERSION;
RO_PLACEMENT_NO_INIT volatile const unsigned short safe_core_version_ @ "SAFECOREVERSION";

//! \brief chars for flow control
const unsigned char BIOS_DLE_CHAR  = 0x10;
//! \brief chars for flow control
const unsigned char BIOS_XON_CHAR  = 0x11;
//! \brief chars for flow control
const unsigned char BIOS_XOFF_CHAR = 0x13;

// protoypes
short biosLedOn(unsigned char no);
short biosLedOff(unsigned char no);
short biosLedBlink(unsigned char no,unsigned short time);
short biosLedFlash(unsigned char no,unsigned short time);
void  biosInitSystem (void);
void  biosmainLoop(void);
void  biosGlobalError(void);
void  biosInitCom (unsigned long baudrate);
void  biosUsbTxClear(void);
void  biosUsbTxError(void);
void  biosUsbRxClear(void);
void  biosUsbRxError(unsigned short code);
INTERRUPT_PROTO void dummyIsr (void);
short biosHalInterfaceInit(void);
short biosHalInterfaceClear(void);
INTERRUPT_PROTO void usbRxIsr (void);
INTERRUPT_PROTO void timerB1Isr (void);
INTERRUPT_PROTO void usbTxIsr (void);
INTERRUPT_PROTO void nmiIsr (void);
INTERRUPT_PROTO void watchdog_timer(void);

//! \brief Buffer for the RX cannel
//! \details actual two buffers used, one for working, the second to catch char which receivend before
//! the cts signal have an effect.
volatile BiosRxRecord bios_rx_record_ = {0, 0, 0, 0, {0,0}, {0,0}};
//! \brief Buffer for the TX cannel
//! \details only one buffer in uif. There is no need for a second. The dll play ping pong with the uif
volatile BiosTxRecord bios_tx_record_ = {0, 0, {0}, {0}, {0}, {0}, {0}, NULL, 0 };

//! \brief Timer for RX and TX timeouts
//! \details timebase 10ms
volatile BiosGlobalTimer bios_global_timer_[BIOS_TIMER_COUNT];
//! \brief uif is under control of the eg. workbench
//! \details \li 0, not under control, reset by EXCEPTION NULL
//! \li 1, set in v3opRx with the first valid command
char bios_wb_control_ = 0;

//! \brief count of LED from uif
#define BIOS_LED_COUNT 2

//! \brief values for (re)set LEDs
const unsigned char BIOS_LED_OFF   = 0x00;
//! \brief values for (re)set LEDs
const unsigned char BIOS_LED_ON    = 0x01;
//! \brief values for (re)set LEDs
const unsigned char BIOS_LED_BLINK = 0x02;

//! \brief for LED driving
struct _BiosLedTimer_
{
  unsigned char  mode;
  unsigned short load;
  unsigned short counter;
  unsigned char  *addr;
  unsigned char bit;
};
typedef struct _BiosLedTimer_ BiosLedTimer;

//! \brief for LED driving
volatile BiosLedTimer bios_leds_[BIOS_LED_COUNT];

//! \brief the LED from the EASY :-)
//! \details The EASY haven't LEDs, this var is a replacment for the port to catch
//! the LED driving
unsigned char bios_led_dummy_;

//! \brief Hold the crystal frequency
//! \details will set in biosInitSystem depended for the detected hardware
//! will used eg. from timers and UART
unsigned long crystal_;

//! \brief modulation register value
//! \details will set in biosInitSystem depended for the detected hardware
unsigned char baudmod_ = 0;

//! \brief flags for hardware features
//! \details will set in biosInitSystem depended for the detected hardware
//! \li bit 0 -> handshake xon/xoff = 1, hw handshake = 0
//! \li bit 1 -> 4 wire
//! \li bit 2 -> 2 wire
unsigned long bios_device_flags_ = 0;

//! \brief hardware version
//! \details will set in biosInitSystem depended for the detected hardware
unsigned short bios_info_hw_0_;

//! \brief hardware version
//! \details will set in biosInitSystem depended for the detected hardware
unsigned short bios_info_hw_1_;

//! \brief gets the received char
unsigned char bios_rx_char_;

//! \brief dummy to switch of XON/XOFF in easy
const unsigned char bios_rx_char_always_0 = 0;

//! \brief points to bios_rx_char_ if XON/XOFF is active
//! \details on EASY it points to bios_rx_char_always_0 to disable XON/XOFF functionality
//! will set in biosInitSystem depended for the detected hardware
unsigned char *bios_rx_char_ptr_;

//! \brief Flag for XON/XOFF status
//! \li 0, TX can send
//! \li 1, TX must not be send
//! \details between receiving xoff and sending chars is a delay (by program flow). This 
//! is not a problem, because the TUSB has enougth reserve in the buffer.
unsigned char bios_xoff_ = 0;

//! \brief pointer to active TX flow control
//! \details \li on EASY it points to the PORT with the handshakesignals
//! \li on uif it points to the (software)flag bios_xoff_
//! will set in biosInitSystem depended for the detected hardware
unsigned char *bios_xoff_ptr_ = &bios_xoff_;

//! \brief Flag for error indication
//! \details \li 1, an error message is to send. Set by every function which can generate an error
//! \li 0, reset in biosmainLoop, if error message was send
volatile char  bios_rx_err_set_ = 0;

//! \brief Message ID from received message which provoke the error.
volatile unsigned char bios_rx_err_id_ = 0;

//! \brief error code
//! \li EXCEPTION_NOT_IMPLEMENT_ERR 	0x8001
//! \li EXCEPTION_MSGID_ERR	        0x8002
//! \li EXCEPTION_CRC_ERR	        0x8003
//! \li EXCEPTION_RX_TIMEOUT_ERR	0x8004
//! \li EXCEPTION_TX_TIMEOUT_ERR	0x8005
//! \li EXCEPTION_RX_OVERFLOW_ERR     	0x8006
//! \li EXCEPTION_TX_NO_BUFFER	        0x8007
//! \li EXCEPTION_COM_RESET	        0x8008
//! \li EXCEPTION_RX_NO_BUFFER	        0x8009
//! \li EXCEPTION_RX_TO_SMALL_BUFFER	0x800A
//! \li EXCEPTION_RX_LENGTH	        0x800B
//! \li HAL specific exceptions	0xFFFF to 0xFF00 (-1 to -255)
volatile unsigned short bios_rx_err_code_;

//! \brief Pointer to additional information
//! \details if an error provide more information about the error
volatile unsigned short *bios_rx_err_payload_ = NULL;

//! \brief Stop RX
//! \details set CTS to TUSB and break RX timeout timer
#define biosSetCts() {P1OUT |= (1 << 6);  bios_global_timer_[0].state |= BIOS_TIMER_BREAK;}

//! \brief release (restart) RX
//! \details reset CTS to TUSB and RX timeout timer running
#define biosResetCts() {P1OUT &= ~(1 << 6);  bios_global_timer_[0].state &= ~BIOS_TIMER_BREAK;}

//! \brief pin (from TUSB) for hardware handshake for TX cannel
//! \details also used as TRUE value for bios_xoff_
#define BIOS_HARD_RTS_BIT 3

//! \brief System start up
//! \details and detection of UIF or EASY.
//! For detection XT2 are started. If XT2 faults it is a UIF
void biosInitSystem (void)
{
    unsigned short i, j;

    // settings from old project
    // won't work without it
    P1DIR = 0;  P2DIR = 0;  P3DIR = 0;    // Reset all ports direction to be inputs
    P4DIR = 0;  P5DIR = 0;  P6DIR = 0;
    
    P1SEL = 0;  P2SEL = 0;  P3SEL = 0;    // Reset all ports selection
    P4SEL = 0;  P5SEL = 0;  P6SEL = 0;
    
    P1OUT = 0;  P2OUT = 0;  P3OUT = 0;    //!<Reset all port output registers
    P4OUT = 0;  P5OUT = 0;  P6OUT = 0;
    
    
    BCSCTL1 &= ~XT2OFF;      // XT2 on
    
    for(j = 0; (j < 0xFF) && ((IFG1 & OFIFG) != 0); j++) // OSCFault flag still set?
    {
        IFG1 &= ~OFIFG;               // Clear OSCFault flag
        for (i = 0xFF; i > 0; i--) __no_operation();      // Time for flag to set
    }
    if((IFG1 & OFIFG) == 0) // XT2 found, it can be only on EASY
    { // E2
        // drive TUSB3410 reset low
        P4DIR |= BIT6;
        IFG1 &= ~OFIFG;                  // Clear OSCFault flag again
        BCSCTL2 |= SELM1 + SELS;         // MCLK = XT2, provide clock for TUSB
        // setup TUSB3410 clock
        P5DIR |= BIT5;
        P5SEL |= BIT5;
        // here setup E2 specific stuff
        crystal_ = CRYSTAL_E2;
        baudmod_ = BAUDMOD_E2;
        // Port1
        //  P1.0 -> 
        //  P1.1 -> 
        //  P1.2 <- 
        //  P1.3 <- URTS
        //  P1.4 <- UDTR
        //  P1.5 -> UDSR
        //  P1.6 -> UCTS
        //  P1.7 <- 
        // init handshake lines from MSP430 -> TUSB3410
        P1OUT &= ~BIT5;          //!<drive both handshake signals low (BIT5 = UDSR, BIT6 = UCTS) high
        P1OUT |= BIT6;  // set CTS to block receiving anything from TUSB (RX overflow on start up problem)
        P1DIR |=  (BIT5+BIT6);
        // Port2
        //  P2.0 ->
        //  P2.1 ->
        //  P2.2 ->
        //  P2.3 ->
        //  P2.4 ->
        //  P2.5 -> SELT
        //  P2.6 -> TGTRST
        //  P2.7 ->
        P2DIR =  0;          
        // Port3
        //  P3.0 <-
        //  P3.1 <-
        //  P3.2 <-
        //  P3.3 <-
        //  P3.4 -> TXD
        //  P3.5 <- RXD
        //  P3.6 <- TXD backcannel unused, handled by TUSB3410 direct
        //  P3.7 <- RXD backcannel unused, handled by TUSB3410 direct
        P3SEL |=  BIT4 + BIT5;
        P3DIR |=  BIT4;
        // Port4
        //  P4.0 ->
        //  P4.1 ->
        //  P4.2 ->
        //  P4.3 ->
        //  P4.4 ->
        //  P4.5 ->
        //  P4.6 <- RST3410
        //  P4.7 <-
        P4OUT |= BIT0 + BIT1 + BIT2 + BIT3 + BIT4 + BIT5;   //!<sets target TEST pin on low, disable TDI 2 TDO, enable TDI
        P4DIR |= BIT0 + BIT1 + BIT2 + BIT3 + BIT4 + BIT5;
        // Port5
        //  P5.0 ->
        //  P5.1 -> TDI
        //  P5.2 <- TDO
        //  P5.3 -> TCK
        //  P5.4 <-
        //  P5.5 -> clk3410
        //  P5.6 <-
        //  P5.7 <-
        P5DIR |= BIT0 + BIT1 + BIT3;
        bios_leds_[0].addr = &bios_led_dummy_;
        bios_leds_[0].bit = BIT0;
        bios_leds_[1].addr = &bios_led_dummy_;
        bios_leds_[1].bit = BIT1;
        for(j = 0; (j < 0xFFFF); j++)
        {
            __no_operation();
        }
        P4DIR &= ~BIT6; // release TUSB3410 reset
        // char set to rx input for xon/xoff
        bios_rx_char_ptr_ = &bios_rx_char_;
        // Rts map to software (xon/xoff
        bios_xoff_ptr_ = &bios_xoff_;
        bios_info_hw_0_ = INFO_E2_HW_0;
        bios_info_hw_1_ = INFO_E2_HW_1;
        bios_device_flags_ = DEVICE_FLAG_SBW2 | DEVICE_FLAG_EASY;;
        //  init timer b for 100hz interrupt
        TBCCTL0 = 0;
        TBCCR0 = crystal_ / (8 * 100) - 1;
        TBCTL = TBSSEL_2 | ID_3 | MC_1 | TBCLR | TBIE;    // alck, div 8, up-mode, clear, irq on
    }
    else
    { // U1
        P3OUT &= ~BIT6;
        BCSCTL1 |= XT2OFF;      // XT2 off
        BCSCTL1 |= XTS;           // ACLK = LFXT1 = HF XTAL
        _BIC_SR(OSCOFF);          // turn on XT1 oscillator
        // oscillator start, the fault flag must be at ten times valid, because the
        // oscillator starts very slow and is for short times valid in this time, 
        // but faults again.
        j = 0;
        do
        {
            IFG1 &= ~OFIFG;
            for (i = 0xFF; i>0; i--) 
            {
                __no_operation();
            }
            if(IFG1 & OFIFG)
            {
                j = 0;
            }
            else
            {
                j++;
            }
        }
        while (j < 10);
        IE1 |= OFIE; // restart the oscillator on fault
        BCSCTL2 |= SELM1+SELM0;     // HF XTAL (safe)
        // here setup U1 specific stuff  
        crystal_ = CRYSTAL_U1;
        baudmod_ = 0x57; //BAUDMOD_U1;
        // Port1
        //  P1.0 -> MODE
        //  P1.1 -> POWER
        //  P1.2 <- nc
        //  P1.3 <- nc
        //  P1.4 <- ???
        //  P1.5 -> UDSR
        //  P1.6 -> UCTS
        //  P1.7 <- MU2
        P1OUT = ~(unsigned char)(BIT0+BIT1);          //!<switch both LEDs off (BIT0 = MODE, BIT1 = POWER)
        P1DIR =  (BIT0+BIT1);          //!<set pins to output direction
        // init handshake lines from MSP430 -> TUSB3410
        P1OUT &= ~BIT5;
        P1OUT |= BIT6;  // set CTS to block receiving anything from TUSB (RX overflow on start up problem)
        P1DIR |=  (BIT5+BIT6);
        // Port2
        //  P2.0 ->
        //  P2.1 ->
        //  P2.2 ->
        //  P2.3 ->
        //  P2.4 ->
        //  P2.5 -> SELT
        //  P2.6 -> TGTRST
        //  P2.7 ->
        P2OUT |=  (BIT5+BIT6);          //!<disable JTAG drivers for TCK, TMS, TDI
        P2DIR |=  (BIT5+BIT6);          //!<target reset high
        // Port3
        //  P3.0 <-
        //  P3.1 <-
        //  P3.2 <-
        //  P3.3 <-
        //  P3.4 -> TXD
        //  P3.5 <- RXD
        //  P3.6 <- Rst3410
        //  P3.7 <-
        P3SEL |=  (BIT4 + BIT5);
        P3DIR |=  (BIT4);
        // Port4
        //  P4.0 -> TEST
        //  P4.1 <-
        //  P4.2 -> ENTDI2TDO
        //  P4.3 -> VCCTON
        //  P4.4 -> TDIOFF
        //  P4.5 -> VF2TDI
        //  P4.6 -> VF2TEST 
        //  P4.7 <-
        P4OUT |= (BIT0+BIT2+BIT4);   //!<sets target TEST pin on low, disable TDI 2 TDO, enable TDI
        P4DIR |= (BIT0+BIT2+BIT3+BIT4+BIT5+BIT6);
        // Port5
        //  P5.0 -> TMS
        //  P5.1 -> TDI
        //  P5.2 <- TDO
        //  P5.3 -> TCK
        //  P5.4 <-
        //  P5.5 <-
        //  P5.6 <-
        //  P5.7 <-
        P5DIR |= (BIT0+BIT1+BIT3);
        bios_leds_[0].addr = (unsigned char*)&P1OUT;
        bios_leds_[0].bit = BIT0;
        bios_leds_[1].addr = (unsigned char*)&P1OUT;
        bios_leds_[1].bit = BIT1;
        // char set to rx input for xon/xoff
        bios_rx_char_ptr_ = &bios_rx_char_;
        // Rts map to software (xon/xoff
        bios_xoff_ptr_ = &bios_xoff_;
        // sw & hw infos
        bios_info_hw_0_ = INFO_U1_HW_0;
        bios_info_hw_1_ = INFO_U1_HW_1;
        bios_device_flags_ = DEVICE_FLAG_SBW2 | DEVICE_FLAG_SBW4 | DEVICE_FLAG_XONOFF;
        //  init timer b for 100hz interrupt
        TBCCTL0 = 0;
        TBCCR0 = crystal_ / (8 * 100) - 1;
        TBCTL = TBSSEL_1 | ID_3 | MC_1 | TBCLR | TBIE;    // alck, div 8, up-mode, clear, irq on
        P3OUT |= BIT6;    // release TUSB3410
    }
    __enable_interrupt();
}

//! \brief restore Xt1 clocks
//! \details if a ozcillator fault occur, the fault flag is cleared and the 
//! SMCLOCK switch back to XT1 source
INTERRUPT(NMI_VECTOR)void nmiIsr (void)
{
    while(IFG1 & OFIFG)
    {
        IFG1 &= ~OFIFG;
    }
    IE1 |= OFIE;    // must be re-enable after every nmi irq
}

//! \brief Stop the watch dog
//! \param[in] time not used
void biosSetWdt(unsigned char time)
{
    WDTCTL = WDTPW + WDTHOLD;             //!<Stop watchdog timer
}

//! \brief fatal error
//! \details LED are alternated, function never return
void biosGlobalError(void)
{
    biosLedOn(BIOS_LED_MODE);
    biosLedOff(BIOS_LED_POWER);
    while(1);
}

//! \brief switch LED on
//! \details only the LED control struc are manipulated. The
//! hardware are in timerB1Isr driven
//! \param[in] LED no 0 = BIOS_LED_MODE (red)\n
//!                   1 = BIOS_LED_POWER (green)
//! \return always 0
short biosLedOn(unsigned char no)
{
    if(no < BIOS_LED_COUNT)
    {
        bios_leds_[no].mode = BIOS_LED_ON;
        bios_leds_[no].load = 0;
        bios_leds_[no].counter = 0;
    }
    return(0);
}

//! \brief switch LED off
//! \details only the LED control struc are manipulated. The
//! hardware are in timerB1Isr driven
//! \param[in] no 0 = BIOS_LED_MODE (red)\n
//!               1 = BIOS_LED_POWER (green)
//! \return always 0
short biosLedOff(unsigned char no)
{
    if(no < BIOS_LED_COUNT)
    {
        bios_leds_[no].mode = BIOS_LED_OFF;
        bios_leds_[no].load = 0;
        bios_leds_[no].counter = 0;
    }
    return(0);
}

//! \brief LED blink continuous
//! \param[in] no 0 = BIOS_LED_MODE (red)\n
//!               1 = BIOS_LED_POWER (green)
//! \param[in] time (*20ms)
//! \return always 0
short biosLedBlink(unsigned char no,unsigned short time)
{
    if(no < BIOS_LED_COUNT)
    {
        bios_leds_[no].load = time * 2;
        bios_leds_[no].counter = time * 2;
        bios_leds_[no].mode = BIOS_LED_BLINK;
    }
    return(0);
}

//! \brief LED flashes for a time
//! \param[in] no 0 = BIOS_LED_MODE (red)\n
//!               1 = BIOS_LED_POWER (green)
//! \param[in] time (*20ms)
//! \return always 0
//! \details if the LED is on, the LED goes off for "time"
//! if the LED is off, the LED goes on for "time"
short biosLedFlash(unsigned char no,unsigned short time)
{
    if((no < BIOS_LED_COUNT) && !bios_leds_[no].counter)
    {
        bios_leds_[no].load = time * 2;
        bios_leds_[no].counter = time * 2;
        bios_leds_[no].mode &= ~BIOS_LED_BLINK;
    }
    return(0);
}

//! \brief LEDs blink alternated
//! \param[in] time (*20ms)
//! \return always 0
//! \details both LED are alternated on.
short biosLedAlternate(unsigned short time)
{
    DISABLE_INTERRUPT;
    bios_leds_[0].load = time * 2;
    bios_leds_[0].counter = time;
    bios_leds_[1].load = time * 2;
    bios_leds_[1].counter = time;
    if(time)
    {
        bios_leds_[0].mode = BIOS_LED_ON | BIOS_LED_BLINK;
        bios_leds_[1].mode = BIOS_LED_OFF | BIOS_LED_BLINK;
    }
    else
    {
        bios_leds_[0].mode = BIOS_LED_OFF;
        bios_leds_[1].mode = BIOS_LED_OFF;
    }    
    ENABLE_INTERRUPT;
    return(0);
    }

//! \brief 10ms timer interrupt
//! \details count all 10ms the timer for RX and TX timeouts.
//! Also the timer for LED drive.
INTERRUPT(TIMERB1_VECTOR) void timerB1Isr (void)
{
    if(TBIV == 0x0E)
    {
        ENABLE_INTERRUPT; // enable interrups, the RX task need a short response time and can't wait of other interrupts
        // process global timers
        if((bios_global_timer_[BIOS_TIMER_RX].count > 1) && !(bios_global_timer_[BIOS_TIMER_RX].state & BIOS_TIMER_BREAK))
        {
            bios_global_timer_[BIOS_TIMER_RX].count--;
        }
        else if(bios_global_timer_[BIOS_TIMER_RX].count == 1)
        {
            bios_global_timer_[0].count = 0;
            biosUsbRxError(EXCEPTION_RX_TIMEOUT_ERR);
        }
        if((bios_global_timer_[BIOS_TIMER_TX].count > 1) && !(bios_xoff_))
        {
            bios_global_timer_[BIOS_TIMER_TX].count--;
        }
        // LED mode    
        DIAG_SUPPRESS(Pa082)
        if((bios_leds_[BIOS_LED_MODE].mode & BIOS_LED_ON) ^ (bios_leds_[BIOS_LED_MODE].counter > (bios_leds_[BIOS_LED_MODE].load/2)))
        {
            *bios_leds_[BIOS_LED_MODE].addr |= bios_leds_[BIOS_LED_MODE].bit;
        }
        else
        {
            *bios_leds_[BIOS_LED_MODE].addr &= ~bios_leds_[BIOS_LED_MODE].bit;
        }
        DIAG_DEFAULT(Pa082)
        if(bios_leds_[BIOS_LED_MODE].counter > 0)
        {
            bios_leds_[BIOS_LED_MODE].counter--;
        }
        else if(bios_leds_[BIOS_LED_MODE].mode & BIOS_LED_BLINK)
        {
            bios_leds_[BIOS_LED_MODE].counter = bios_leds_[BIOS_LED_MODE].load;
        }

        // LED power
        DIAG_SUPPRESS(Pa082)
        if((bios_leds_[BIOS_LED_POWER].mode & BIOS_LED_ON) ^ (bios_leds_[BIOS_LED_POWER].counter > (bios_leds_[BIOS_LED_POWER].load/2)))
        {
            *bios_leds_[BIOS_LED_POWER].addr |= bios_leds_[BIOS_LED_POWER].bit;
        }
        else
        {
            *bios_leds_[BIOS_LED_POWER].addr &= ~bios_leds_[BIOS_LED_POWER].bit;
        }
        DIAG_DEFAULT(Pa082)
        if(bios_leds_[BIOS_LED_POWER].counter > 0)
        {
            bios_leds_[BIOS_LED_POWER].counter--;
        }
        else if(bios_leds_[BIOS_LED_POWER].mode & BIOS_LED_BLINK)
        {
            bios_leds_[BIOS_LED_POWER].counter = bios_leds_[BIOS_LED_POWER].load;
        }
    }
}

//! \brief dummy to catch all irq
//! \details biosHalInterfaceClear set all by bios unused irqs to dummy
INTERRUPT_PROTO void dummyIsr (void)
{
    ;
}
//! \brief Pointer to HAL signature
const unsigned short *hal_signature = (unsigned short*)0xDFFE;

//! \brief Pointer to Bios IRQ vector table
//! \details using of linker file give linker errors
const volatile unsigned short *bios_intvec_ = (unsigned short*)0xFDDA;

//! \brief Pointer to HAL IRQ vector table
RO_PLACEMENT_NO_INIT volatile const unsigned short hal_intvec_[16] @ "HALINTVEC";

//! \brief Pointer to irq table in RAM
//! the RAM table is filled form safecore, all irqs are forwarded to the RAM table
NO_INIT volatile unsigned short irq_forward_[16] @ "IRQ_FORWARD";

//! \brief install to HAL macros
//! \details copy the HAL irq to RAM irq table. Start the HAL startup code by using the reset vector.
//! \return -1 -> no HAL infos found, HAL not installed
short biosHalInterfaceInit(void)
{
    HalMainFunc halStartUpCode = NULL;
    unsigned char cmd[6] = {0x05, CMDTYP_EXECUTELOOP, 0, 0, 0, 0};
    unsigned char i;
  
    biosHalInterfaceClear();
    if((hal_intvec_[15] >= 0x2520) && (hal_intvec_[15] <= 0xFDD9)) 
    {
        if(*hal_signature == 0x5137)
        {
            // copy HAL IRQ-pointer to RAM, only if not used by BIOS
            for(i=0; i < (sizeof(irq_forward_)/sizeof(unsigned short)); i++)
            {
                if((bios_intvec_[i] == 0xFFFF) && (hal_intvec_[i] != 0xFFFF))
                {
                    irq_forward_[i] = hal_intvec_[i];
                }
            }
          
            halStartUpCode = (HalMainFunc)hal_intvec_[15]; // calls the (modified) startup code of HAL
            hal_infos_ = halStartUpCode((struct stream_funcs*)&_stream_Funcs, bios_device_flags_); // return HAL sw infos
        }
        if(hal_infos_ != NULL)
        {
            hal_ptr_ = (HAL_REC_ARRAY)(*hal_infos_).hal_list_ptr;
            if(hal_ptr_ != NULL)
            {
                for(i=0; i <(*hal_infos_).hal_size; i++)
                {
                    if(((*hal_ptr_)[i].id != 0xFFFF) && ((*hal_ptr_)[i].id >= 0xFF00))
                    {
                        cmd[4] = i;
                        cmd[5] = 0;
                        v3opSetLoop(cmd, 0);
                    }
                }
            }
        }
    }
    else
    {
        return -1;
    }
    return 0;
}

//! \brief reset the HAL interface
//! \details set irqs to dummy, but didn't disable interrupts.
//! Set all HAL call addresses to NULL
short biosHalInterfaceClear(void)
{
    unsigned char i;

    hal_infos_ = &no_hal_infos_;
    hal_ptr_ = NULL;

    for(i=0; i < (sizeof(irq_forward_)/sizeof(unsigned short)); i++)
    {
        if(bios_intvec_[i] == 0xFFFF)
        {
          irq_forward_[i] = (unsigned short)&dummyIsr;
        }
    }
    return(0);
}

//! \brief main loop
//! \details returns never. 
//! \li send error messages
//! \li execute "commands in loop", eg. voltage regulation, wait(poll) for hit breakpoints
//! \li execute commands send from dll
void  biosmainLoop(void)
{
    unsigned char loop_array_counter;
    unsigned char rx_queu_counter = 0;
    unsigned char rx_queu_counter_tmp;
    StreamSafe stream_tmp;
    HalFuncInOut pCallAddr;
    
    biosResetCts(); // release CTS line
    while(1)
    {
        // send error messages
        if(bios_rx_err_set_)
        {
            DIAG_SUPPRESS(Pa082)
            v3opSendException(bios_rx_err_id_, bios_rx_err_code_, (unsigned short*)bios_rx_err_payload_);
            DIAG_DEFAULT(Pa082)
            bios_rx_err_set_ = 0;
            biosResetCts();
        }
        // search for and execute "commands in loop"
        for(loop_array_counter = 0; loop_array_counter < V3OP_LOOP_ARRAY_COUNT; loop_array_counter++)
        {
            if(v3op_loop_array_[loop_array_counter].addr <  (*hal_infos_).hal_size)
            {
                pCallAddr = (HalFuncInOut)(*hal_ptr_)[v3op_loop_array_[loop_array_counter].addr].function;
                if(pCallAddr != NULL && v3op_loop_array_[loop_array_counter].active != 0)
                {
                    if(STREAM_out_init(v3op_loop_array_[loop_array_counter].msg_id, v3op_loop_array_[loop_array_counter].msg_type) >= 0)
                    {
                        STREAM_internal_stream(&v3op_loop_array_[loop_array_counter].indata[MESSAGE_EXECUTE_PAYLOAD_POS], v3op_loop_array_[loop_array_counter].indata[0]-3, (unsigned char*)0x0001, 0, &stream_tmp);
                        if(pCallAddr(MESSAGE_NEW_MSG | MESSAGE_LAST_MSG) == 1)
                        {
                            STREAM_flush();
                            if(v3op_loop_array_[loop_array_counter].flags & V3OP_LOOP_WAIT_FLAG)
                            {
                                v3opKillLoop(v3op_loop_array_[loop_array_counter].msg_id);
                            }
                        }
                        STREAM_external_stream(&stream_tmp);
                    }
                }
            }
        }
        // test on new messages from dll and execute
        rx_queu_counter_tmp = rx_queu_counter;
        do
        {
            if(bios_rx_record_.state[rx_queu_counter] & BIOS_RX_RDY)
            {
                biosLedFlash(BIOS_LED_MODE,20);
                STREAM_in_init((BiosRxRecord*)&bios_rx_record_, rx_queu_counter);
                rx_queu_counter_public_ = rx_queu_counter;
                v3opRx(bios_rx_record_.data[rx_queu_counter]);
                rx_queu_counter = rx_queu_counter_public_;
                bios_rx_record_.state[rx_queu_counter] &= ~BIOS_RX_RDY;
                break;
            }
            rx_queu_counter++;
            if(rx_queu_counter >= BIOS_RX_QUEUS)
            {
              rx_queu_counter = 0;
            }
        }
        while(rx_queu_counter_tmp != rx_queu_counter);
    }
}

//! \brief test on overflow and framing error (from TUSB)
INLINE(forced)
char biosUsbOverflow(void)
{
    return((U0RCTL & (FE | OE)) > 0);
}

//! \brief setup the serial port to/from TUSB
//! \details init hardware and (software)buffers
//! \param[in] baudrate speed in bit/s
void biosInitCom (unsigned long baudrate)
{ 
    unsigned char i;
    DIAG_SUPPRESS(Pe550)
    unsigned char dummy;
    DIAG_DEFAULT(Pe550)

    for(i = 0; i < BIOS_RX_QUEUS; i++)
    {
        bios_rx_record_.datas[i] = (unsigned short*)malloc(BIOS_RX_SIZE);
        if(bios_rx_record_.datas[i] == NULL)
        {
            biosGlobalError();
        }
        bios_rx_record_.data[i] = (unsigned char*)bios_rx_record_.datas[i];
    }
    for(i = 0; i < BIOS_TX_QUEUS; i++)
    {
        bios_tx_record_.data[i] = (unsigned char*)bios_tx_record_.datas[i];
    }
    UCTL0  = 0;
    UCTL0  = SWRST;                       // hold USART logic in reset state
    ME1   |= UTXE0 + URXE0;               // Enable USART0 TXD/RXD
    UCTL0 |= CHAR;                        // 8-bit character
    if(BCSCTL1 & XT2OFF)
    {
        UTCTL0 = SSEL0;                       // UCLK = ACLK
    }
    else
    {
        UTCTL0 = SSEL1;                       // UCLK = SMCLK
    }
    UBR00  = crystal_/baudrate;            // load baudrate
    UBR10  = (crystal_/baudrate)>>8;       //
    UMCTL0 = baudmod_;                     // Modulation
    UCTL0 &= ~SWRST;                      // Initialize USART state machine
    bios_xoff_ = 0;
    IE1   |= URXIE0 | UTXIE0;             // Enable USART0 TX interrupt
    dummy = USB_READ;
    biosResetCts();
}

//! \brief finsh record to send
void biosPrepareTx(unsigned int size)
{
    bios_tx_record_.send_counter[bios_tx_record_.cannel_to_send] = 0;
    DIAG_SUPPRESS(Pa082)
    bios_tx_record_.count[bios_tx_record_.active] = size;
    bios_tx_record_.state[bios_tx_record_.active] |= BIOS_TX_TO_SEND;
    DIAG_DEFAULT(Pa082)
}

//! \brief trigger sending of TX buffer
//! \details to send the first char we set UTXIFG, followed char loaded by 
//! true TX buffer empty interrupt
void biosStartTx(void)
{
    bios_global_timer_[BIOS_TIMER_TX].count = BIOS_TX_TIMEOUT;
    IFG1 |= UTXIFG0;
}

//! \brief clears the TX software buffer
void biosUsbTxError(void)
{
    biosUsbTxClear();
}

//! \brief clears the TX software buffer
void biosUsbTxClear(void)
{
    unsigned char i;
    
    memset((unsigned char*)&bios_tx_record_, 0, sizeof(bios_tx_record_));
    bios_global_timer_[BIOS_TIMER_TX].count = 0;
    bios_rx_char_ = 0;
    for(i = 0; i < BIOS_TX_QUEUS; i++)
    {
        bios_tx_record_.data[i] = (unsigned char*)bios_tx_record_.datas[i];
    }
}

//! \brief target for dummyread form UART
volatile unsigned char usb_read_dummy_;

//! \brief clear RX hard and softwarebuffer
void biosUsbRxClear(void)
{
    unsigned char i;
    unsigned short j;
    unsigned char mark = 0;
    
    if(IE1 & URXIE0)
    {
        IE1 &= ~URXIE0;   
        mark = 1;
    }
    
    bios_rx_record_.active = 0;
    for(i = 0; i < BIOS_RX_QUEUS; i++)
    {
        bios_rx_record_.state[i] = 0;
        bios_rx_record_.last_cmd_typ = 0;
        bios_rx_record_.last_msg_id = 0;
        bios_rx_record_.data[i][0] = 0;
        bios_rx_record_.crc[i] = 0;
        bios_rx_record_.count[i] = 0;
        bios_rx_record_.size[i] = 0;
    }
    bios_global_timer_[0].count = 0;
    j = 300;
    do
    {
        usb_read_dummy_ = USB_READ;
        usb_read_dummy_ = USB_READ;
        usb_read_dummy_ = USB_READ;
        IFG1 &= ~URXIFG0;
        U0RCTL &= ~(FE |PE | OE | BRK | RXERR);
        for(i=0; i < 255; i++)
        {
            NO_OPERATION;
        }
        j--;
    }
    while(j && (IFG1 & URXIFG0));
    if(mark)
    {
        IE1   |= URXIE0;   
    }
}
//! \brief trigger a error message
//! \details trigger a error message and clear the RX hard and software buffers
void biosUsbRxError(unsigned short code)
{
    IE1   &= ~URXIE0;   
    biosUsbRxClear();
    bios_rx_err_code_ = code;
    bios_rx_err_set_ = 1;
    bios_rx_err_payload_ = NULL;
    bios_rx_err_id_ = 0;
    IE1   |= URXIE0;   
}

//! \brief receive char form UART
//! \details get chars form hardware RX buffer and stor them in the RX software buffer(s).
//! Messages are checked on crc. and ack/exceptions processed in this function.
INTERRUPT(USART0RX_VECTOR)void usbRxIsr (void)
{
    static unsigned char last_rx_char;
    static unsigned short crc_tmp = 0;

    // block TUSB send
    biosSetCts();
    // test on buffer overflow and framing error
    if(biosUsbOverflow())
    {
        biosUsbRxError(EXCEPTION_RX_OVERFLOW_ERR);
        goto usbRxIsrExit;
    }
    // get char from hardwarebuffer
    bios_rx_char_ = USB_READ;
    // test for xon/xoff flow control char
    if((*bios_rx_char_ptr_ & 0xFC) == 0x10)
    {
        if(bios_rx_char_ == BIOS_XON_CHAR)
        {
            // test for continue transmiting
            if(bios_xoff_ & 0x02)  
            {
                biosStartTx();
            }
            bios_xoff_ = 0;
            goto usbRxIsrExit;
        }
        else if (bios_rx_char_ == BIOS_XOFF_CHAR) 
        {
            // block sending to TUSB
            bios_xoff_ = 1 << BIOS_HARD_RTS_BIT;
            goto usbRxIsrExit;
        }
        else if (bios_rx_char_ == BIOS_DLE_CHAR)
        {
            last_rx_char = bios_rx_char_;
            goto usbRxIsrExit;
        }
    }
    else if(last_rx_char == BIOS_DLE_CHAR)
    {
        bios_rx_char_ |= last_rx_char;
        last_rx_char = 0;
    }
    // test for free buffer
    if(bios_rx_record_.state[bios_rx_record_.active] & BIOS_RX_RDY)
    {
        biosUsbRxError(EXCEPTION_RX_NO_BUFFER);
        goto usbRxIsrExit;
    }
    // test for first char in a message
    if(!bios_rx_record_.count[bios_rx_record_.active])
    {
        // a message must received in 100ms completely
        // if not a timeout error generate by timerB1Isr
        bios_global_timer_[0].count = 10; // start timeout timer 100ms
        bios_rx_record_.crc[1] = 0;
        // length of messages must be even
        if(bios_rx_char_ & 0x01)
        {
            bios_rx_record_.size[bios_rx_record_.active] = bios_rx_char_ + 2;
        }
        else
        {
            bios_rx_record_.size[bios_rx_record_.active] = bios_rx_char_ + 3;
        }
    }
    // test for buffer size
    if(bios_rx_record_.count[bios_rx_record_.active] < (BIOS_RX_SIZE + 2))
    {
        DIAG_SUPPRESS(Pa082)
        // crc calculation
        if(bios_rx_record_.count[bios_rx_record_.active] & 0x0001)
        {
            crc_tmp |= (unsigned short)bios_rx_char_ << 8;
            bios_rx_record_.crc[1] ^= crc_tmp;
        }
        else
        {
            crc_tmp = bios_rx_char_;
        }
        // store char in RX software buffer
        bios_rx_record_.data[bios_rx_record_.active][bios_rx_record_.count[bios_rx_record_.active]++] = bios_rx_char_;
        DIAG_DEFAULT(Pa082)
    }
    else
    {
        biosUsbRxError(EXCEPTION_RX_TO_SMALL_BUFFER);
        goto usbRxIsrExit;
    }
    // test for completly message
    DIAG_SUPPRESS(Pa082)
    if(bios_rx_record_.count[bios_rx_record_.active] > (bios_rx_record_.size[bios_rx_record_.active]))
    DIAG_DEFAULT(Pa082)
    {
        bios_global_timer_[0].count = 0;  // stop timeout timer
        // crc must be always 0xFFFF, because the crc field in message is xor, too.
        if(bios_rx_record_.crc[1] == 0xFFFF)
        {
            if(bios_rx_record_.data[bios_rx_record_.active][MESSAGE_CMDTYP_POS] == RESPTYP_ACKNOWLEDGE)
            {
                // search for message which are waiting on a ack
                for(unsigned char i = 0; i < BIOS_TX_QUEUS; i++) 
                { 
                    DIAG_SUPPRESS(Pa082)
                    // waits a transmited message for a ACK?
                    if((bios_tx_record_.state[i] & BIOS_TX_WAIT_ON_ACK) &&
                        ((bios_tx_record_.data[i][MESSAGE_MSG_ID_POS] & 0x3F) == bios_rx_record_.data[bios_rx_record_.active][MESSAGE_MSG_ID_POS]))
                    {
                        // if the transmited message are a member of a multi package message, the ack provide the package number
                        if( bios_tx_record_.data[i][3] || (bios_rx_record_.data[bios_rx_record_.active][0] > 3) )
                        {
                            // package number are in byte 4
                            if(bios_tx_record_.data[i][3] == bios_rx_record_.data[bios_rx_record_.active][4])
                    DIAG_DEFAULT(Pa082)
                            {
                                bios_tx_record_.state[i] &= ~(BIOS_TX_WAIT_ON_ACK | BIOS_TX_TO_SEND | BIOS_TX_NO_SEND);
                                bios_global_timer_[BIOS_TIMER_TX].count = 0;
                                break;
                            }
                        }
                        else
                        {
                            bios_tx_record_.state[i] &= ~(BIOS_TX_WAIT_ON_ACK | BIOS_TX_TO_SEND | BIOS_TX_NO_SEND);
                            break;
                        }
                    }
                }
            }
            else if(bios_rx_record_.data[bios_rx_record_.active][MESSAGE_CMDTYP_POS] ==  RESPTYP_EXCEPTION)
            {
                // exception with message ID = 0 always reset the communication
                // also all user macros in the loop are terminated
                if(bios_rx_record_.data[bios_rx_record_.active][MESSAGE_MSG_ID_POS] == 0)
                {
                    biosLedOn(BIOS_LED_POWER);
                    biosLedOff(BIOS_LED_MODE);
                    bios_wb_control_ = 0;
                    v3opKillLoop(0);
                    biosUsbRxClear();
                    biosUsbTxClear();
                }
                else
                {
                    // search for message which are waiting on a ack
                    for(unsigned char i = 0; i < BIOS_TX_QUEUS; i++) 
                    {
                        DIAG_SUPPRESS(Pa082)
                        if(((bios_tx_record_.state[i] & (BIOS_TX_WAIT_ON_ACK | BIOS_TX_TO_SEND)) == (BIOS_TX_WAIT_ON_ACK | BIOS_TX_TO_SEND)) &&
                            (bios_tx_record_.data[i][MESSAGE_MSG_ID_POS] == bios_rx_record_.data[bios_rx_record_.active][MESSAGE_MSG_ID_POS]) &&
                            (bios_tx_record_.data[i][3] == bios_rx_record_.data[bios_rx_record_.active][4]))
                        DIAG_DEFAULT(Pa082)
                        {
                            // only reset the wait_on_ack flag, 
                            //! \todo add a resend function
                            bios_tx_record_.state[i] &= ~BIOS_TX_WAIT_ON_ACK;
                            break;
                        }
                    }
                }
            }
            else if((bios_rx_record_.data[bios_rx_record_.active][MESSAGE_MSG_ID_POS] == 0) && (bios_rx_record_.data[bios_rx_record_.active][MESSAGE_CMDTYP_POS] ==CMDTYP_COMRESET))
            {
                // clear RX cannel
                biosUsbRxClear();
                goto usbRxIsrExit;
            }
            else
            {
                // mark message as ready
                // biosmainLoop pass the message to v3opRx for macro execution
                DIAG_SUPPRESS(Pa082)
                bios_rx_record_.state[bios_rx_record_.active] |= BIOS_RX_RDY;
                DIAG_DEFAULT(Pa082)
                // switch to the next Rx buffer
                bios_rx_record_.active++;
                if(bios_rx_record_.active >= BIOS_RX_QUEUS)
                {
                    bios_rx_record_.active = 0;
                }
            }
        }         
        else
        {
            // set a crc error
            bios_rx_record_.crc[0] = EXCEPTION_CRC_ERR;
            DIAG_SUPPRESS(Pa082)
            bios_rx_record_.crc[1] ^= *((unsigned short*)&bios_rx_record_.data[bios_rx_record_.active][bios_rx_record_.size[bios_rx_record_.active]-1]);
            DIAG_DEFAULT(Pa082)
            bios_rx_record_.crc[1] ^= 0xFFFF;
            bios_rx_err_code_ = sizeof(bios_rx_record_.crc)/sizeof(unsigned short);
            bios_rx_err_set_ = 1;
            bios_rx_err_id_ = bios_rx_record_.data[bios_rx_record_.active][MESSAGE_MSG_ID_POS];
            bios_rx_err_payload_ = (unsigned short*)&bios_rx_record_.crc;
        }
        // no size error
        bios_rx_record_.count[bios_rx_record_.active] = 0;
        bios_rx_record_.crc[1] = 0;
    }
    usbRxIsrExit:  
    biosResetCts(); // release TUSB RX
}

//! \brief function is called by hardware interrupt if TX buffer goes empty
//! \details move a char from the TX software buffer to the hardware TX buffer.
INTERRUPT(USART0TX_VECTOR)void usbTxIsr (void)
{
    unsigned char first_cannel;
    // test on flow control
    if(*bios_xoff_ptr_)
    {
        bios_xoff_ = 0x02 | (1 << BIOS_HARD_RTS_BIT); // bit 1 indicate that a flow control release must trigger this interrupt
        return;
    }
    // some thing to send in active send buffer?
    first_cannel = bios_tx_record_.cannel_to_send;
    while(!(bios_tx_record_.state[bios_tx_record_.cannel_to_send] & BIOS_TX_TO_SEND))
    {
        bios_tx_record_.cannel_to_send++;
        if(bios_tx_record_.cannel_to_send >= BIOS_TX_QUEUS) 
        {
            bios_tx_record_.cannel_to_send = 0;
        }
        if(bios_tx_record_.cannel_to_send == first_cannel)
        {
            return;
        }
    }
    // test on chars in software TX buffer
    DIAG_SUPPRESS(Pa082)
    if(bios_tx_record_.send_counter[bios_tx_record_.cannel_to_send] < bios_tx_record_.count[bios_tx_record_.cannel_to_send]) 
    {
        // send char from software buffer
        USB_WRITE = bios_tx_record_.data[bios_tx_record_.cannel_to_send][bios_tx_record_.send_counter[bios_tx_record_.cannel_to_send]++];
    }
    else
    {
        // no char to send in actual TX software buffer
        // we don't clear TX software buffer, reset only flags. If need we can 
        // send the buffer again. (resending in not implemented)
        bios_tx_record_.state[bios_tx_record_.cannel_to_send] &= ~BIOS_TX_TO_SEND;
        bios_tx_record_.send_counter[bios_tx_record_.cannel_to_send] = 0;    
        first_cannel = bios_tx_record_.cannel_to_send;
        if(!(bios_tx_record_.state[bios_tx_record_.cannel_to_send] & BIOS_TX_WAIT_ON_ACK))
        {
            bios_global_timer_[BIOS_TIMER_TX].count = 0;
        }
        // search in TX software buffer for data to send. If no data found, it was the 
        // last TX buffer empty interrupt.
        do
        {
            bios_tx_record_.cannel_to_send++;
            if(bios_tx_record_.cannel_to_send >= BIOS_TX_QUEUS) 
            {
                bios_tx_record_.cannel_to_send = 0;
            }
            if(bios_tx_record_.state[bios_tx_record_.cannel_to_send] & BIOS_TX_TO_SEND)
            {
                if(bios_tx_record_.send_counter[bios_tx_record_.cannel_to_send] <= ((unsigned short)bios_tx_record_.data[bios_tx_record_.cannel_to_send][0])) // add left side +1, if crc active
                {
                    USB_WRITE = bios_tx_record_.data[bios_tx_record_.cannel_to_send][bios_tx_record_.send_counter[bios_tx_record_.cannel_to_send]++];
                    bios_global_timer_[BIOS_TIMER_TX].count = BIOS_TX_TIMEOUT;
                }
            }
        }
        while(!(bios_tx_record_.state[bios_tx_record_.cannel_to_send] & BIOS_TX_TO_SEND) && (bios_tx_record_.cannel_to_send != first_cannel));
    }
    DIAG_DEFAULT(Pa082)
}
