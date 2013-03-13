/**
* \ingroup MODULHIL
*
* \file hil_4w.c
*
* \brief 4 wire implementation of hil interface
*
*/
/*
 *
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

#include "hw_compiler_specific.h"
#include "HalGlobalVars.h"
#include "arch.h"
#include "edt.h"

#define  PIN_TMS            BIT0
#define  PIN_TDI            BIT1
#define  PIN_TDO            BIT2
#define  PIN_TCK            BIT3


//CONST_AT( struct jtag _Jtag, HAL_ADDR_CONST_JTAG) = {
const struct jtag _Jtag = {
  (unsigned char)0x08,  // TCK, P5.3 (out) (high)
  (unsigned char)0x01,  // TMS, P5.0 (out) (high)
  (unsigned char)0x02,  // TDI, P5.1 (out) (high)
  (unsigned char)0x04,  // TDO, P5.2 (in)
  (unsigned char*)&P5IN,
  (unsigned char*)&P5OUT
};

const unsigned char JTAG_ENTRY_IR0[] = {
(PIN_TMS),(PIN_TMS|PIN_TCK),(PIN_TMS),(PIN_TMS|PIN_TCK),0,(PIN_TCK),0,(PIN_TCK)
};
const unsigned char JTAG_ENTRY_IR1[] = {
(PIN_TDI|PIN_TMS),(PIN_TDI|PIN_TMS|PIN_TCK),(PIN_TDI|PIN_TMS),(PIN_TDI|PIN_TMS|PIN_TCK),(PIN_TDI),(PIN_TDI|PIN_TCK),(PIN_TDI),(PIN_TDI|PIN_TCK)
};
const unsigned char JTAG_ENTRY_DR0[] = {
(PIN_TMS),(PIN_TMS|PIN_TCK),0,(PIN_TCK),0,(PIN_TCK)
};
const unsigned char JTAG_ENTRY_DR1[] = {
(PIN_TDI|PIN_TMS),(PIN_TDI|PIN_TMS|PIN_TCK),(PIN_TDI),(PIN_TDI|PIN_TCK),(PIN_TDI),(PIN_TDI|PIN_TCK)
};
const unsigned char JTAG_IDLE0[] = {
(PIN_TMS),(PIN_TMS|PIN_TCK),0,(PIN_TCK)
};
const unsigned char JTAG_IDLE1[] = {
(PIN_TDI|PIN_TMS),(PIN_TDI|PIN_TMS|PIN_TCK),(PIN_TDI),(PIN_TDI|PIN_TCK)
};

// -----------------------------------------------------------------------------
short _hil_4w_CheckJtagFuse(void)
{
    // perform a JTAG fuse check
    TMSset1;
    TMSset0;
    EDT_Delay_1us(5);
    TMSset1;
    TMSset0;
    EDT_Delay_1us(5);
    TMSset1;
    return 0;
}

// -----------------------------------------------------------------------------
short _hil_4w_TapReset(void)
{
    // Reset TAP Controller State Machine
    // Set default state for JTAG signals (TDI = TMS = TCK = 1)
    TDIset1;
    TMSset1;
    TCKset1;
    // Clock TCK six (6) times
    TCKset0;
    TCKset1;
    TCKset0;
    TCKset1;
    TCKset0;
    TCKset1;
    TCKset0;
    TCKset1;
    TCKset0;
    TCKset1;
    TCKset0;
    TCKset1;
    // TAP Controller State Machine is now in "Test-Logic Reset" state
    // Clock TCK one more time with TMS = 0
    TMSset0;
    TCKset0;
    TCKset1;
    return 0;
}

#define Invalid_Manufactor_IdCode 0x000000FEul
#define Mask_Manufactor_IdCode 0x000000FEul

#define StoreTTDI()     (*_Jtag.Out)
#define RestoreTTDI(x)  (x &  _Jtag.TDI ? (*_Jtag.Out |= _Jtag.TDI) : (*_Jtag.Out &=  ~_Jtag.TDI))
#define ScanTDO()       ((*_Jtag.In   &  _Jtag.TDO) != 0)

// defined in hil.c
extern void testVpp(unsigned char mode);
extern void SetVpp(long voltage);

unsigned short _hil_4w_EnumChain(void)
{
    unsigned long TDOval = 0x00000000;    // Initialize shifted-in word
    unsigned long MSB = 0x80000000;
    unsigned long LSB = 0x00000001;
    unsigned long DataIn = Invalid_Manufactor_IdCode | LSB;
    unsigned char i = 0;
    unsigned char detIdCode = 0;
    unsigned short numOfDevices = 0;
    
    _hil_4w_TapReset();
    
    // JTAG FSM state = Run-Test/Idle
    TMSset1;
    EDT_Delay_1us(10);
    TCKset0;
    EDT_Delay_1us(10);
    TCKset1;
    EDT_Delay_1us(10);
    // JTAG FSM state = Select DR-Scan
    TMSset0;
    EDT_Delay_1us(10);
    TCKset0;
    EDT_Delay_1us(10);
    TCKset1;
    EDT_Delay_1us(10);
    // JTAG FSM state = Capture-DR
    TCKset0;
    EDT_Delay_1us(10);
    TCKset1;
    EDT_Delay_1us(10);
    // JTAG FSM state = Shift-IR
  
    while(1)
    {
        if((DataIn & LSB) == 0)
        {
            TDIset0;
        }
        else
        {
            TDIset1;
        }
        DataIn >>= 1;
        TCKset0;
        EDT_Delay_1us(10);
        TCKset1;
        TDOval >>= 1;			    // TDO could be any port pin
        if (ScanTDO())
        {
            TDOval |= MSB;
            if(0 == detIdCode)                // Test if LSB of IdCode
            {
                i = 0;
                detIdCode = 1;
            }
        }
        else if(0 == detIdCode)
        {
            numOfDevices++;
        }
    
        i += detIdCode;
        if(32 == i)
        {
            detIdCode = 0;
            i = 0;
            if(Invalid_Manufactor_IdCode == (TDOval & Mask_Manufactor_IdCode))
            {
                // End of chain detected
                break; 
            }
            // Device with valid IdCode detected
            numOfDevices++;
        }
        if(0xFF == numOfDevices)
        {
            // Only 254 devices are supported
            // Probably hardware connection is broken
            numOfDevices = 0;
            break;
        }
    }  
    
    TMSset1;
    EDT_Delay_1us(10);
    TCKset0;
    EDT_Delay_1us(10);
    TCKset1;  
    // JTAG FSM = Exit-DR
    TCKset0;
    EDT_Delay_1us(10);
    TCKset1;
    EDT_Delay_1us(10);
    // JTAG FSM = Update-DR
    TMSset0;
    EDT_Delay_1us(10);
    TCKset0;
    EDT_Delay_1us(10);
    TCKset1;
    EDT_Delay_1us(10);
    // JTAG FSM = Run-Test/Idle
    TMSset1; // to save power during debugging
    EDT_Delay_1us(10);
    return numOfDevices;
}



// -----------------------------------------------------------------------------
unsigned char _hil_4w_Instr(unsigned char Instruction)
{
    unsigned char  TDOvalue;
    
    // DMA application
    DMA1SZ = 8;                         // load DMA1 with size
    if(StoreTTDI() & _Jtag.TDI)
    {
        DMA1SA = (unsigned int)JTAG_ENTRY_IR1;
        DMA2SA = (unsigned int)JTAG_IDLE1;
    }
    else
    {
        DMA1SA = (unsigned int)JTAG_ENTRY_IR0;
        DMA2SA = (unsigned int)JTAG_IDLE0;
    }
    DMA1CTL |= DMAEN;     // enable DMA1
    DMA2CTL |= DMAEN;     // enable DMA2
    DMA1CTL |= DMAREQ;    // DMA1 FIRE!!!
    // JTAG FSM state = Shift-IR
    P5SEL |= 0x0E; // enablespi 
    UCTL1 &= ~CHAR;
    U1TXBUF = Instruction; //Dataloadspi 8
    while(!(UTCTL1 & TXEPT));
    P5SEL &= ~0x0E; // disable spi
    if(Instruction & 0x1)
    {
        TDIset1TMSset1
    }
    else
    {
        TDIset0TMSset1
    }
    TCKset0
    TCKset1
    TDOvalue = U1RXBUF << 1;
    TDOvalue |= ScanTDO();
    // common exit
    DMA2CTL |= DMAREQ;    // DMA2 FIRE!!!
    
    // JTAG FSM = Run-Test/Idle
    return TDOvalue;
}

unsigned char _hil_4w_SetReg_XBits08(unsigned char Data)
{
    unsigned char  TDOvalue;
    
    DMA1SZ = 6;
    if(StoreTTDI() & _Jtag.TDI)
    {
        DMA1SA = (unsigned int)JTAG_ENTRY_DR1;
        DMA2SA = (unsigned int)JTAG_IDLE1;
    }
    else
    {
        DMA1SA = (unsigned int)JTAG_ENTRY_DR0;
        DMA2SA = (unsigned int)JTAG_IDLE0;
    }
    DMA1CTL |= DMAEN;
    DMA2CTL |= DMAEN;
    DMA1CTL |= DMAREQ;
    // JTAG FSM state = Shift-DR
    P5SEL |= 0x0E; // enablespi 
    UCTL1 &= ~CHAR;
    U1TXBUF = Data; //Dataloadspi 8
    while(!(UTCTL1 & TXEPT));
    TDOvalue = U1RXBUF << 1;
    P5SEL &= ~0x0E; // disable spi
    if(Data & 0x1)
    {
        TDIset1TMSset1
    }
    else
    {
        TDIset0TMSset1
    }
    TCKset0
    TCKset1
    TDOvalue |= ScanTDO();
    // common exit
    DMA2CTL |= DMAREQ;
    return TDOvalue;
}
unsigned short _hil_4w_SetReg_XBits16(unsigned short Data)
{
    unsigned short  TDOvalue = 0;
    unsigned char  *outptr = (unsigned char*)&Data;
    unsigned char  *inptr = (unsigned char*)&TDOvalue;
    
    DMA1SZ = 6;
    if(StoreTTDI() & _Jtag.TDI)
    {
        DMA1SA = (unsigned int)JTAG_ENTRY_DR1;
        DMA2SA = (unsigned int)JTAG_IDLE1;
    }
    else
    {
        DMA1SA = (unsigned int)JTAG_ENTRY_DR0;
        DMA2SA = (unsigned int)JTAG_IDLE0;
    }
    DMA1CTL |= DMAEN;
    DMA2CTL |= DMAEN;
    DMA1CTL |= DMAREQ;
    // JTAG FSM state = Shift-DR
    P5SEL |= 0x0E; // enablespi 
    UCTL1 |= CHAR;
    U1TXBUF = *(outptr + 1);
    while(!(UTCTL1 & TXEPT));
    *(inptr + 1) = U1RXBUF;
    UCTL1 &= ~CHAR;
    U1TXBUF = *outptr; //Dataloadspi 8
    while(!(UTCTL1 & TXEPT));
    *inptr = U1RXBUF << 1;
    P5SEL &= ~0x0E; // disable spi
    if(Data & 0x1)
    {
        TDIset1TMSset1
    }
    else
    {
        TDIset0TMSset1
    }
    TCKset0
    TCKset1
    *inptr |= ScanTDO();
    // common exit
    DMA2CTL |= DMAREQ;
    return TDOvalue;
}
unsigned long _hil_4w_SetReg_XBits20(unsigned long Data)
{
    unsigned long  TDOvalue = 0;
    unsigned char  *outptr = (unsigned char*)&Data;
    unsigned char  *inptr = (unsigned char*)&TDOvalue;
    
    DMA1SZ = 6;
    if(StoreTTDI() & _Jtag.TDI)
    {
        DMA1SA = (unsigned int)JTAG_ENTRY_DR1;
        DMA2SA = (unsigned int)JTAG_IDLE1;
    }
    else
    {
        DMA1SA = (unsigned int)JTAG_ENTRY_DR0;
        DMA2SA = (unsigned int)JTAG_IDLE0;
    }
    DMA1CTL |= DMAEN;
    DMA2CTL |= DMAEN;
    DMA1CTL |= DMAREQ;
    // JTAG FSM state = Shift-DR
    if(*(outptr+2) & 0x08)
    {
        TDIset1
    }
    else 
    {
        TDIset0
    }
    TCKset0 TCKset1
    *(inptr+2) |= ScanTDO();
    
    if(*(outptr+2) & 0x04)
    {
        TDIset1
    }
    else
    {
        TDIset0
    }
    TCKset0 TCKset1
    *(inptr+2) <<= 1;
    *(inptr+2) |= ScanTDO();
    
    if(*(outptr+2) & 0x02)
    {
        TDIset1
    }
    else
    {
        TDIset0
    }
    TCKset0 TCKset1
    *(inptr+2) <<= 1;
    *(inptr+2) |= ScanTDO();
    
    if(*(outptr+2) & 0x01)
    {
        TDIset1
    }
    else
    {
        TDIset0
    }
    TCKset0 TCKset1
    *(inptr+2) <<= 1;
    *(inptr+2) |= ScanTDO();
    
    P5SEL |= 0x0E; // enablespi 
    UCTL1 |= CHAR;
    U1TXBUF = *(outptr + 1);
    while(!(UTCTL1 & TXEPT));
    *(inptr + 1) = U1RXBUF;
    UCTL1 &= ~CHAR;
    U1TXBUF = *outptr; //Dataloadspi 8
    while(!(UTCTL1 & TXEPT));
    *inptr = U1RXBUF << 1;
    P5SEL &= ~0x0E; // disable spi
    if(Data & 0x1)
    {
        TDIset1TMSset1
    }
    else
    {
        TDIset0TMSset1
    }
    TCKset0
    TCKset1
    *inptr |= ScanTDO();
    // de-scramble upper 4 bits if it was a 20bit shift
    TDOvalue = ((TDOvalue >> 4) | (TDOvalue << 16)) & 0x000FFFFF;
    // common exit
    DMA2CTL |= DMAREQ;
    return TDOvalue;
}

// -----------------------------------------------------------------------------
unsigned long _hil_4w_SetReg_XBits32(unsigned long Data)
{
    unsigned long  TDOvalue = 0;
    unsigned char  *outptr = (unsigned char*)&Data;
    unsigned char  *inptr = (unsigned char*)&TDOvalue;
    
    DMA1SZ = 6;
    if(StoreTTDI() & _Jtag.TDI)
    {
        DMA1SA = (unsigned int)JTAG_ENTRY_DR1;
        DMA2SA = (unsigned int)JTAG_IDLE1;
    }
    else
    {
        DMA1SA = (unsigned int)JTAG_ENTRY_DR0;
        DMA2SA = (unsigned int)JTAG_IDLE0;
    }
    DMA1CTL |= DMAEN;
    DMA2CTL |= DMAEN;
    DMA1CTL |= DMAREQ;
    
    // JTAG FSM state = Shift-DR
    P5SEL |= 0x0E; // enablespi 
    UCTL1 |= CHAR;
    U1TXBUF = *(outptr + 3);
    while(!(UTCTL1 & TXEPT));
    *(inptr + 3) = U1RXBUF;
    U1TXBUF = *(outptr + 2);
    while(!(UTCTL1 & TXEPT));
    *(inptr + 2) = U1RXBUF;
    U1TXBUF = *(outptr + 1);
    while(!(UTCTL1 & TXEPT));
    *(inptr + 1) = U1RXBUF;
    UCTL1 &= ~CHAR;
    U1TXBUF = *outptr; 
    while(!(UTCTL1 & TXEPT));
    *inptr = U1RXBUF << 1;
    P5SEL &= ~0x0E; // disable spi
    if(Data & 0x1)
    TDIset1TMSset1
    else
    TDIset0TMSset1
    TCKset0
    TCKset1
    *inptr |= ScanTDO();
    // common exit
    DMA2CTL |= DMAREQ;
    return TDOvalue;
}
// -----------------------------------------------------------------------------
unsigned long long _hil_4w_SetReg_XBits64(unsigned long long Data)
{
    unsigned long long TDOvalue = 0;
    unsigned char  *outptr = (unsigned char*)&Data;
    unsigned char  *inptr = (unsigned char*)&TDOvalue;
    
    DMA1SZ = 6;
    if(StoreTTDI() & _Jtag.TDI)
    {
        DMA1SA = (unsigned int)JTAG_ENTRY_DR1;
        DMA2SA = (unsigned int)JTAG_IDLE1;
    }
    else
    {
        DMA1SA = (unsigned int)JTAG_ENTRY_DR0;
        DMA2SA = (unsigned int)JTAG_IDLE0;
    }
    DMA1CTL |= DMAEN;
    DMA2CTL |= DMAEN;
    DMA1CTL |= DMAREQ;
    
    // JTAG FSM state = Shift-DR
    P5SEL |= 0x0E; // enablespi 
    UCTL1 |= CHAR;
    
    U1TXBUF = *(outptr + 7);
    while(!(UTCTL1 & TXEPT));
    *(inptr + 7) = U1RXBUF;
    
    U1TXBUF = *(outptr + 6);
    while(!(UTCTL1 & TXEPT));
    *(inptr + 6) = U1RXBUF;
    
    U1TXBUF = *(outptr + 5);
    while(!(UTCTL1 & TXEPT));
    *(inptr + 5) = U1RXBUF;
    
    U1TXBUF = *(outptr + 4);
    while(!(UTCTL1 & TXEPT));
    *(inptr + 4) = U1RXBUF;
    
    U1TXBUF = *(outptr + 3);
    while(!(UTCTL1 & TXEPT));
    *(inptr + 3) = U1RXBUF;
    
    U1TXBUF = *(outptr + 2);
    while(!(UTCTL1 & TXEPT));
    *(inptr + 2) = U1RXBUF;
    
    U1TXBUF = *(outptr + 1);
    while(!(UTCTL1 & TXEPT));
    *(inptr + 1) = U1RXBUF;
    
    UCTL1 &= ~CHAR;
    U1TXBUF = *outptr; 
    while(!(UTCTL1 & TXEPT));
    *inptr = U1RXBUF << 1;
    
    P5SEL &= ~0x0E; // disable spi
    if(Data & 0x1)
    TDIset1TMSset1
    else
    TDIset0TMSset1
    TCKset0
    TCKset1
    *inptr |= ScanTDO();
    // common exit
    DMA2CTL |= DMAREQ;
    return TDOvalue;
}
// -----------------------------------------------------------------------------
void _hil_4w_Tclk(unsigned char state)
{
    if(state)
    {
        TDIset1
    }
    else
    {
        TDIset0
    }
}

// -----------------------------------------------------------------------------
void _hil_4w_StepPsa(unsigned long length)
{
    while(length-- > 0)
    {
        TCLKset1 _NOP();
        TCLKset0 _NOP();

        TCKset0 _NOP();
        TMSset1 _NOP();
        TCKset1 _NOP(); // select DR scan
        TCKset0 _NOP();
        TMSset0 _NOP();
        
        TCKset1 _NOP(); // capture DR
        TCKset0 _NOP();
        TCKset1 _NOP(); // shift DR
        TCKset0 _NOP();
        
        TMSset1 _NOP();
        TCKset1 _NOP();// exit DR
        TCKset0 _NOP();
        
        // Set JTAG FSM back into Run-Test/Idle
        TCKset1 _NOP();
        TMSset0 _NOP();
        TCKset0 _NOP();
        TCKset1 _NOP();
        _NOP(); _NOP(); _NOP(); _NOP(); _NOP();
    }
}
// -----------------------------------------------------------------------------
void _hil_4w_StepPsaTclkHigh(unsigned long length)
{
    while(length--)
    {
        TCLKset1 _NOP();

        TCKset0 _NOP();
        TMSset1 _NOP();
        TCKset1 _NOP();// select DR scan
        TCKset0 _NOP();
        TMSset0 _NOP();
        
        TCKset1 _NOP();// capture DR
        TCKset0 _NOP();
        TCKset1 _NOP();// shift DR
        TCKset0 _NOP();
        
        TMSset1 _NOP();
        TCKset1 _NOP(); // exit DR
        TCKset0 _NOP();
        
        // Set JTAG FSM back into Run-Test/Idle
        TCKset1 _NOP();
        TMSset0 _NOP();
        TCKset0 _NOP();
        TCKset1 _NOP();

        _NOP(); _NOP(); _NOP(); _NOP(); _NOP();
        TCLKset0 _NOP();
    }
}

// -----------------------------------------------------------------------------
short _hil_4w_BlowFuse(unsigned char targetHasTestVpp)
{
    if(!targetHasTestVpp) 
    {
        cntrl_sig_16bit;
        SetReg_16Bits_(0x7401); // TDOs get TDI functionality (internal)
        testVpp(0);             // Switchs also TDO functionality to TDI.
    }
    EDT_Instr(IR_PREPARE_BLOW);                     // initialize fuse blowing
    EDT_Delay_1ms(1);
    
    SetVpp(1);
    EDT_Instr(IR_EX_BLOW);                          // execute fuse blowing
    EDT_Delay_1ms(1);
    SetVpp(0);                                       // switch VPP off
    
    testVpp(1);                                      // Restore the normal function of TDO and TDI (within the interface).

    // now perform a BOR via JTAG - we loose control of the device then...
    EDT_Instr(IR_TEST_REG);
    EDT_SetReg_XBits32(0x00000200);    
    
    return 0;
}
/* EOF */
