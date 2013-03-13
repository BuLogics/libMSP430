/*
 * hil_jsbw.c
 * 
 * <FILEBRIEF>
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

//! \ingroup MODULHIL
//! \file hil_jsbw.c
//! \brief 
//!

#include "hw_compiler_specific.h"
#include "HalGlobalVars.h"
#include "arch.h"
#include "edt.h"

/*****************************************/
// JSBW definitions
#define	F_BYTE	     8
#define F_WORD		 16
#define F_ADDR       20
#define F_LONG		 32

#define TDICTRL2     ENTDI2TDO // TO BE ADDAPTED!!!!!!!!s

/*#define JSBW_TMSH    TSTCTRLOUT |= TEST;  EDT_Delay_1us(400);TGTCTRLOUT &= ~TGTRST; EDT_Delay_1us(1); TGTCTRLOUT |= TGTRST; EDT_Delay_1us(1); // TMS = 1
#define JSBW_TMSL    TSTCTRLOUT &= ~TEST; EDT_Delay_1us(400);TGTCTRLOUT &= ~TGTRST; EDT_Delay_1us(1); TGTCTRLOUT |= TGTRST; EDT_Delay_1us(1);// TMS = 0
#define JSBW_TDIH    TSTCTRLOUT |= TEST;  EDT_Delay_1us(400);TGTCTRLOUT &= ~TGTRST; EDT_Delay_1us(1); TGTCTRLOUT |= TGTRST;  EDT_Delay_1us(1);           // TDI = 1
#define JSBW_TDIL    TSTCTRLOUT &= ~TEST; EDT_Delay_1us(400);TGTCTRLOUT &= ~TGTRST; EDT_Delay_1us(1);  TGTCTRLOUT |= TGTRST; EDT_Delay_1us(1);          // TDI = 0
#define JSBW_TDOsbw  TSTCTRLOUT |= TDICTRL2; TGTCTRLOUT &= ~TGTRST; EDT_Delay_1us(1);  TGTCTRLOUT |= TGTRST; EDT_Delay_1us(1);  TSTCTRLOUT &= ~TDICTRL2;   // TDO cycle without reading TDO
#define JSBW_TDO_RD  TSTCTRLOUT |= TDICTRL2; TGTCTRLOUT &= ~TGTRST; EDT_Delay_1us(1); tdo_bit = JTAGIN; TGTCTRLOUT |= TGTRST; EDT_Delay_1us(1);  TSTCTRLOUT &= ~TDICTRL2;  // TDO cycle with TDO read

#define JSBW_TMSL_TDIL()  { JSBW_TMSL  JSBW_TDIL  JSBW_TDOsbw }
#define JSBW_TMSH_TDIL()  { JSBW_TMSH  JSBW_TDIL  JSBW_TDOsbw }
#define JSBW_TMSL_TDIH()  { JSBW_TMSL  JSBW_TDIH  JSBW_TDOsbw }
#define JSBW_TMSH_TDIH()  { JSBW_TMSH  JSBW_TDIH  JSBW_TDOsbw }
#define JSBW_TMSL_TDIH_TDOrd() { JSBW_TMSL  JSBW_TDIH  JSBW_TDO_RD }
#define JSBW_TMSL_TDIL_TDOrd() { JSBW_TMSL  JSBW_TDIL  JSBW_TDO_RD }
#define JSBW_TMSH_TDIH_TDOrd() { JSBW_TMSH  JSBW_TDIH  JSBW_TDO_RD }
#define JSBW_TMSH_TDIL_TDOrd() { JSBW_TMSH  JSBW_TDIL  JSBW_TDO_RD }*/


/*----------------------------------------------------------------------------
   Macros to control LPMx.5*/


#define TDICTRL2  ENTDI2TDO // TO BE ADDAPTED!!!!!!!!s

#define   JSBW_TMSH    TGTCTRLOUT |= JSBWsbwdato;  EDT_Delay_1us(400);TSTCTRLOUT |= JSBsbwclk; EDT_Delay_1us(1); TSTCTRLOUT &= ~JSBsbwclk; EDT_Delay_1us(1); // TMS = 1
#define   JSBW_TMSL    TGTCTRLOUT &= ~JSBWsbwdato; EDT_Delay_1us(400);TSTCTRLOUT |= JSBsbwclk; EDT_Delay_1us(1); TSTCTRLOUT &= ~JSBsbwclk; EDT_Delay_1us(1);// TMS = 0
//#define   TMSLDH  TGTCTRLOUT &= ~JSBWsbwdato; TGTCTRLOUT |= JSBsbwclk; TGTCTRLOUT &= ~JSBsbwclk; TGTCTRLOUT &= ~JSBsbwclk; // TMS = 0, then TCLK immediately = 1
#define   JSBW_TDIH    TGTCTRLOUT |= JSBWsbwdato;  EDT_Delay_1us(400);TSTCTRLOUT |= JSBsbwclk; EDT_Delay_1us(1); TSTCTRLOUT &= ~JSBsbwclk;  EDT_Delay_1us(1);           // TDI = 1
#define   JSBW_TDIL    TGTCTRLOUT &= ~JSBWsbwdato; EDT_Delay_1us(400);TSTCTRLOUT |= JSBsbwclk; EDT_Delay_1us(1);  TSTCTRLOUT &= ~JSBsbwclk; EDT_Delay_1us(1);          // TDI = 0
#define   JSBW_TDOsbw  TSTCTRLOUT |= TDICTRL2; TSTCTRLOUT |= JSBsbwclk; EDT_Delay_1us(1);  TSTCTRLOUT &= ~JSBsbwclk; EDT_Delay_1us(1);  TSTCTRLOUT &= ~TDICTRL2;   // TDO cycle without reading TDO
#define   JSBW_TDO_RD  TSTCTRLOUT |= TDICTRL2; TSTCTRLOUT |= JSBsbwclk; EDT_Delay_1us(1); tdo_bit = JTAGIN; TSTCTRLOUT &= ~JSBsbwclk; EDT_Delay_1us(1);  TSTCTRLOUT &= ~TDICTRL2;  // TDO cycle with TDO read


#define JSBW_TMSL_TDIL()  { JSBW_TMSL  JSBW_TDIL  JSBW_TDOsbw }
#define JSBW_TMSH_TDIL()  { JSBW_TMSH  JSBW_TDIL  JSBW_TDOsbw }
#define JSBW_TMSL_TDIH()  { JSBW_TMSL  JSBW_TDIH  JSBW_TDOsbw }
#define JSBW_TMSH_TDIH()  { JSBW_TMSH  JSBW_TDIH  JSBW_TDOsbw }
#define JSBW_TMSL_TDIH_TDOrd() { JSBW_TMSL  JSBW_TDIH  JSBW_TDO_RD }
#define JSBW_TMSL_TDIL_TDOrd() { JSBW_TMSL  JSBW_TDIL  JSBW_TDO_RD }
#define JSBW_TMSH_TDIH_TDOrd() { JSBW_TMSH  JSBW_TDIH  JSBW_TDO_RD }
#define JSBW_TMSH_TDIL_TDOrd() { JSBW_TMSH  JSBW_TDIL  JSBW_TDO_RD }


/*****************************************/

VAR_AT(unsigned char TCLK_saved, HAL_ADDR_VAR_TCLK_SAVED);

extern unsigned char *tstctrl_port_;
extern unsigned char entdi2tdo_;
unsigned char tdo_bit = 0;

// -----------------------------------------------------------------------------
// JSBW functions
//! \note Does not implement the full HIL, since only some functions are needed.
//!       Used DLLv2 functions as reference.
// -----------------------------------------------------------------------------
unsigned long jsbw_Shift(unsigned int Format, unsigned long Data)
{
    unsigned long TDOword = 0x00000000;
    unsigned long MSB = 0x00000000;
    unsigned int i;

    //JTAGSEL &= ~(TDI | TDO | TCK);      // function select to ports
    switch(Format)
    {
    case F_BYTE: MSB = 0x00000080;
        break;
    case F_WORD: MSB = 0x00008000;
        break;
    case F_ADDR: MSB = 0x00080000;
        break;
    case F_LONG: MSB = 0x80000000;
        break;
    default: // this is an unsupported format, function will just return 0
        return TDOword;
    }
    for (i = Format; i > 0; i--)
    {
        if (i == 1)                  // last bit requires TMS=1; TDO one bit before TDI
        {
            if((Data & MSB) == 0)
            {
                JSBW_TMSH_TDIL_TDOrd();
            }
            else
            {
                JSBW_TMSH_TDIH_TDOrd();
            }
        }
        else
        {
            if((Data & MSB) == 0)
            {
                JSBW_TMSL_TDIL_TDOrd();
            }
            else
            {
                JSBW_TMSL_TDIH_TDOrd();
            }
        }
        Data <<= 1;
    }

    if (TCLK_saved & JSBWsbwdato)
    {
        JSBW_TMSH_TDIH();
        JSBW_TMSL_TDIH();
    }
    else
    {
        JSBW_TMSH_TDIL();
        JSBW_TMSL_TDIL();
    }

    // de-scramble bits on a 20bit shift
    if(Format == F_ADDR)
    {
        TDOword = ((TDOword << 16) + (TDOword >> 4)) & 0x000FFFFF;
    }
   
    return(TDOword);
}

// -----------------------------------------------------------------------------
unsigned short jsbw_IR_Shift(unsigned char instruction)
{
    // JTAG FSM state = Run-Test/Idle
    //SetTMS();
    //ClrTCK();
    //SetTCK();
    if (TCLK_saved & JSBWsbwdato) //PrepTCLK
    {
        JSBW_TMSH_TDIH();
    }
    else
    {
        JSBW_TMSH_TDIL();
    }

    // JTAG FSM state = Select DR-Scan
    //ClrTCK();
    //SetTCK();
    JSBW_TMSH_TDIH();

    // JTAG FSM state = Select IR-Scan
    //ClrTMS_TCK();
    //SetTCK();
    JSBW_TMSL_TDIH();
    // JTAG FSM state = Capture-IR
    //ClrTCK();
    //SetTCK();
    JSBW_TMSL_TDIH();

  // JTAG FSM state = Shift-IR, Shiftin TDI (8 bit)
  return(jsbw_Shift(F_BYTE, instruction));  // JTAG FSM state = Run-Test/Idle
}

// -----------------------------------------------------------------------------
unsigned long jsbw_DR_Shift(unsigned int Format, unsigned long data)
{
    // JTAG FSM state = Run-Test/Idle
    //SetTMS();
    //ClrTCK();
    //SetTCK();
    if (TCLK_saved & JSBWsbwdato) //PrepTCLK
    {
        JSBW_TMSH_TDIH();
    }
    else
    {
        JSBW_TMSH_TDIL();
    }

    // JTAG FSM state = Select DR-Scan
    //ClrTMS_TCK();
    //SetTCK();
    JSBW_TMSL_TDIH();
    // JTAG FSM state = Capture-DR
    //ClrTCK();
    //SetTCK();
    JSBW_TMSL_TDIH();

    // JTAG FSM state = Shift-DR, Shiftin TDI (16 bit)
    return (jsbw_Shift(Format, data));
    // JTAG FSM state = Run-Test/Idle
}

// -----------------------------------------------------------------------------
void JSBW_TapReset(void)
{
    unsigned int i = 0;
    // Reset JTAG FSM
    for (i = 6; i > 0; i--)      // 6 is nominal
    {
        JSBW_TMSH_TDIH();
    }
    // JTAG FSM is now in Test-Logic-Reset
    JSBW_TMSL_TDIH();                 // now in Run/Test Idle

    // Fuse check
    JSBW_TMSH_TDIH();
    JSBW_TMSL_TDIH();
    JSBW_TMSH_TDIH();
    JSBW_TMSL_TDIH();
    JSBW_TMSH_TDIH();
    // In every TDI slot a TCK for the JTAG machine is generated.
    // Thus we need to get TAP in Run/Test Idle state back again.
    JSBW_TMSH_TDIH();
    JSBW_TMSL_TDIH();      
    EDT_Delay_1ms(10);  
}

// -----------------------------------------------------------------------------
void JSBW_MagicPattern(void)
{
    jsbw_IR_Shift(IR_JMB_EXCHANGE);
    EDT_Delay_1ms(10);       
    jsbw_DR_Shift(16, 0x0001);
    EDT_Delay_1ms(10);    
    jsbw_DR_Shift(16, 0xA55A);    
    EDT_Delay_1ms(15);        
}

// -----------------------------------------------------------------------------
void JSBW_JtagUnlock(void)
{
   jsbw_IR_Shift(IR_TEST_3V_REG);
   EDT_Delay_1ms(10);  
   jsbw_DR_Shift(16, 0x4020);
   EDT_Delay_1ms(10);      
}

void jRelease(void)
{
    // drive target RST/SBWTDIO pin high
    TGTCTRLOUT |=  JSBWsbwdato;        // TDI drives target RST high
    TSTCTRLOUT &= ~ENTDI2TDO;      // enable TDI2TDO
    EDT_Delay_1ms(1);
    // drive target TEST/SBWTCK pin low
    TSTCTRLOUT |= JSBsbwclk;         // TCK drives target TEST low - release Spy-Bi-Wire logic
    TGTCTRLOUT &= ~SELT;        // enable JTAG & RST pin drivers
}

/* EOF */
