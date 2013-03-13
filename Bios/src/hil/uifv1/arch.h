/**
* \ingroup MODULHIL
*
* \file arch.h
*
* \brief Important defines for the firmware
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

#ifndef _UIFV1_ARCH_H_
#define _UIFV1_ARCH_H_

struct jtag {
  unsigned char  TCK;
  unsigned char  TMS;
  unsigned char  TDI;
  unsigned char  TDO;
  unsigned char* In;
  unsigned char* Out;
};

extern const struct jtag _Jtag;

#define RSTHIGH 0
#define RSTLOW  1

#define DEFAULT_RSTDELAY 5

// 4-wire JTAG: low level signal access
#define TMSset1    { *_Jtag.Out |=  _Jtag.TMS; }
#define TMSset0    { *_Jtag.Out &= ~_Jtag.TMS; }
#define TDIset1    { *_Jtag.Out |=  _Jtag.TDI; }
#define TDIset0    { *_Jtag.Out &= ~_Jtag.TDI; }
#define TCKset1    { *_Jtag.Out |=  _Jtag.TCK; }
#define TCKset0    { *_Jtag.Out &= ~_Jtag.TCK; }
#define TCLKset1   { EDT_Tclk(1); }
#define TCLKset0   { EDT_Tclk(0); }
#define TCLK       { TCLKset0 TCLKset1 }
#define TDIset1TMSset1    { *_Jtag.Out |=  _Jtag.TDI | _Jtag.TMS; }
#define TDIset0TMSset1    { *_Jtag.Out &= ~_Jtag.TDI; *_Jtag.Out |= _Jtag.TMS;}


// 2-wire Spy-Bi-Wire: low level signal access
#define JTAGOUT (*_Jtag.Out)
#define JTAGIN  (*_Jtag.In)
#define sbwdato (_Jtag.TDI)
#define sbwclk  (_Jtag.TCK)
#define sbwdati (_Jtag.TDO)

#define TEST           BIT0  // P4.0 (out) (high)
#define TSTCTRLOUT     *tstctrl_port_
#define TGTCTRLOUT     P2OUT
#define ENTDI2TDO      entdi2tdo_
#define SELT           BIT5  // P2.5 out (high) (aktiv low)
#define TGTRST         BIT6  // P2.6 out (high) (aktiv low)


#define RSTset1    {   if (gprotocol_id == SPYBIWIRE)                          \
                       {                                                       \
                         { JTAGOUT |= sbwdato; }                               \
                       }                                                       \
                       else                                                    \
                       {                                                       \
                         { TGTCTRLOUT &= ~SELT; TGTCTRLOUT |=  TGTRST; }       \
                       }                                                       \
                       EDT_Delay_1ms(DEFAULT_RSTDELAY);                        \
                   }
#define RSTset0    {   if (gprotocol_id == SPYBIWIRE)                          \
                       {                                                       \
                         { JTAGOUT &= ~sbwdato; }                              \
                       }                                                       \
                       else                                                    \
                       {                                                       \
                         { TGTCTRLOUT &= ~SELT; TGTCTRLOUT &= ~TGTRST; }       \
                       }                                                       \
                       EDT_Delay_1ms(DEFAULT_RSTDELAY);                        \
                   }
#define TSTset1    {   if (gprotocol_id == SPYBIWIRE)                          \
                       {                                                       \
                         { JTAGOUT |= sbwclk;}                                \
                       }                                                       \
                       else                                                    \
                       {                                                       \
                         { ((TSTCTRLOUT) &= (~TEST)); }                        \
                       }                                                       \
                       EDT_Delay_1ms(DEFAULT_RSTDELAY);                        \
                   }
#define TSTset0    {   if (gprotocol_id == SPYBIWIRE)                          \
                       {                                                       \
                         {JTAGOUT &= ~sbwclk;  }                               \
                       }                                                       \
                       else                                                    \
                       {                                                       \
                         { ((TSTCTRLOUT) |= (TEST)); }                      \
                       }                                                       \
                       EDT_Delay_1ms(DEFAULT_RSTDELAY);                        \
                   }

#define TMSL    JTAGOUT &= ~sbwdato; JTAGOUT &= ~sbwclk; JTAGOUT |= sbwclk;
#define TMSLDH  JTAGOUT &= ~sbwdato; JTAGOUT &= ~sbwclk; JTAGOUT |= sbwdato; JTAGOUT |= sbwclk;
#define TDIH    JTAGOUT |= sbwdato;  JTAGOUT &= ~sbwclk; JTAGOUT |= sbwclk;
#define TDIL    JTAGOUT &= ~sbwdato; JTAGOUT &= ~sbwclk; JTAGOUT |= sbwclk;
#define TDOsbw  TSTCTRLOUT ^= ENTDI2TDO; __disable_interrupt(); JTAGOUT &= ~sbwclk; JTAGOUT |= sbwclk; __enable_interrupt(); TSTCTRLOUT ^= ENTDI2TDO;
#define TDO_RD  TSTCTRLOUT ^= ENTDI2TDO; __disable_interrupt(); JTAGOUT &= ~sbwclk; TDOvalue <<= 1; TDOvalue |= (JTAGIN & sbwdati) != 0; JTAGOUT |= sbwclk; __enable_interrupt(); TSTCTRLOUT ^= ENTDI2TDO;

// Constants for VPP connection and process state at Blow-Fuse
#define VPP_ON_TDI				0
#define VPP_ON_TEST				1

#define SETTDOIN    ((P4OUT) &= (~BIT2))
#define SETTDOOUT   ((P4OUT) |= (BIT2))
#define SETTDION    ((P4OUT) |= (BIT4))
#define SETTDIOFF   ((P4OUT) &= (~BIT4))

#define VPPon(x)		(x == VPP_ON_TEST ? (P4OUT |= BIT6) : (P4OUT |= BIT5))
#define VPPoff()		((P4OUT) &= ~(BIT6 | BIT5))


#define CRYSTAL         8000000         // externally connected HF crystal
#define SETVFFREQ       100000          // pulse frequency to setup VF, this one's the fastest ~6ms settletime
#define FLASHFREQ       450000
#define TA_SETVF_DIV    CRYSTAL/SETVFFREQ/2
#define TA_STROBE_DIV   CRYSTAL/FLASHFREQ
#define TA_STROBE_DIV2  CRYSTAL/FLASHFREQ/2

#define FREQUENCY		(CRYSTAL/1000)  // CPU frequency (master/slave) in KHz
#define SEC_TIMEOUT     7
#define DEF_TIMEOUT     (CRYSTAL*SEC_TIMEOUT)/13

#define  VFCHN            2

#define  STROBEOUT      BIT4  // P2.4 (out) (secondary function)
#define  STROBEIN       BIT7  // P4.7 (in)  (secondary function)

#define  VCCTON         BIT3  // P4.3 (out) (low)

#define VCCTon()        ((TSTCTRLOUT) |= (VCCTON))
#define VCCToff()       ((TSTCTRLOUT) &= (~VCCTON))

#define    JSBWsbwdato   TGTRST
//#define    sbwdati   TDO
#define    JSBsbwclk    TEST
//#define    reset     TMS
//#define    rti       TCK   // Run/test-idle

#endif
