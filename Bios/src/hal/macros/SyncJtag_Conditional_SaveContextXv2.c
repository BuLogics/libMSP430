/**
* \ingroup MODULMACROSXV2
*
* \file SyncJtag_Conditional_SaveContextXv2.c
*
* \brief <FILEBRIEF>
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

#include "arch.h"
#include "edt.h"
#include "hal.h"
#include "hal_ref.h"
#include "stream.h"
#include "JTAG_defs.h"
#include "EEM_defs.h"


#define SR             2
#define	EEM_STOPPED    0x0080 

extern unsigned short altRomAddressForCpuRead;

/**
  SyncJtag_Conditional_SaveContextXv2
  inData:  <wdtAddr(16)> <wdtCtrl(16)>
  outData: <wdtCtrl(16)> <PC(32)> <SR(16)>
  wdtAddr: watchdog timer address
  wdtCtrl: watchdog timer control value
  PC:      program counter register (R0)
  SR:      status register (R2)
*/

extern DevicePowerSettings devicePowerSettings;

//! \todo EEM cycle counter, decide if it should be handled by host DLL!
HAL_FUNCTION(_hal_SyncJtag_Conditional_SaveContextXv2)
{
  decl_out
  decl_out_long
  
  unsigned char pipe_empty = 0;
  unsigned char cpuoff = 0;
  unsigned char intreq = 0;
  unsigned char halt = 0;
  unsigned long halt_count = 0;
  const unsigned long MaxCyclesForSync = 10000; // must be defined, dependant on DMA (burst transfer)!!!
  unsigned long i = 0;
  unsigned short MyOut[4];
  unsigned short wdt_addr;
  unsigned short wdt_value;
  unsigned long TimeOut = 0;

//------------------------------------------------------------------------------
// Start note 822: Special handling ynchronize to JTAG
  
    // -------------------------Power mode handling start ----------------------
    DisableLpmx5();
    // -------------------------Power mode handling end ------------------------
  
  // read out EEM control register...
  eem_read_control
  SetReg_16Bits(0x0000)
  // ... and check if device got already stopped by the EEM
  if (!(lOut & EEM_STOPPED))
  { // do this only if the device is NOT already stopped.
    // read out control signal register first
    cntrl_sig_capture
    SetReg_16Bits(0x0000)
    // check if CPUOFF bit is set
    if(!(lOut & CNTRL_SIG_CPUOFF))
    { // do the following only if the device is NOT in Low Power Mode
      unsigned long tbValue;
      unsigned long tbCntrl;
      unsigned long tbMask;
      unsigned long tbComb;
      unsigned long tbBreak;

      // Trigger Block 0 value register      
      eem_data_exchange32
      SetReg_32Bits(MBTRIGxVAL + READ + TB0);   // load address
      SetReg_32Bits(0);                         // shift in dummy 0
      tbValue = lOut_long;
      // Trigger Block 0 control register      
      SetReg_32Bits(MBTRIGxCTL + READ + TB0);   // load address
      SetReg_32Bits(0);                         // shift in dummy 0
      tbCntrl = lOut_long;
      // Trigger Block 0 mask register      
      SetReg_32Bits(MBTRIGxMSK + READ + TB0);   // load address
      SetReg_32Bits(0);                         // shift in dummy 0
      tbMask = lOut_long;
      // Trigger Block 0 combination register      
      SetReg_32Bits(MBTRIGxCMB + READ + TB0);   // load address
      SetReg_32Bits(0);                         // shift in dummy 0
      tbComb = lOut_long;
      // Trigger Block 0 combination register      
      SetReg_32Bits(BREAKREACT + READ);         // load address
      SetReg_32Bits(0);                         // shift in dummy 0
      tbBreak = lOut_long;
      
      // now configure a trigger on the next instruction fetch
      SetReg_32Bits(MBTRIGxCTL + WRITE + TB0);   // load address
      SetReg_32Bits(CMP_EQUAL + TRIG_0 + MAB);
      SetReg_32Bits(MBTRIGxMSK + WRITE + TB0);   // load address
      SetReg_32Bits(MASK_ALL);
      SetReg_32Bits(MBTRIGxCMB + WRITE + TB0);   // load address
      SetReg_32Bits(EN0);
      SetReg_32Bits(BREAKREACT + WRITE);         // load address
      SetReg_32Bits(EN0);
      
      // enable EEM to stop the device
      SetReg_32Bits(GENCLKCTRL + WRITE);         // load address
      SetReg_32Bits(MCLK_SEL0 + SMCLK_SEL0 + ACLK_SEL0 + STOP_MCLK + STOP_SMCLK + STOP_ACLK);
      eem_write_control
      SetReg_16Bits_(EMU_CLK_EN + CLEAR_STOP + EEM_EN);
      
      {
        short lTimeout = 0;
        do
        {
          eem_read_control
          SetReg_16Bits(0x0000)
          lTimeout++;
        }
        while(!(lOut & EEM_STOPPED) && lTimeout < 3000);
      }
      // restore the setting of Trigger Block 0 previously stored
      // Trigger Block 0 value register      
      eem_data_exchange32
      SetReg_32Bits(MBTRIGxVAL + WRITE + TB0);   // load address
      SetReg_32Bits(tbValue);
      SetReg_32Bits(MBTRIGxCTL + WRITE + TB0);   // load address
      SetReg_32Bits(tbCntrl);
      SetReg_32Bits(MBTRIGxMSK + WRITE + TB0);   // load address
      SetReg_32Bits(tbMask);
      SetReg_32Bits(MBTRIGxCMB + WRITE + TB0);   // load address
      SetReg_32Bits(tbComb);
      SetReg_32Bits(BREAKREACT + WRITE);         // load address
      SetReg_32Bits(tbBreak);
    }
  }
// End: special handling note 822
//------------------------------------------------------------------------------
  
  // enable clock control before sync
  eem_write_control
  SetReg_16Bits_(EMU_CLK_EN + EEM_EN);

  // sync device to JTAG   
  SyncJtagXv2();
  
  // reset CPU stop reaction - CPU is now under JTAG control
  // Note: does not work on F5438 due to note 772, State storage triggers twice on single stepping
  eem_write_control
  SetReg_16Bits(EMU_CLK_EN + CLEAR_STOP + EEM_EN)
  SetReg_16Bits(EMU_CLK_EN + CLEAR_STOP)

  cntrl_sig_16bit
  SetReg_16Bits(0x1501)
  // clock system into Full Emulation State now...
  // while checking control signals CPUSUSP (pipe_empty), CPUOFF, INTREQ and HALT
    
  cntrl_sig_capture 
  SetReg_16Bits(0)   
  ///////////////////////////////////////////
  // Note 805: check if wait is set
  if((lOut & 0x8) == 0x8)
  {
    // wait until wait is end
    while((lOut & 0x8) == 0x8 && TimeOut++ < 30000)
    {
        TCLKset0 // provide falling clock edge
        TCLKset1 // provide rising clock edge
        cntrl_sig_capture 
        SetReg_16Bits(0)  
    }
  }
  //Note 805 end: Florian, 21 Dec 2010
  ///////////////////////////////////////////  
  
  cntrl_sig_capture
  do
  {
    TCLKset0 // provide falling clock edge
    // check control signals during clock low phase
    SetReg_16Bits(0) // shift out current control signal register value
    (lOut & CNTRL_SIG_CPUSUSP) ? (pipe_empty = TRUE) : (pipe_empty = FALSE);
    (lOut & CNTRL_SIG_INTREQ)  ? (intreq     = TRUE) : (intreq     = FALSE);
    (lOut & CNTRL_SIG_HALT)    ? (halt       = TRUE) : (halt       = FALSE);
    
    TCLKset1 // provide rising clock edge
    
    if(halt)
    {
      halt_count = 0; // reset counter when DMA transfer is active
    }
    else
    {
      halt_count++;   // increment counter when no DMA request is available
    }
    
    i++; // increment loop counter for braek condition
  }
  // break condition:
  //    pipe_empty = 1
  //    intreq     = 0
  //    halt_count > 3
  //    or an error occured (MaxCyclesForSync exeeded!!)
  //df  while(!(pipe_empty && !intreq && halt_count > 3) && i < MaxCyclesForSync);
    while(!pipe_empty && (i < MaxCyclesForSync));

  //! \todo check error condition
  if(i >= MaxCyclesForSync)
  {
    ;
  }
  
  // the interrupts will be diabled now - JTAG takes over control of the control signals
  cntrl_sig_16bit
  SetReg_16Bits(0x0501)

  // recall an interrupt request
  /**
   * End of  : i_Conditional
   */

  /**
   * Begin of: i_SaveContext
   */
  // provide 1 clock in order to have the data ready in the first transaction slot
  TCLK
  addr_capture
  SetReg_20Bits(0)

  cntrl_sig_capture
  SetReg_16Bits(0) // shift out current control signal register value
  if(lOut & CNTRL_SIG_CPUOFF)
  {
    cpuoff = TRUE;
  }
  else
  {
    cpuoff = FALSE;
  }

  // adjust program counter according to control signals
  if(cpuoff)
  {
    lOut_long -= 2;
  }
  else
  {
    lOut_long -= 4;
  }  
  {
      unsigned long  tempPc = lOut_long;   
      /********************************************************/
      /* Note 1495, 1637 special handling for program counter */
      /********************************************************/
      if(tempPc == 0xFFFE)
      {
        unsigned long  tempPc = 0;    
        unsigned short read = 0;		
        ReadMemWordXv2(0xFFFE,read); 
        tempPc = (0x00000000 | read);
        
        MyOut[1] = (unsigned short)(tempPc & 0xFFFF);
        MyOut[2] = (unsigned short)(tempPc >>16);
      }
      /* End note 1495, 1637 */
      else
      {    
        MyOut[1] = (unsigned short)(tempPc & 0xFFFF);
        MyOut[2] = (unsigned short)(tempPc >>16);
      }  
  }
  /**
  * End of  : i_SaveContext
  */
  
  // set EEM FEATURE enable now!!!
  eem_write_control
  SetReg_16Bits(EMU_FEAT_EN + EMU_CLK_EN + CLEAR_STOP)
  
  // check for Init State
  cntrl_sig_capture
  SetReg_16Bits(0)

  // hold Watchdog Timer
  STREAM_get_word(&wdt_addr);
  STREAM_get_word(&wdt_value);

  ReadMemWordXv2(wdt_addr, MyOut[0]);
  wdt_value |= (MyOut[0] & 0xFF); // set original bits in addition to stop bit
  WriteMemWordXv2(wdt_addr, wdt_value | 0x80);

  {
    unsigned short Mova;
    unsigned short Rx_l;
    unsigned long  Rx;
    // save Status Register content
    Mova  = 0x0060;
    Mova += (SR<<8) & 0x0F00;
    ReadCpuRegXv2(Mova, Rx);
    MyOut[3] = (unsigned short)Rx;
    // reset Status Register to 0 - needs to be restored again for release
    Mova  = 0x0080;
    Mova += (SR & 0x000F);
    Rx_l = (((unsigned short)Rx) & 0xFFEF); // reset CPUOFF bit
    WriteCpuRegXv2(Mova, Rx_l);
  }
  
  STREAM_put_bytes((unsigned char*)MyOut,8);
  return 0;
}

