/**
* \ingroup MODULMACROSX
*
* \file SyncJtag_Conditional_SaveContextX.c
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
#include "error_def.h"
#include "EEM_defs.h"

#define MAX_TCE1 10
/**
  SyncJtag_Conditional_SaveContext
  inData:  <wdtAddr(16)> <wdtCtrl(16)>
  outData: <wdtCtrl(16)> <PC(32)> <SR(16)>
  wdtAddr: watchdog timer address
  wdtCtrl: watchdog timer control value
  PC:      program counter register (R0)
  SR:      status register (R2)
*/

extern DeviceSettings deviceSettings;

//--------------------------------------------------------------------------
// static STATUS_T clkTclkAndCheckDTC(void)
// \todo Günther: Seems to only apply to devices with the DTC bug, determine the effect
//       this code has on devices that do not have the DTC bug
static long clkTclkAndCheckDTC(void)
{
#define MAX_DTC_CYCLE 10

  decl_out
  unsigned int cntrlSig;
  int dtc_cycle_cnt = 0;
  long timeOut= 0;
  
  do
    {
      TCLKset0

      cntrl_sig_capture
      SetReg_16Bits(0)
          
      cntrlSig = lOut;

      if ((dtc_cycle_cnt > 0) &&
          ((cntrlSig & CNTRL_SIG_CPU_HALT) == 0))
      {
        // DTC cycle completed, take over control again...
        cntrl_sig_16bit
        SetReg_16Bits(CNTRL_SIG_TCE1 | CNTRL_SIG_CPU | CNTRL_SIG_TAGFUNCSAT);
      }
      if ((dtc_cycle_cnt == 0) &&
          ((cntrlSig & CNTRL_SIG_CPU_HALT) == CNTRL_SIG_CPU_HALT))
      {
        // DTC cycle requested, grant it...
        cntrl_sig_16bit
        SetReg_16Bits(CNTRL_SIG_CPU_HALT | CNTRL_SIG_TCE1 | 
                      CNTRL_SIG_CPU | CNTRL_SIG_TAGFUNCSAT);
        dtc_cycle_cnt++;
      }
      TCLKset1
      timeOut++;
    }
    while ((dtc_cycle_cnt < MAX_DTC_CYCLE) &&
           ((cntrlSig & CNTRL_SIG_CPU_HALT) == CNTRL_SIG_CPU_HALT) && timeOut < 5000);

  if (dtc_cycle_cnt < MAX_DTC_CYCLE)
  {
    return (0);
  }
  else
  {
    return (-1);
  }
}


HAL_FUNCTION(_hal_SyncJtag_Conditional_SaveContextX)
{  
  decl_out
  decl_out_long
  decl_isInstrLoad
  decl_instrLoad      
  unsigned short i = 0;
  short MyOut[5] = {0};
  unsigned short address;
  unsigned short wdtVal;
  unsigned short statusReg = 0;
  
  if(STREAM_get_word(&address) != 0) 
  {
      return (HALERR_SYNC_JTAG_CONDITIONAL_NO_WDT_ADDRESS);
  }
  
  if(STREAM_get_word(&wdtVal) == -1)
  {
      return (HALERR_SYNC_JTAG_CONDITIONAL_NO_WDT_VALUE);
  }    

  // DLLv2 syncCPUAndCyclesAfterDebug()
  TCLKset1  // Stability improvement: should be possible to remove this here, default state of TCLK should be one
  
  cntrl_sig_capture
  SetReg_16Bits(0x0000)
      
  if(!(lOut & CNTRL_SIG_TCE)) {
   // If the JTAG and CPU are not already synchronized ...
    // Initiate Jtag and CPU synchronization. Read/Write is under CPU control. Source TCLK via TDI.
    // Do not effect bits used by DTC (CPU_HALT, MCLKON).
    cntrl_sig_high_byte
    SetReg_8Bits((CNTRL_SIG_TAGFUNCSAT | CNTRL_SIG_TCE1 | CNTRL_SIG_CPU) >> 8);

    // TCE eventually set indicates synchronization (and clocking via TCLK).
    SyncJtag()
    if(!i)
    { // Synchronization failed!
        return (HALERR_SYNC_JTAG_CONDITIONAL_JTAG_TIMEOUT);
    }
  }// end of if(!(lOut & CNTRL_SIG_TCE))
  
  if(lOut & CNTRL_SIG_CPU_HALT)
  {
      TCLKset0
      cntrl_sig_16bit
      // Clear HALT. Read/Write is under CPU control. As a precaution, disable interrupts.
      SetReg_16Bits(CNTRL_SIG_TCE1 | CNTRL_SIG_CPU | CNTRL_SIG_TAGFUNCSAT)
      TCLKset1
  }
  else
  {
      cntrl_sig_16bit
      // Clear HALT. Read/Write is under CPU control. As a precaution, disable interrupts.          
      SetReg_16Bits(CNTRL_SIG_TCE1 | CNTRL_SIG_CPU | CNTRL_SIG_TAGFUNCSAT)
  }

  instrLoad // step until next instruction load boundary if not being already there
  if(lInstrLoad != 0)
  {
      return (HALERR_INSTRUCTION_BOUNDARY_ERROR);
  }          

  // read MAB = PC here
  addr_capture
  SetReg_20Bits(0x000)
  MyOut[1] = (unsigned short)(lOut_long & 0xFFFF);
  MyOut[2] = (unsigned short)(lOut_long >> 16);

  // DLLv2: Check if a breakpoint was hit
  // \todo Determine if this is needed
  
 // disable EEM and clear stop reaction
  eem_write_control
  SetReg_16Bits(0x0003)
  SetReg_16Bits(0x0000)
    
  if (deviceSettings.clockControlType == GCC_EXTENDED)
  {
        eem_data_exchange32
        SetReg_32Bits_(MX_GENCNTRL + MX_WRITE)               // write access to EEM General Control Register (MX_GENCNTRL)
        SetReg_32Bits_(EMU_FEAT_EN | EMU_CLK_EN | CLEAR_STOP | EEM_EN)   // write into MX_GENCNTRL
            
        eem_data_exchange32 // Stability improvement: should be possible to remove this, required only once at the beginning            
        SetReg_32Bits_(MX_GENCNTRL + MX_WRITE)               // write access to EEM General Control Register (MX_GENCNTRL)        
        SetReg_32Bits_(EMU_FEAT_EN | EMU_CLK_EN)         // write into MX_GENCNTRL
  }
  
  //-----------------------------------------------------------------------
  data_16bit
  TCLKset1                   // added for 2xx support
  SetReg_16Bits_(0x4303)
  TCLKset0
  data_capture
  TCLKset1


  // Advance to an instruction load boundary if an interrupt is detected.
  // The CPUOff bit will be cleared.
  // Basically, if there is an interrupt pending, the above dummy instruction
  // will have initialted its processing, and the CPU will not be on an
  // instruction load boundary following the dummy instruction. 
  
  // \Günther: Looks a lot like clckToInstrLoad(), with the exception that clkTclkAndCheckDTC()
  //           is called continuously
  // \WLUT   : Correct;-)
  i = 0;
  cntrl_sig_capture
  SetReg_16Bits(0);
  while(!(lOut & CNTRL_SIG_INSTRLOAD) && (i < MAX_TCE1))
  {
      if(clkTclkAndCheckDTC() != 0)
      {
          return (HALERR_SYNC_JTAG_CONDITIONAL_JTAG_TIMEOUT);
      }
      i++;

      SetReg_16Bits(0);
  }
  
  if(i == MAX_TCE1) 
  {
      return (HALERR_SYNC_JTAG_CONDITIONAL_JTAG_TIMEOUT);
  }
  
  // Read PC now!!! Only the NOP or BIS #0,R4 instruction above was clocked into the device
  // The PC value should now be (OriginalValue + 2)
  // read MAB = PC here
  addr_capture
  SetReg_20Bits(0x0000)
  lOut_long -= 2;
  MyOut[1] = (unsigned short)(lOut_long & 0xFFFF);
  MyOut[2] = (unsigned short)(lOut_long >> 16);
  
  if(i == 0)
  { // An Interrupt was not detected
    // WLUT: lOut does not contain the content of the CNTRL_SIG register anymore at this point
    //       need to capture it again...different to DLLv3 sequence but don't expect any negative effect due to recapturing
      cntrl_sig_capture
      SetReg_16Bits(0x0000)
      if(lOut & CNTRL_SIG_CPU_OFF)
      {
          MyOut[1] += 2;
          
          data_16bit
          TCLKset1               // Added for 2xx support
          SetReg_16Bits(0xC032);
          if(clkTclkAndCheckDTC() != 0)
          {
            return (HALERR_SYNC_JTAG_CONDITIONAL_JTAG_TIMEOUT);
          }

          data_16bit
          TCLKset1               // Added for 2xx support \Günther: Why set TCLK 1 again?
          SetReg_16Bits(0x0010);
          if(clkTclkAndCheckDTC() != 0)
          {
            return (HALERR_SYNC_JTAG_CONDITIONAL_JTAG_TIMEOUT);
          }
          // DLLv2 preserve the CPUOff bit
          statusReg |= STATUS_REG_CPUOFF;
      }
  }
  else
  {
    addr_capture
    SetReg_20Bits(0)
    MyOut[1] = (unsigned short)(lOut_long & 0xFFFF);
    MyOut[2] = (unsigned short)(lOut_long >> 16);
    MyOut[4] = 1; // Inform DLL about interrupt
  }
  
  // DLL v2: deviceHasDTCBug
  //     Configure the DTC so that it transfers after the present CPU instruction is complete (ADC10FETCH).
  //     Save and clear ADC10CTL0 and ADC10CTL1 to switch off DTC (Note: Order matters!).

  // Regain control of the CPU. Read/Write will be set, and source TCLK via TDI.
  cntrl_sig_16bit
  SetReg_16Bits(CNTRL_SIG_TCE1 | CNTRL_SIG_READ | CNTRL_SIG_TAGFUNCSAT)
  
  isInstrLoad              // Test if we are on an instruction load boundary
  if(lIsInstrLoad != 0)
  {
      return (HALERR_INSTRUCTION_BOUNDARY_ERROR);
  }
  
  // Hold Watchdog
  ReadMemWordX(address, MyOut[0]); // safe WDT value
  wdtVal |= (MyOut[0] & 0x00FF); // set original bits in addition to stop bit
  WriteMemWordX(address, wdtVal);
  
  // set PC to a save address pointing to ROM to avoid RAM corruption on certain devices
  SetPcX(ROM_ADDR)

  // read status register
  ReadCpuRegX(2,MyOut[3]);
  MyOut[3] |= statusReg;   // compine with preserved CPUOFF bit setting

  // return output
  STREAM_put_bytes((unsigned char*)MyOut, sizeof(MyOut));

  return (0);
}
