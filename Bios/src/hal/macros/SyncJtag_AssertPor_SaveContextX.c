/**
* \ingroup MODULMACROSX
*
* \file SyncJtag_AssertPor_SaveContextX.c
*
* \brief Sync with device, assert a Power On Reset and save processor context
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

#include "error_def.h"
#include "arch.h"
#include "edt.h"
#include "hal.h"
#include "hal_ref.h"
#include "stream.h"
#include "EEM_defs.h"

extern DeviceSettings deviceSettings;

/**
  SyncJtag_AssertPor_SaveContextX
  inData:  <wdtAddr(16)> <wdtCtrl(16)>
  outData: <wdtCtrl(16)> <PC(32)> <SR(16)>
  wdtAddr: watchdog timer address
  wdtCtrl: watchdog timer control value
  PC: program counter register (R0)
  SR: status register (R2)
*/
HAL_FUNCTION(_hal_SyncJtag_AssertPor_SaveContextX)
{
    decl_out
    decl_out_long
    decl_instrLoad
    unsigned short i = 0;
    unsigned short MyOut[4];    //! Output
    unsigned short address;
    unsigned short wdtVal;
    
    // Check input parameters before we proceed
    if(STREAM_get_word(&address) != 0)
    {
        return (HALERR_SYNC_JTAG_ASSERT_POR_NO_WDT_ADDRESS);
    }
    if(STREAM_get_word(&wdtVal) != 0)
    {
        return (HALERR_SYNC_JTAG_ASSERT_POR_NO_WDT_VALUE);
    }
    
    // Sync the JTAG
    cntrl_sig_16bit
    SetReg_16Bits(0x2401)
    cntrl_sig_capture
        
    if(lOut != JTAGVERSION)
    {
        return (HALERR_JTAG_VERSION_MISMATCH);
    }

    cntrl_sig_capture    
    SetReg_16Bits(0x0000)    // read control register once

    TCLKset1
        
    if(!(lOut & CNTRL_SIG_TCE))
    {
        // If the JTAG and CPU are not already synchronized ...
        // Initiate Jtag and CPU synchronization. Read/Write is under CPU control. Source TCLK via TDI.
        // Do not effect bits used by DTC (CPU_HALT, MCLKON).
        cntrl_sig_high_byte
        SetReg_8Bits((CNTRL_SIG_TAGFUNCSAT | CNTRL_SIG_TCE1 | CNTRL_SIG_CPU) >> 8) // initiate CPU synchronization but release low byte of CNTRL sig register to CPU control

        // Address Force Sync special handling
        eem_data_exchange32              // read access to EEM General Clock Control Register (GCLKCTRL)
        SetReg_32Bits_(MX_GCLKCTRL + MX_READ)
        SetReg_32Bits(0)                 // read the content of GCLKCNTRL into lOUt
        // Set Force Jtag Synchronization bit in Emex General Clock Control register.
        lOut |=  0x0040;                 // 0x0040 = FORCE_SYN in DLLv2
        eem_data_exchange32                // Stability improvement: should be possible to remove this, required only once at the beginning
        SetReg_32Bits_(MX_GCLKCTRL + MX_WRITE)   // write access to EEM General Clock Control Register (GCLKCTRL)
        SetReg_32Bits(lOut)              // write into GCLKCNTRL
        // Reset Force Jtag Synchronization bit in Emex General Clock Control register.
        lOut &= ~0x0040;
        eem_data_exchange32                // Stability improvement: should be possible to remove this, required only once at the beginning
        SetReg_32Bits_(MX_GCLKCTRL + MX_WRITE)   // write access to EEM General Clock Control Register (GCLKCTRL)
        SetReg_32Bits(lOut)              // write into GCLKCNTRL

        SyncJtag();
        
        if(!i)
        { // Synchronization failed!
            return (HALERR_SYNC_JTAG_ASSERT_POR_JTAG_TIMEOUT);
        }
    }// end of if(!(lOut & CNTRL_SIG_TCE))

    if(lOut & CNTRL_SIG_CPU_HALT)
    {
        TCLKset0
        cntrl_sig_16bit
        SetReg_16Bits(0x2401)
        TCLKset1
    }
    else
    {
        cntrl_sig_16bit
        SetReg_16Bits(0x2401)
    }
    // execute a dummy instruction here
    data_16bit
    TCLKset1                       // Stability improvement: should be possible to remove this TCLK is already 1
    SetReg_16Bits_(0x4303)         // 0x4303 = NOP
    TCLKset0
    data_capture
    TCLKset1

    // step until next instruction load boundary if not being already there
    instrLoad
    if(lInstrLoad != 0)
    {
        return (HALERR_INSTRUCTION_BOUNDARY_ERROR);
    }

    if (deviceSettings.clockControlType == GCC_EXTENDED)
    {
        eem_data_exchange32
        SetReg_32Bits_(MX_GENCNTRL + MX_WRITE)               // write access to EEM General Control Register (MX_GENCNTRL)
        SetReg_32Bits_(EMU_FEAT_EN | EMU_CLK_EN | CLEAR_STOP | EEM_EN)   // write into MX_GENCNTRL
            
        eem_data_exchange32 // Stability improvement: should be possible to remove this, required only once at the beginning            
        SetReg_32Bits_(MX_GENCNTRL + MX_WRITE)               // write access to EEM General Control Register (MX_GENCNTRL)        
        SetReg_32Bits_(EMU_FEAT_EN | EMU_CLK_EN)         // write into MX_GENCNTRL
    }

    TCLKset0
    cntrl_sig_16bit
    SetReg_16Bits_(CNTRL_SIG_READ | CNTRL_SIG_TCE1 | CNTRL_SIG_PUC | CNTRL_SIG_TAGFUNCSAT); // Assert PUC
    TCLKset1
    cntrl_sig_16bit
    SetReg_16Bits_(CNTRL_SIG_READ | CNTRL_SIG_TCE1 | CNTRL_SIG_TAGFUNCSAT); // Negate PUC

    TCLKset0
    cntrl_sig_16bit
    SetReg_16Bits_(CNTRL_SIG_READ | CNTRL_SIG_TCE1 | CNTRL_SIG_PUC | CNTRL_SIG_TAGFUNCSAT); // Assert PUC
    TCLKset1
    cntrl_sig_16bit
    SetReg_16Bits_(CNTRL_SIG_READ | CNTRL_SIG_TCE1 | CNTRL_SIG_TAGFUNCSAT); // Negate PUC

    // Explicitly set TMR
    SetReg_16Bits(CNTRL_SIG_READ | CNTRL_SIG_TCE1); // Enable access to Flash registers
        
    flash_16bit_update               // Disable flash test mode
    SetReg_16Bits_(FLASH_SESEL1)     // Pulse TMR
    SetReg_16Bits_(FLASH_SESEL1 | FLASH_TMR)
    SetReg_16Bits_(FLASH_SESEL1)
    SetReg_16Bits_(FLASH_SESEL1 | FLASH_TMR) // Set TMR to user mode
        
    cntrl_sig_high_byte
    SetReg_8Bits_((CNTRL_SIG_TAGFUNCSAT | CNTRL_SIG_TCE1) >> 8) // Disable access to Flash register

    // step until an appropriate instruction load boundary
    for(i = 0; i < 10; i++)
    {
        addr_capture
        SetReg_20Bits(0x0000)
        if(lOut_long == 0xFFFE || lOut_long == 0x0F00)
        {
            break;
        }
        TCLK
    }
    if(i == 10)
    {
        return (HALERR_INSTRUCTION_BOUNDARY_ERROR);
    }
    TCLK
    TCLK

    TCLKset0
    addr_capture
    SetReg_16Bits(0x0000)
    TCLKset1

    // step until next instruction load boundary if not being already there
    instrLoad
    if(lInstrLoad != 0)
    {
        return (HALERR_INSTRUCTION_BOUNDARY_ERROR);
    }

    // Hold Watchdog
    ReadMemWordX(address, MyOut[0]); // safe WDT value
    wdtVal |= (MyOut[0] & 0x00FF); // set original bits in addition to stop bit
    WriteMemWordX(address, wdtVal);

    // read MAB = PC here
    addr_capture
    SetReg_20Bits(0)
    MyOut[1] = (unsigned short)(lOut_long & 0xFFFF);
    MyOut[2] = (unsigned short)(lOut_long >> 16);

    
    // set PC to a save address pointing to ROM to avoid RAM corruption on certain devices
    SetPcX(ROM_ADDR)

    {
        unsigned long Rx;
        // read status register
        ReadCpuRegX(2,Rx);
        MyOut[3] = (unsigned short)Rx;
    }

    // return output
    STREAM_put_bytes((unsigned char*)MyOut,8);
  
    return(0);
}

