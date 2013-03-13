/**
* \ingroup MODULMACROSX
*
* \file RestoreContext_ReleaseJtagX.c
*
* \brief Restore the CPU context and releast Jtag control
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


HAL_FUNCTION(_hal_RestoreContext_ReleaseJtagX)
{
    unsigned short wdt_addr;
    unsigned short wdt_value;
    unsigned long  pc;
    unsigned short sr;
    unsigned short control_mask;
    unsigned short mdb;
    unsigned short releaseJtag;

    if(STREAM_get_word(&wdt_addr) != 0)
    {
      return HALERR_RESTORECONTEXT_RELEASE_JTAG_NO_WDT_ADDRESS;
    }
    if(STREAM_get_word(&wdt_value) != 0)
    {
      return HALERR_RESTORECONTEXT_RELEASE_JTAG_NO_WDT_VALUE;
    }
    if(STREAM_get_long(&pc) != 0)
    {
      return HALERR_RESTORECONTEXT_RELEASE_JTAG_NO_PC;
    }
    if(STREAM_get_word(&sr) != 0)
    {
      return HALERR_RESTORECONTEXT_RELEASE_JTAG_NO_SR;
    }
    if(STREAM_get_word(&control_mask) != 0)
    {
      return HALERR_RESTORECONTEXT_RELEASE_JTAG_NO_CONTROL_MASK;
    }
    if(STREAM_get_word(&mdb) == -1)
    {
      return HALERR_RESTORECONTEXT_RELEASE_JTAG_NO_MDB;
    }
    if (STREAM_get_word(&releaseJtag) == -1)
    {
      releaseJtag = 0;
    }

    // Write back Status Register
    WriteCpuRegX(2, sr)

    // Restore Watchdog Control Register
    WriteMemWordX(wdt_addr, wdt_value);

    // restore Program Counter
    SetPcX(pc);

    if (deviceSettings.clockControlType == GCC_EXTENDED)
    {
        eem_data_exchange32
        SetReg_32Bits_(MX_GENCNTRL + MX_WRITE)               // write access to EEM General Control Register (MX_GENCNTRL)
        SetReg_32Bits_(EMU_FEAT_EN | EMU_CLK_EN | CLEAR_STOP | EEM_EN)
    }

    // activate EEM
    eem_write_control
    SetReg_16Bits_(control_mask)
    
    // Pre-initialize MDB before release if
    if(mdb)
    {
        data_16bit
        SetReg_16Bits_(mdb)
        TCLKset0
        addr_capture
        TCLKset1
    }
    else
    {
      addr_capture
    }

    // release target device from JTAG control
    cntrl_sig_release
      
    if(releaseJtag)
    {
        EDT_Close(); // release JTAG on go 
    }

    return(0);
}
