/**
* \ingroup MODULMACROS
*
* \file Psa.c
*
* \brief Calculate CRC16 checksum over the specified memory area using
* the integrated PSA hardware
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
#include "stddef.h"
#include "JTAG_defs.h"

/**
  Psa
  Calculate CRC16 checksum over the specified memory area using
  the integrated PSA hardware.
  inData:  <addr(32)> <length(32)> <type(8)>
  outData: <psaCrc(16)>
  addr:    start address of CRC calculation (must be even)
  length:  number of words (16bit) to be verified
  type:    specifies the type of verification, !!not required for Xv2!!
           (0 = regular, 1 = enhanced, retrieved from device DB)
  psaCrc:  CRC-16 value
*/

HAL_FUNCTION(_hal_Psa)
{
    //Setup values for watchdog control regsiters
    unsigned char DummyIn[4] = {WDTCTL_ADDRESS & 0xFF, (WDTCTL_ADDRESS >> 8) & 0xFF,
                                        WDTHOLD_DEF, WDTPW_DEF};
    decl_out
    decl_instrLoad
    unsigned long addr;
    unsigned long length;
    StreamSafe stream_tmp;
    short ret_value = 0;
  
    if(STREAM_get_long(&addr) != 0)
    {
        return HALERR_PSA_NO_ADDRESS;
    }
    if(STREAM_get_long(&length) < 0)
    {
        return HALERR_PSA_NO_SIZE;
    }

    STREAM_internal_stream(DummyIn, sizeof(DummyIn), NULL, 0, &stream_tmp);
    ret_value = HAL_SyncJtag_AssertPor_SaveContext(MESSAGE_NEW_MSG | MESSAGE_LAST_MSG);
    STREAM_external_stream(&stream_tmp);
    if(ret_value < 0)
    {
        return ret_value;
    }

    instrLoad
    EDT_SetupPsa(addr);
    
    data_psa

    EDT_StepPsa(length);

    shift_out_psa
    SetReg_16Bits(0)
    STREAM_put_word(lOut);
        
    TCLKset1

    EDT_EndPsa();

    STREAM_internal_stream(DummyIn, sizeof(DummyIn), NULL, 0, &stream_tmp);
    ret_value = HAL_SyncJtag_AssertPor_SaveContext(MESSAGE_NEW_MSG | MESSAGE_LAST_MSG);
    STREAM_external_stream(&stream_tmp);

    return(ret_value);
}
