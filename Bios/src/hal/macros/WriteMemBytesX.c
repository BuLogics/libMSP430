/**
* \ingroup MODULMACROSX
*
* \file WriteMemBytesX.c
*
* \brief Write bytes to a memory mapped location
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
#include "stream.h"
#ifdef DB_PRINT
#include "debug.h"
#endif

HAL_FUNCTION(_hal_WriteMemBytesX)
{
    static unsigned long lLen;
    static unsigned long lAddr;
    short ret_value = 0;
    unsigned char tmp_uchar;
    
    if(flags & MESSAGE_NEW_MSG)
    {
        if(STREAM_get_long(&lAddr) != 0)
        {
            return HALERR_WRITE_MEM_WORD_NO_RAM_ADDRESS;
        }
        if(STREAM_get_long(&lLen) == -1)
        {
            return HALERR_WRITE_MEM_WORD_NO_RAM_SIZE;
        }
        halt_cpu
        TCLKset0
        cntrl_sig_low_byte
        SetReg_16Bits_(0x18)
    }
    
    lLen *= 2; //DLL always sends size in word
    for(; lLen && (ret_value == 0); lLen--)
    {
        ret_value = STREAM_get_byte(&tmp_uchar);
        addr_16bit
        SetReg_20Bits_(lAddr)
        data_to_addr
        SetReg_8Bits_(tmp_uchar);
        TCLKset1
        TCLKset0
        lAddr += 1;
    }

    if(flags & MESSAGE_LAST_MSG)
    {
        release_cpu
    }
    else if(ret_value == 1)
    {
        STREAM_out_change_type(RESPTYP_ACKNOWLEDGE);
        ret_value = 0;
    }
    else
    {
        ret_value = HALERR_WRITE_MEM_WORD_UNKNOWN;
    }
    return(ret_value);
}

