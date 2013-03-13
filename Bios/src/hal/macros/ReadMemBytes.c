/**
* \ingroup MODULMACROS
*
* \file ReadMemBytes.c
*
* \brief Read bytes from a memory mapped location
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

/**
  ReadMemBytes
  Read bytes from a memory mapped location.
  inData:  <addr(32)> <length(32)>
  outData: <data(8)>{*}
  addr: the address to start reading from
  length: number of bytes to read
  data: requested data
*/

HAL_FUNCTION(_hal_ReadMemBytes)
{
    decl_out
    short ret_value = 0;
    unsigned long i;
    unsigned long lAddr;
    unsigned long lLen;
    
    if(STREAM_get_long(&lAddr) != 0)
    {
        ret_value = HALERR_READ_MEM_BYTES_NO_ADDRESS;
        goto exit;
    }
    if(STREAM_get_long(&lLen) < 0)
    {
        ret_value = HALERR_READ_MEM_BYTES_NO_SIZE;
        goto exit;
    }
    halt_cpu
    TCLKset0
    cntrl_sig_16bit
    SetReg_16Bits(0x2419)
      
    lLen *= 2; //DLL always sends size in word
    for(i = 0; i < lLen; i++)
    {
        addr_16bit
        SetReg_16Bits((unsigned short)lAddr)
        data_to_addr
        TCLKset1
        TCLKset0
        SetReg_16Bits(0)
        STREAM_put_byte((unsigned char)(lOut & 0xFF));
        lAddr += 1; 
    }
    release_cpu
exit:    
    return(ret_value);
}

