/**
* \ingroup MODULMACROSXV2
*
* \file ReadMemWordsXv2.c
*
* \brief Read words (16bit values) from a memory mapped location
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
#include "stream.h"

/**
  ReadMemWordsXv2
  Read words (16bit values) from a memory mapped location.
  inData:  <addr(32)> <length(32)>
  outData: <data(16)>{*}
  addr: the address to start reading from
  length: number of words to read
  data: requested data
*/

HAL_FUNCTION(_hal_ReadMemWordsXv2)
{
  decl_out
  unsigned long i;
  unsigned long lAddr;
  unsigned long lLen;

  STREAM_get_long(&lAddr);
  STREAM_get_long(&lLen);
//! \todo remove lines below after checking JTAG-chain requirements
//  cntrl_sig_16bit      
//  SetReg_16Bits(0x0101 | TCE)  
  TCLKset0
  
  for(i = 0; i < lLen; i++)
  {
    addr_16bit             
    SetReg_20Bits_(lAddr)  
    TCLKset1                    
    TCLKset0
	data_capture
    SetReg_16Bits(0);
    STREAM_put_word(lOut);              
    lAddr += 2;
  }
  TCLKset1
  // one or more cycle, so CPU is driving correct MAB
  TCLK

  return 0;
}

