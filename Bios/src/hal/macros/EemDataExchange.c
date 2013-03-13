/**
* \ingroup MODULEMACROS
*
* \file EemDataExchange.c
*
* \brief Streaming read and write of EEM register values
*
*/
/*
 * EemDataExchange.c
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

#include "arch.h"
#include "edt.h"
#include "hal.h"
#include "hal_ref.h"
#include "stream.h"

/**
  EemDataExchange
  Streaming read and write of EEM register values. This function is for
  16bit CPUs.
  inData:  <count(8)> (<addr(8)> [<ivalue(16)>]){*}
  outData: <ovalue(16)>{*}
  count: number of the following exchanges
  addr: address to read from or write to, the LSB marks a read operation
  ivalue: only present for write operations
  ovalue: one for each read operation
*/

extern unsigned short lastTraceWritePos;

HAL_FUNCTION(_hal_EemDataExchange)
{
  decl_out
  unsigned char NumberOfExchanges;
  unsigned char tmp_char;  
  unsigned short  tmp_uint;  
  
  STREAM_get_byte(&NumberOfExchanges);
  eem_data_exchange
 
  while(NumberOfExchanges)
  {
    STREAM_get_byte(&tmp_char);
    if(tmp_char & 0x01)
    { // read access
      SetReg_16Bits(tmp_char);   // load address
      SetReg_16Bits(0);       // shift in dummy 0
      STREAM_put_word(lOut); // put output into stream
    }
    else
    { // write access
      STREAM_get_word(&tmp_uint);
      SetReg_16Bits(tmp_char);           // load address
      SetReg_16Bits(tmp_uint);  // shift in value
      
      //Reset when state storage is reset
      if (tmp_char == 0x9E && tmp_uint & 0x40)
      {
          lastTraceWritePos = 0;
      }
    }
    NumberOfExchanges--;
  }
  
  return 0;
}

