/**
* \ingroup MODULMACROS
*
* \file EemDataExchangeAFE2xx.c
*
* \brief Streaming read and write of EEM register values
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
#include "EEM_defs.h"

/**
  EemDataExchangeAFE2xx
  Streaming read and write of EEM register values. This function is for
  16bit CPUs.
  inData:  <count(8)> (<addr(8)> [<ivalue(32)>]){*}
  outData: <ovalue(16)>{*}
  count: number of the following exchanges
  addr: address to read from or write to, the LSB marks a read operation
  ivalue: only present for write operations
  ovalue: one for each read operation
*/

//! \todo change API from 16 to 32 bit to be compatible to the X version
//!        of this macro.
//! \todo function mapping in host DLL, currently remapped to EemDataExchangeXv2
//!        to test dll with EW430...for single steps with Xv2 the orginal version
//!        of this macro is required...there must be a bug in the X version!!!
HAL_FUNCTION(_hal_EemDataExchangeAFE2xx)
{
  decl_out
  unsigned char NumberOfExchanges;
  unsigned char registerAddress;
  unsigned short value;  
  
  STREAM_get_byte(&NumberOfExchanges);
  eem_data_exchange
 
  while(NumberOfExchanges)
  {
    STREAM_get_byte(&registerAddress);
    if (registerAddress & 0x01)
    { // read access
      SetReg_16Bits(registerAddress);     // load address
      SetReg_16Bits(0);       // shift in dummy 0
      STREAM_put_word(lOut);  // put output into stream
    }
    else
    { // write access
      STREAM_get_word(&value);

      if (registerAddress == GENCLKCTRL)
      {
          value |= 0x1;
      }

      SetReg_16Bits(registerAddress);   // load address
      SetReg_16Bits(value); // shift in value
    }
    NumberOfExchanges--;
  }
  
  return 0;
}

