/**
* \ingroup MODULMACROSXV2
*
* \file PsaXv2.c
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

#include "arch.h"
#include "edt.h"
#include "hal.h"
#include "hal_ref.h"
#include "stream.h"
#include "stddef.h"

extern unsigned char mclk_modules[16];

/**
  PsaXv2
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
HAL_FUNCTION(_hal_PsaXv2)
{
  //Setup values for watchdog control regsiters
  unsigned char DummyIn[21] = {WDTCTL_ADDRESS_5XX & 0xFF, (WDTCTL_ADDRESS_5XX >> 8) & 0xFF,
                                        WDTHOLD_DEF, WDTPW_DEF};
  decl_out
  unsigned long addr;
  unsigned long length;
  int i;
  StreamSafe stream_tmp;
  
  STREAM_get_long(&addr);
  STREAM_get_long(&length);
  
  for(i=0;i<16;i++)
  {
    DummyIn[5+i]=mclk_modules[i];
  }
  STREAM_internal_stream(DummyIn, sizeof(DummyIn), NULL, 0, &stream_tmp);
  HAL_SyncJtag_AssertPor_SaveContextXv2(MESSAGE_NEW_MSG | MESSAGE_LAST_MSG);
  STREAM_external_stream(&stream_tmp);

  {
    unsigned short Mova;
    unsigned short Pc_l;

    Mova  = 0x0080;
    Mova += (unsigned short)(((addr)>>8) & 0x00000F00);
    Pc_l  = (unsigned short)(((addr) & 0xFFFF));
    /**
      * Start of macro: i_SetPc
      */
    SetPcXv2(Mova, Pc_l);
    TCLKset1
    cntrl_sig_16bit
    SetReg_16Bits_(0x0501)
    /**
      * End of macro: i_SetPc
      */
  }

  data_16bit
  SetReg_16Bits_((unsigned short)(addr - 2));
  data_psa

  EDT_StepPsa(length);
  
  shift_out_psa
  SetReg_16Bits(0)
  STREAM_put_word(lOut);
  
  for(i=0;i<16;i++)
  {
    DummyIn[5+i] = mclk_modules[i];
  }
  STREAM_internal_stream(DummyIn, sizeof(DummyIn), NULL, 0, &stream_tmp);
  HAL_SyncJtag_AssertPor_SaveContextXv2(MESSAGE_NEW_MSG | MESSAGE_LAST_MSG);
  STREAM_external_stream(&stream_tmp);
  
  return 0;
}
