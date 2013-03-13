/**
* \ingroup MODULMACROSXV2
*
* \file ReadAllCpuRegsNon1377Xv2.c
*
* \brief Read CPU register values, except R0 and R2
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
  ReadAllCpuRegsXv2
  Read CPU register values, except R0 and R2. This function is for 16bit CPUs.
  inData:  -
  outData: <SP(24)> <Rn(24)>{12}
  SP: stack pointer register (R1)
  Rn: registers R4-R15
*/

extern unsigned short romAddressForCpuRead;

HAL_FUNCTION(_hal_ReadAllCpuRegsNon1377Xv2)
{
  decl_out
  unsigned short Registers;
  unsigned short Mova;
  unsigned long Rx;

  // Read the General Purpose registers.
  for (Registers = 1; Registers < 16; Registers++) // Read registers SP, and R4 through R15.
  {
    if(Registers == 2)
    {
      Registers += 2;
    }
    Mova  = 0x0060;
    Mova += (Registers << 8) & 0x0F00;
    ReadCpuRegXv2(Mova, Rx);
    STREAM_put_bytes((unsigned char*)&Rx, 3);
  }

  // all CPU register values have been moved to the JMBOUT register address
  // -> JMB needs to be cleared again!!
  i_ReadJmbOut(Rx)
  return 0;
}

