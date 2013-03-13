/**
* \ingroup MODULMACROSXV2
*
* \file WriteAllCpuRegsXv2.c
*
* \brief Write CPU register values, except R0 and R2
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
#include "../include/hw_compiler_specific.h"
#ifdef DB_PRINT
#include "debug.h"
#endif

/**
  WriteAllCpuRegsXv2
  Write CPU register values, except R0 and R2. This function is for 20bit CPUs.
  inData:  <SP(24)> <Rn(24)>{12}
  outData: -
  SP: stack pointer register (R1)
  Rn: registers R4-R15
*/

HAL_FUNCTION(_hal_WriteAllCpuRegsXv2)
{
  DIAG_SUPPRESS(Pe550)
  decl_out_long
  DIAG_DEFAULT(Pe550)
  unsigned short Registers;
  unsigned long  Rx;
  unsigned short Mova;
  unsigned short Rx_l;
  
  for (Registers = 1; Registers < 16; Registers++)
  {
    if(Registers == 2)
    {
      Registers += 2;
    }
    STREAM_get_byte((unsigned char*)&Rx);
    STREAM_get_byte((unsigned char*)&Rx+1);
    STREAM_get_byte((unsigned char*)&Rx+2);
    *((unsigned char*)&Rx+3) = 0;
    Mova  = 0x0080;
    Mova += (unsigned short)((Rx>>8) & 0x00000F00);
    Mova += (Registers & 0x000F);
    Rx_l  = (unsigned short)Rx;
    WriteCpuRegXv2(Mova, Rx_l);
  }

  return 0;
}

