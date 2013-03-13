/**
* \ingroup MODULMACROS
*
* \file WriteAllCpuRegs.c
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

#include "error_def.h"
#include "../include/hw_compiler_specific.h"
#include "arch.h"
#include "edt.h"
#include "hal.h"
#include "hal_ref.h"
#include "stream.h"

/**
  WriteAllCpuRegsXv2
  Write CPU register values, except R0 and R2. This function is for 20bit CPUs.
  inData:  <SP(32)> <Rn(32)>{12}
  outData: -
  SP: stack pointer register (R1)
  Rn: registers R4-R15
*/

//! \todo change API from 16 to 32 bit to be compatible to the X version
//        of this macro.
//! \todo function mapping in host DLL, currently remapped to WriteAllCpuRegsXv2
HAL_FUNCTION(_hal_WriteAllCpuRegs)
{
    unsigned short Registers;
    unsigned short  Rx;
    short tmp;
    
    for (Registers = 1; Registers < 16; Registers++)
    {
        if(Registers == 2)
        {
            Registers += 2;
        }
        tmp = STREAM_get_word(&Rx);
        if((Registers != 15) && (tmp != 0) || (Registers == 15) && (tmp < 0))
        {
            return HALERR_WRITE_ALL_CPU_REGISTERS_STREAM;
        }
        WriteCpuReg(Registers, Rx);
    }    
    return 0;
}


