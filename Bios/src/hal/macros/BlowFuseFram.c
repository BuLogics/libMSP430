/**
* \ingroup <FILEGROUP>
*
* \file BlowFuseFram.c
*
* \brief Blow JTAG security fuse
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
#include "hal_ref.h"
#ifdef DB_PRINT
#include "debug.h"
#endif

/**
  BlowFuse
  Blow JTAG security fuse.
  After calling this function JTAG access is not possible anymore.
  inData:  <tgtHasTest(8)>
  outData: -
  tgtHasTest: specifies if target device has TEST pin or not (bool)
*/

//! \todo implementation & rename to jtag access protection
HAL_FUNCTION(_hal_BlowFuseFram)
{
    short ret_value = 0;
    unsigned short lockKey[2] = {0x5555,0x5555};

    // Write password into device
    WriteMemWordXv2(0xFF80,lockKey[0]);
    WriteMemWordXv2(0xFF82,lockKey[1]);     
        
    // now perform a BOR via JTAG - we loose control of the device then...
    test_reg
    SetReg_32Bits_(0x00000200);

    return ret_value;
}
