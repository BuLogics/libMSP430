/**
* \ingroup MODULMACROSXV2
*
* \file IsJtagFuseBlown.c
*
* \brief Check if the JTAG fuse is blown
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
#ifdef DB_PRINT
#include "debug.h"
#endif

/**
  IsJtagFuseBlown 

*/

HAL_FUNCTION(_hal_IsJtagFuseBlown)
{
    int  i;
    decl_out;
    
    EDT_TapReset();  // reset TAP state machine -> Run-Test/Idle
    EDT_CheckJtagFuse(); // check security fuse
    
    for (i = 0; i < 3; i++) // First test could be negative.
    {
        cntrl_sig_capture
        SetReg_16Bits(0xaaaa);        
        // If the fuse is blown, the device will be in JTAG bypass mode, and data
        // input to the device will be echoed with a one-bit delay.
        if (lOut == 0x5555)
        {
            STREAM_put_word(lOut); // Fuse is blown.
        }
    }
    STREAM_put_word(lOut); // Fuse is not blown.

    return 0;
}
