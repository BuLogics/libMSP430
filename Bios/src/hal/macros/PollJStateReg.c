/**
* \ingroup MODULMACROS
*
* \file PollJStateReg.c
*
* \brief <FILEBRIEF>
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

#include "hw_compiler_specific.h"
#include "HalGlobalVars.h"
#include "arch.h"
#include "edt.h"
#include "hal.h"
#include "hal_ref.h"
#include "stream.h"
#include "stddef.h"
#include "error_def.h"
#include "JTAG_defs.h"

/**
  PollJStateReg
  Queries the JSTATE register and reports any changes in the relevant bits
  inData:  <maskLow(32)> <maskHigh(32)> <forceSendState(16)>
  outData: <captureFlag(16)> <JStateLow(32)> <JStateHigh(32)>
*/

#define BP_HIT_MASK 0x100000000ull
#define LPMX5_MASK 0x8000000000000000ull

HAL_FUNCTION(_hal_PollJStateReg)
{
    decl_out
    decl_out_long
    decl_out_long_long
    short RetState = HALERR_UNDEFINED_ERROR;
    unsigned long  lMaskLow = 0,  lMaskHigh = 0;
    unsigned long long JStateMask = 0;  
    unsigned short forceSendState = 0;    
    StreamSafe stream_tmp;

    // Contains the previously recorded JSTATE
    static unsigned long long prevJState = 0x0000000000000000;

    STREAM_get_long(&lMaskLow); 
    STREAM_get_long(&lMaskHigh); 
    
    STREAM_get_word(&forceSendState);     

    JStateMask = ((unsigned long long)lMaskHigh << 32) | lMaskLow;

    // Query new JSTATE
    jstate_read
    SetReg_64Bits(0x0000000000000000)
    
    if(!forceSendState && ((prevJState & BP_HIT_MASK) != (lOut_long_long & BP_HIT_MASK))&&(lOut_long_long & BP_HIT_MASK))
    {
        STREAM_put_word(BP_HIT_FLAG);   
        STREAM_put_word(0x80);             
        STREAM_put_long(lOut_long_long & 0xFFFFFFFF);
        STREAM_put_long(lOut_long_long >> 32);
        
        RetState = 1;
    }  
    else if(forceSendState || ((prevJState & LPMX5_MASK) != (lOut_long_long & LPMX5_MASK)))
    {
        STREAM_put_word(JSTATE_CAPTURE_FLAG);         
        STREAM_put_long(lOut_long_long & 0xFFFFFFFF);
        STREAM_put_long(lOut_long_long >> 32);
        
        if (((lOut_long_long & LPMX5_MASK) == 0) && !forceSendState)
        {   // Device woke up on its own, so sync again
            //Setup values for watchdog control regsiters
            unsigned char DummyIn[8] = {WDTCTL_ADDRESS_5XX & 0xFF,(WDTCTL_ADDRESS_5XX >> 8) & 0xFF,
                                     WDTHOLD_DEF,WDTPW_DEF,0,0,0,0};
            unsigned short read = 0;		
            unsigned short Mova;
            unsigned short Pc_l;

            STREAM_internal_stream(DummyIn, sizeof(DummyIn), NULL, 0, &stream_tmp);
            HAL_SyncJtag_Conditional_SaveContextXv2(MESSAGE_NEW_MSG | MESSAGE_LAST_MSG);
            STREAM_external_stream(&stream_tmp);

            // Read and Set the Program counter
            ReadMemWordXv2(0xFFFE, read)
            Mova  = 0x0080;
            Mova += (unsigned short)((read >>  8) & 0x00000F00);
            Pc_l  = (unsigned short)((read & 0xFFFF));
            SetPcXv2(Mova, (Pc_l+2));

            // Set status register            
            WriteCpuRegXv2(2,0)            
        }        
        RetState = 1;
    }
    else
    {
        RetState = 2;
    }
    
    if(forceSendState)
    {
        RetState = 0;
    }
    else
    {
        prevJState = lOut_long_long;
    }
  
    return RetState;
}
