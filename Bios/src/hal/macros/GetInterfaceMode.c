/**
* \ingroup MODULMACROSXV2
*
* \file GetInterfaceMode.c
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

HAL_FUNCTION(_hal_GetInterfaceMode)
{
    unsigned short id=0, loopCount=7,i=0, protocol =0;
    
    // create known state!
    EDT_Close();  

    for (i = 0; i < loopCount; i++)
    {
        // set JTAG mode 1xx- 4xx
        if(i == 0 || i == 3)// set JTAG
        {
             protocol = JTAG_IF; 
             EDT_SetProtocol(protocol);               
        }
        else 
	    {
            // set SBW2 mode all devices
			if(i == 1 || i == 4)
			{
                 protocol = SPYBIWIRE_IF; 
                 EDT_SetProtocol(protocol);				
			}
            // set SBW4 mode all devices
			else if(i == 2|| i ==5 || i ==6) 
			{
                protocol = SPYBIWIREJTAG_IF; 
			    EDT_SetProtocol(protocol);
			}
        }
        // Run  4wire/SBW entry Sequence & Reset high
        EDT_Open(RSTHIGH); 
        // Reset TAP state machine -> Run-Test/Idle
        EDT_TapReset();  
        // Run Fuse Check
        EDT_CheckJtagFuse();
        // shift out JTAG ID
        id = EDT_Instr(IR_CNTRL_SIG_CAPTURE);    
        // Release JTAG, to set the interface into an known state / 
        EDT_Close();
        // now check for vaild JTAG ID
        if (id == JTAGVERSION || id == JTAGVERSION91 || id == JTAGVERSION95 ||id == JTAGVERSION99 )
        {
            STREAM_put_word(id);
            STREAM_put_word(protocol);
            return 0;
        }  
    }
    // Error no mode found; Just dummy values for id and protocoll
    STREAM_put_word(0xFFFF);
    STREAM_put_word(0xAAAA);
    return HALERR_UNDEFINED_ERROR; 
}
