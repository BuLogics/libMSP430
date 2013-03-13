/**
* \ingroup MODULMACROSXV2
*
* \file UnlockDeviceXv2.c
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

HAL_FUNCTION(_hal_UnlockDeviceXv2)
{
    unsigned short passwordLenght= 0x0 ;
    unsigned short i = 0x0;
    unsigned short id = 0;
    decl_out
    decl_jtagMailboxIn
    unsigned short Password[60] = {0};
    unsigned short protocol;
    
    
    // get the protocol
    if(STREAM_get_word(&protocol) < 0)
    {
        return (HALERR_START_JTAG_NO_PROTOCOL);
    }
     // steam get password length
     if(STREAM_get_word(&passwordLenght) != 0)
     {
        return (HALERR_UNLOCK_NO_PASSWORD_LENGTH);
     }      
     // steam get password itself
     for(i =0; i < passwordLenght; i++)
     {
        if(STREAM_get_word(&Password[i]) < 0)
        {
            return (HALERR_UNLOCK_INVALID_PASSWORD_LENGTH);
        }          
     }    
    //--------------------------------------------------------------------------  
    //phase 0 if device was in LPMx5 or camping in an endless loop
    //-------------------------------------------------------------------------- 
     
     EDT_SetProtocol(protocol); // this is jsut for debug must be changed 
     EDT_Open(RSTHIGH);
     EDT_TapReset();  
     //EDT_EnumChain();

    //--------------------------------------------------------------------------  
    //phase 1 of device entry using a user password
    //-------------------------------------------------------------------------- 
     // Apply  4wire/SBW entry Sequence & holt Reset low
     EDT_Open(RSTLOW);
     // reset TAP state machine -> Run-Test/Idle
     EDT_TapReset();  
     // start JTAG mailbox & feed in password exchange request
     i_WriteJmbIn32(MAGIC_PATTERN , DR_JMB_PASSWORD_EXCHANGE_REQUEST);
     // check for Timeout issue during mailbox request 
     if(jtagMailboxIn == 1)
     {
        return (HALERR_JTAG_PASSWORD_WRONG);
     }                            
     id = EDT_Instr(IR_CNTRL_SIG_CAPTURE); 
     
     if(id != 0x91)
     {
        return (HALERR_JTAG_PASSWORD_WRONG);  
     }
     //------------------------------------------------------------------------- 
     //phase 2 of device entry using a user password
     //-------------------------------------------------------------------------
     // Apply again 4wire/SBW entry Sequence & Reset high
     EDT_Open(RSTHIGH); 
     // Reset TAP state machine -> Run-Test/Idle
     EDT_TapReset();  
     i = 0x0;
     // start JTAG mailbox & feed in password & make sure that you are faster
     // then 1.2 seconds if not the bootcode will kick you out & Use 16Bit JBM mode        
     while(i < passwordLenght)
     {
        i_WriteJmbIn(Password[i]) 
        // check for Timeout becuase of wong passord which was shifted in  
        if(jtagMailboxIn == 1)
        {
            return (HALERR_JTAG_PASSWORD_WRONG);
        }
        i++;
    }     
    id = EDT_Instr(IR_CNTRL_SIG_CAPTURE); 
    
    if(id != 0x91)
    {
        return (HALERR_JTAG_PASSWORD_WRONG);  
    }       
     
    return (0);
}
