/**
* \ingroup MODULMACROSXV2
*
* \file ResetXv2.c
*
* \brief Apply an RST NMI to a CPUxv2 device
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
#include "error_def.h"
#include "../include/hw_compiler_specific.h"

/**
   _hal_ResetXv2
   this macro will applay an RST NMI to a CPUxv2 device.
   after the reset the device execution will be stopped
   by using the JTAG mailbox 

*/

HAL_FUNCTION(_hal_ResetXv2)
{      
        decl_out    
        unsigned short id=0;
        decl_jtagMailboxIn
        
        EDT_SetTest(0);
        EDT_Delay_1ms(40);   
        EDT_SetReset(0);           
        EDT_Delay_1ms(40);              
        EDT_SetReset(1);
        EDT_Delay_1ms(40);   
        EDT_SetTest(1); 
        EDT_Delay_1ms(40);   
        
        EDT_Open(RSTLOW);
        EDT_TapReset();      
        EDT_CheckJtagFuse();
        // put in magic pattern to stop user code execution
        i_WriteJmbIn(MAGIC_PATTERN);
        
        if(jtagMailboxIn == 1)
        {
            return HALERR_UNDEFINED_ERROR;    
        } 
        // close connection after the device has been send to LPM4
        EDT_Close();
        
        EDT_Delay_1ms(40);   
        // run entry sequnce but pull rst high during the sequence 
        EDT_Open(RSTHIGH);
        EDT_TapReset();      
        EDT_CheckJtagFuse();

        id = EDT_Instr(IR_CNTRL_SIG_CAPTURE);     
        if (id == JTAGVERSION95 || id == JTAGVERSION91 || id == JTAGVERSION99)
        {         
            return 0;
        }   
        return HALERR_UNDEFINED_ERROR;
}
