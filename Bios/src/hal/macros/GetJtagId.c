/**
* \ingroup MODULMACROS
*
* \file GetJtagId.c
*
* \brief <FILEBRIEF>
*
*/
/*
 * Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/ 
 * 
 * 
 *  Redistribution and use in source and binary forms, with or without 
 *  modification, are permittUpdate License Headered provided that the following conditions 
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
#include "JTAG_defs.h"
#include "error_def.h"
#include "../include/hw_compiler_specific.h"

/**
  GetJtagId: required as seperate macro beside StartJtag to get the JtagId
             of an individual device in the JTAG chain
*/
HAL_FUNCTION(_hal_GetJtagId)
{
    short i;
    unsigned short JtagId = 0;
    unsigned short CoreIpId = 0; 
    long DeviceIpPointer = 0; 
    long IdDataAddr = 0;
      
    decl_out
    decl_out_long
  
	//To make really sure to get the valid information
    for (i = 0; i < 4; )
    {
        JtagId = EDT_Instr(IR_CNTRL_SIG_CAPTURE);    
        // now get device id pointer and core ip id
        if (JtagId == JTAGVERSION || JtagId == JTAGVERSION8D )
        {
            CoreIpId = 0;
            DeviceIpPointer = 0;
            IdDataAddr = 0xFF0;
            
            //retrun values
            STREAM_put_word(JtagId);
            STREAM_put_word(CoreIpId);
            STREAM_put_long(DeviceIpPointer);
            STREAM_put_long(IdDataAddr);
            return 0;
        }       
        else if(JtagId == JTAGVERSION91 || JtagId == JTAGVERSION99 || JtagId == JTAGVERSION95)
        {                      
            // Get Core identification info
            core_ip_pointer     
            SetReg_16Bits(0);
            CoreIpId = lOut;
            // Get device identification pointer
            device_ip_pointer 
            SetReg_20Bits(0);     
            DeviceIpPointer = lOut_long;
            // The ID pointer is an un-scrambled 20bit value
            DeviceIpPointer = ((DeviceIpPointer & 0xFFFF) << 4 )  + (DeviceIpPointer >> 16 );            
            if(DeviceIpPointer)
            {
                IdDataAddr = DeviceIpPointer + 4;
            }            
            //retrun values
            STREAM_put_word(JtagId);
            STREAM_put_word(CoreIpId);
            STREAM_put_long(DeviceIpPointer);
            STREAM_put_long(IdDataAddr);           
            return 0;
        }
        else
        {
            i++; // no valid JtagId was found
        }           
    }// end of JTAG id scan
     return HALERR_UNDEFINED_ERROR;    
}
