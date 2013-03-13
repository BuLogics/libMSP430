/*
 * halMain.c
 *
 * <FILE_BRIEF>
 *
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

//! \ingroup MODULHAL
//! \file halMain.c
//! \brief
//!
//! \author  Detlef Fink (03/10/2011)

//! \page hal_page1 Hardware Abstraction Layer (HAL)
//! \brief The HAL ...
//! \author  Detlef Fink (03/10/2011)
//!
//! <b>files</b>\n
//! \li startup.s43
//! \li halMain.c
//! \li hal.c
//! \li halGlobalVars.c

#include "hw_compiler_specific.h"
#include "stream.h"
#include "hal.h"
#include "edt.h"
#include "stdlib.h"
#include "string.h"
#include "HalGlobalVars.h"



// include the version info here to write it to the info mem
#include "../../../DLL430_v3/version.h"

extern void globalVarsInit(void);

extern short _hil_Init( void );
extern unsigned short _hal_mclkCntrl0;
extern unsigned long hil_device_flags_;

// prototypes for core/HAL interface
void *ResetFirmware(void *stream_adr, unsigned long device_flags);

CONST_AT(HAL_INFOS hal_infos_, HAL_ADDR_CONST_HAL_INFOS) = 
{
  ResetFirmware,          // _initTask
  0x8200 | VERSION_PATCH,   // import from DLL by version.h
  VERSION_BUILD           // import from DLL by version.h
};
REQUIRED(hal_infos_)

//HAL signature, indicates HAL part is valid
const unsigned short hal_signature @ "HALSIG" = 0x5137;
REQUIRED(hal_signature)

// vars
#ifndef UNIX
VAR_AT(struct stream_funcs *_stream_Funcs, HAL_ADDR_VAR_STREAM_FUNCS);
#endif
VAR_AT(HAL_INFOS hal_infos_in_ram_, HAL_ADDR_VAR_HAL_INFOS_IN_RAM);
VAR_AT(unsigned short hal_delay_loop_us_, HAL_ADDR_VAR_DELAY_LOOP_US);
VAR_AT(unsigned short hal_delay_loop_ms_, HAL_ADDR_VAR_DELAY_LOOP_MS);

// called via cstartup form biosHalInterfaceInit
REQUIRED(ResetFirmware)
void *ResetFirmware(void *stream_adr, unsigned long device_flags)
{
    globalVarsInit();
    hil_device_flags_ = device_flags;
    if(hil_device_flags_ & DEVICE_FLAG_EASY)
    {
        hal_delay_loop_us_ = 2;
        hal_delay_loop_ms_ = 0x535;
    }
    else
    {
        hal_delay_loop_us_ = 1;
        hal_delay_loop_ms_ = 0x7CF;
    }
    
    memcpy((HAL_INFOS*)&hal_infos_in_ram_,&hal_infos_, sizeof(hal_infos_));
    // save address of stream funcs, located in core
#ifndef UNIX    
    _stream_Funcs=(struct stream_funcs *)stream_adr;
#endif
    _init_Hal();
    if((hal_functions_[sizeof(hal_functions_)/sizeof(HalRec)-1].function == NULL) && !(hil_device_flags_ & DEVICE_FLAG_EASY))
    {
        hal_functions_[sizeof(hal_functions_)/sizeof(HalRec)-1].id = 0xFFFE; 
        hal_functions_[sizeof(hal_functions_)/sizeof(HalRec)-1].function = (void*)_edt_Common_Methods.Loop;     
    }
    _hil_Init();
    _hal_mclkCntrl0=0x040f;
    
    hal_infos_in_ram_.hal_size = sizeof(hal_functions_)/sizeof(HalRec);
    hal_infos_in_ram_.hal_list_ptr = hal_functions_;
    return((void*)&hal_infos_in_ram_); // return software infos
}
