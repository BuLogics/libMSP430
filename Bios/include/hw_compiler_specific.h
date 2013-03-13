/** \ingroup MODULHAL
 * \ingroup MODULBIOS
 * \file hw_compiler_specific.h
 * \brief
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

//! \author  Detlef Fink (03/10/2011)
#ifndef HW_COMPILER_SPECIFIC_H
#define HW_COMPILER_SPECIFIC_H

#ifdef __ICC430__

    #include "msp430.h"
    #include "intrinsics.h"

    #define BAUDRATE        460800          // communication speed to TUSB3410
    
    #define CRYSTAL_U1         8000000         // externally connected HF crystal
    #define CRYSTAL_E2        12000000         // externally connected HF crystal
    #define BAUDMOD_U1         0x52            // corresponding modulation register value
    #define BAUDMOD_E2         0x00            // corresponding modulation register value
    #define USB_READ U0RXBUF
    #define USB_WRITE U0TXBUF
    
    #define INFO_U1_HW_0 0xFF55
    #define INFO_U1_HW_1 0x0140
    #define INFO_E2_HW_0 0xFF45
    #define INFO_E2_HW_1 0x0200
    
    #define CONCAT0(x,y) x##y
    #define CONCAT(x,y) CONCAT0(x,y)
    #define PRAGMA(x) _Pragma(#x)
    #define DIAG_SUPPRESS(x) PRAGMA(diag_suppress=x)
    #define DIAG_DEFAULT(x) PRAGMA(diag_default=x)
    #define REQUIRED(x) PRAGMA(required=x)
    #define INTERRUPT(x) PRAGMA(vector=x) __interrupt
    #define INTERRUPT_PROTO __interrupt
    #define INLINE(x) PRAGMA(inline=x)
    #define VAR_AT(x, y) __no_init x @ y
    #define CONST_AT(x, y) const x @ y
    #define RO_PLACEMENT __ro_placement
    #define NO_INIT __no_init
    #define RO_PLACEMENT_NO_INIT __ro_placement __no_init

    #define ENABLE_INTERRUPT __enable_interrupt()
    #define DISABLE_INTERRUPT __disable_interrupt()
    #define NO_OPERATION __no_operation()

  
    #define PTR_FOR_CMP unsigned short
  
#else
    // dummys for unused function in DLL
    #define DIAG_SUPPRESS(x)
    #define DIAG_DEFAULT(x)
    #define REQUIRED(x)
    #define setRst(x)
    #define setTst(x)
    #define VAR_AT(x, y) x
    #define CONST_AT(x, y) const x
    
    #define PTR_FOR_CMP unsigned long

#endif

#endif
