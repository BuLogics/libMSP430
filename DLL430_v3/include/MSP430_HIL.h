/*
 * MSP430_HIL.h
 *
 * API for low level HIL access.
 *
 * Copyright (C) 2004 - 2011 Texas Instruments Incorporated - http://www.ti.com/ 
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

/**
\file MSP430_HIL.h
 
 \brief       This file contains the DLL function headers and definitions
              for low level HIL access.
 
 \par         Project:
              MSP-FET430UIF (TI USB FET) HIL API
 
 \par         Developed using:
              MS Visual C++ 2003/2010
 
 \par         Supported API calls:
              - MSP430_HIL_Open()
			  - MSP430_HIL_Connect()
			  -	MSP430_HIL_Bsl()
			  -	MSP430_HIL_Close()

			  -	MSP430_HIL_JTAG_IR()
			  -	MSP430_HIL_JTAG_DR()
			  -	MSP430_HIL_TST()
			  -	MSP430_HIL_TCK()
			  -	MSP430_HIL_TMS()
			  -	MSP430_HIL_TDI()
			  -	MSP430_HIL_RST()
			  -	MSP430_HIL_ResetJtagTap()
*/


#ifndef MSP430_HIL_H
#define MSP430_HIL_H

#include "MSP430.h"

#if defined(__cplusplus)
extern "C" {
#endif


// Functions. -----------------------------------------------------------------
STATUS_T WINAPI MSP430_HIL_Open(void);
STATUS_T WINAPI MSP430_HIL_Connect(void);
//STATUS_T WINAPI MSP430_HIL_Bsl(void);
STATUS_T WINAPI MSP430_HIL_Close(LONG vccOff);

LONG WINAPI MSP430_HIL_JTAG_IR(LONG instruction);
LONG WINAPI MSP430_HIL_JTAG_DR(LONG data, LONG bits);
LONGLONG WINAPI MSP430_HIL_JTAG_DRX(LONGLONG data, LONG bits);
STATUS_T WINAPI MSP430_HIL_TST(LONG state);
STATUS_T WINAPI MSP430_HIL_TCK(LONG state);
STATUS_T WINAPI MSP430_HIL_TMS(LONG state);
STATUS_T WINAPI MSP430_HIL_TDI(LONG state);
STATUS_T WINAPI MSP430_HIL_RST(LONG state);
void WINAPI MSP430_HIL_ResetJtagTap(void);

#if defined(__cplusplus)
}
#endif

#endif // MSP430_HIL_H
