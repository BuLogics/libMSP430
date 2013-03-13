/*
 * hal_require.h
 * 
 * <FILEBRIEF>
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


#ifndef _HAL_REQUIRE_H_
#define _HAL_REQUIRE_H_

// common macros
#pragma required=_hal_Init
#pragma required=_hal_SetVcc
#pragma required=_hal_GetVcc
#pragma required=_hal_StartJtag
#pragma required=_hal_StopJtag
#pragma required=_hal_GetFuses
#pragma required=_hal_BlowFuse
#pragma required=_hal_WaitForEem
#pragma required=_hal_BitSequence
#pragma required=_hal_GetJtagId
#pragma required=_hal_SetDeviceChainInfo
#pragma required=_hal_SetChainConfiguration
#pragma required=_hal_GetNumOfDevices
#pragma required=_hal_GetInterfaceMode
// MSP430 architecture
#pragma required=_hal_SyncJtag_AssertPor_SaveContext
#pragma required=_hal_SyncJtag_Conditional_SaveContext
#pragma required=_hal_RestoreContext_ReleaseJtag
#pragma required=_hal_ReadMemBytes
#pragma required=_hal_ReadMemWords
#pragma required=_hal_ReadMemQuick
#pragma required=_hal_WriteMemBytes
#pragma required=_hal_WriteMemWords
#pragma required=_hal_EemDataExchange
#pragma required=_hal_SingleStep
#pragma required=_hal_ReadAllCpuRegs
#pragma required=_hal_WriteAllCpuRegs
#pragma required=_hal_Psa
#pragma required=_hal_WriteFlashBlock
#pragma required=_hal_WriteFlashWord
#pragma required=_hal_ExecFunclet
// MSP430X architecture
#pragma required=_hal_SyncJtag_AssertPor_SaveContextX
#pragma required=_hal_SyncJtag_Conditional_SaveContextX
#pragma required=_hal_RestoreContext_ReleaseJtagX
#pragma required=_hal_ReadMemBytesX
#pragma required=_hal_ReadMemWordsX
#pragma required=_hal_ReadMemQuickX
#pragma required=_hal_WriteMemBytesX
#pragma required=_hal_WriteMemWordsX
#pragma required=_hal_EemDataExchangeX
#pragma required=_hal_SingleStepX
#pragma required=_hal_ReadAllCpuRegsX
#pragma required=_hal_WriteAllCpuRegsX
#pragma required=_hal_PsaX
#pragma required=_hal_WriteFlashBlockX
#pragma required=_hal_WriteFlashWordX
#pragma required=_hal_ExecFuncletX
// MSP430Xv2 architecture
#pragma required=_hal_SyncJtag_AssertPor_SaveContextXv2
#pragma required=_hal_SyncJtag_Conditional_SaveContextXv2
#pragma required=_hal_RestoreContext_ReleaseJtagXv2
#pragma required=_hal_ReadMemBytesXv2
#pragma required=_hal_ReadMemWordsXv2
#pragma required=_hal_ReadMemQuickXv2
#pragma required=_hal_WriteMemBytesXv2
#pragma required=_hal_WriteMemWordsXv2
#pragma required=_hal_EemDataExchangeXv2
#pragma required=_hal_SingleStepXv2
#pragma required=_hal_ReadAllCpuRegsXv2
#pragma required=_hal_WriteAllCpuRegsXv2
#pragma required=_hal_PsaXv2
#pragma required=_hal_WriteFlashBlockXv2
#pragma required=_hal_WriteFlashWordXv2
#pragma required=_hal_ExecFuncletXv2
#pragma required=_hal_ExecuteFuncletXv2
#pragma required=_hal_UnlockDeviceXv2

#endif

