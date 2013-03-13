/*
 * hal_ref.h
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
//!  \file hal_ref.h
//!  
//!  

#ifndef _HAL_REF_H_
#define _HAL_REF_H_

#include "hal.h"
/**
 * HAL Function Prototypes
 */
#define MACRO(x)  short _hal_##x (unsigned short flags);
MACRO(Zero)  
MACRO_LIST
#undef MACRO

void _init_Hal(void);

/**
 * HAL Function Pointer - these are the HAL Exports
 */
// common macros
#define HAL_Zero                              ((HalFuncInOut)hal_functions_[ID_Zero].function)
#define HAL_Init                              ((HalFuncInOut)hal_functions_[ID_Init].function)
#define HAL_SetVcc                            ((HalFuncInOut)hal_functions_[ID_SetVcc].function)
#define HAL_GetVcc                            ((HalFuncInOut)hal_functions_[ID_GetVcc].function)
#define HAL_StartJtag                         ((HalFuncInOut)hal_functions_[ID_StartJtag].function)
#define HAL_StartJtagActivationCode           ((HalFuncInOut)hal_functions_[ID_StartJtagActivationCode].function)
#define HAL_StopJtag                          ((HalFuncInOut)hal_functions_[ID_StopJtag].function)
#define HAL_Configure                         ((HalFuncInOut)hal_functions_[ID_Configure].function)
#define HAL_GetFuses                          ((HalFuncInOut)hal_functions_[ID_GetFuses].function)
#define HAL_BlowFuse                          ((HalFuncInOut)hal_functions_[ID_BlowFuse].function)
#define HAL_WaitForEem                        ((HalFuncInOut)hal_functions_[ID_WaitForEem].function)
#define HAL_BitSequence                       ((HalFuncInOut)hal_functions_[ID_BitSequence].function)
#define HAL_GetJtagId                         ((HalFuncInOut)hal_functions_[ID_GetJtagId].function)
#define HAL_SetDeviceChainInfo                ((HalFuncInOut)hal_functions_[ID_SetDeviceChainInfo].function)
#define HAL_SetChainConfiguration             ((HalFuncInOut)hal_functions_[ID_SetChainConfiguration].function)
#define HAL_GetNumOfDevices                   ((HalFuncInOut)hal_functions_[ID_GetNumOfDevices].function)
#define HAL_GetInterfaceMode                  ((HalFuncInOut)hal_functions_[ID_GetInterfaceMode].function)
#define HAL_MagicPattern                      ((HalFuncInOut)hal_functions_[MagicPattern].function)
// MSP430 architecture
#define HAL_SyncJtag_AssertPor_SaveContext    ((HalFuncInOut)hal_functions_[ID_SyncJtag_AssertPor_SaveContext].function)
#define HAL_SyncJtag_Conditional_SaveContext  ((HalFuncInOut)hal_functions_[ID_SyncJtag_Conditional_SaveContext].function)
#define HAL_RestoreContext_ReleaseJtag        ((HalFuncInOut)hal_functions_[ID_RestoreContext_ReleaseJtag].function)
#define HAL_ReadMemBytes                      ((HalFuncInOut)hal_functions_[ID_ReadMemBytes].function)
#define HAL_ReadMemWords                      ((HalFuncInOut)hal_functions_[ID_ReadMemWords].function)
#define HAL_ReadMemQuick                      ((HalFuncInOut)hal_functions_[ID_ReadMemQuick].function)
#define HAL_WriteMemBytes                     ((HalFuncInOut)hal_functions_[ID_WriteMemBytes].function)
#define HAL_WriteMemWords                     ((HalFuncInOut)hal_functions_[ID_WriteMemWords].function)
#define HAL_EemDataExchange                   ((HalFuncInOut)hal_functions_[ID_EemDataExchange].function)
#define HAL_SingleStep                        ((HalFuncInOut)hal_functions_[ID_SingleStep].function)
#define HAL_ReadAllCpuRegs                    ((HalFuncInOut)hal_functions_[ID_ReadAllCpuRegs].function)
#define HAL_WriteAllCpuRegs                   ((HalFuncInOut)hal_functions_[ID_WriteAllCpuRegs].function)
#define HAL_Psa                               ((HalFuncInOut)hal_functions_[ID_Psa].function)
#define HAL_ExecuteFunclet                    ((HalFuncInOut)hal_functions_[ExecuteFunclet].function)
// MSP430X architecture
#define HAL_SyncJtag_AssertPor_SaveContextX   ((HalFuncInOut)hal_functions_[ID_SyncJtag_AssertPor_SaveContextX].function)
#define HAL_SyncJtag_Conditional_SaveContextX ((HalFuncInOut)hal_functions_[ID_SyncJtag_Conditional_SaveContextX].function)
#define HAL_RestoreContext_ReleaseJtagX       ((HalFuncInOut)hal_functions_[ID_RestoreContext_ReleaseJtagX].function)
#define HAL_ReadMemBytesX                     ((HalFuncInOut)hal_functions_[ID_ReadMemBytesX].function)
#define HAL_ReadMemWordsX                     ((HalFuncInOut)hal_functions_[ID_ReadMemWordsX].function)
#define HAL_ReadMemQuickX                     ((HalFuncInOut)hal_functions_[ID_ReadMemQuickX].function)
#define HAL_WriteMemBytesX                    ((HalFuncInOut)hal_functions_[ID_WriteMemBytesX].function)
#define HAL_WriteMemWordsX                    ((HalFuncInOut)hal_functions_[ID_WriteMemWordsX].function)
#define HAL_EemDataExchangeX                  ((HalFuncInOut)hal_functions_[ID_EemDataExchangeX].function)
#define HAL_SingleStepX                       ((HalFuncInOut)hal_functions_[ID_SingleStepX].function)
#define HAL_ReadAllCpuRegsX                   ((HalFuncInOut)hal_functions_[ID_ReadAllCpuRegsX].function)
#define HAL_WriteAllCpuRegsX                  ((HalFuncInOut)hal_functions_[ID_WriteAllCpuRegsX].function)
#define HAL_PsaX                              ((HalFuncInOut)hal_functions_[ID_PsaX].function)
#define HAL_ExecuteFuncletX                   ((HalFuncInOut)hal_functions_[ExecuteFuncletX].function)
// CoreIp430Xv2 architecture
#define HAL_SyncJtag_AssertPor_SaveContextXv2   ((HalFuncInOut)hal_functions_[ID_SyncJtag_AssertPor_SaveContextXv2].function)
#define HAL_SyncJtag_Conditional_SaveContextXv2 ((HalFuncInOut)hal_functions_[ID_SyncJtag_Conditional_SaveContextXv2].function)
#define HAL_RestoreContext_ReleaseJtagXv2       ((HalFuncInOut)hal_functions_[ID_RestoreContext_ReleaseJtagXv2].function)
#define HAL_ReadMemBytesXv2                     ((HalFuncInOut)hal_functions_[ID_ReadMemBytesXv2].function)
#define HAL_ReadMemWordsXv2                     ((HalFuncInOut)hal_functions_[ID_ReadMemWordsXv2].function)
#define HAL_ReadMemQuickXv2                     ((HalFuncInOut)hal_functions_[ID_ReadMemQuickXv2].function)
#define HAL_WriteMemBytesXv2                    ((HalFuncInOut)hal_functions_[ID_WriteMemBytesXv2].function)
#define HAL_WriteMemWordsXv2                    ((HalFuncInOut)hal_functions_[ID_WriteMemWordsXv2].function)
#define HAL_EemDataExchangeXv2                  ((HalFuncInOut)hal_functions_[ID_EemDataExchangeXv2].function)
#define HAL_SingleStepXv2                       ((HalFuncInOut)hal_functions_[ID_SingleStepXv2].function)
#define HAL_ReadAllCpuRegsXv2                   ((HalFuncInOut)hal_functions_[ID_ReadAllCpuRegsXv2].function)
#define HAL_WriteAllCpuRegsXv2                  ((HalFuncInOut)hal_functions_[ID_WriteAllCpuRegsXv2].function)
#define HAL_PsaXv2                              ((HalFuncInOut)hal_functions_[ID_PsaXv2].function)
#define HAL_ExecuteFuncletXv2                   ((HalFuncInOut)hal_functions_[ID_ExecuteFuncletXv2].function)
#define HAL_UnlockDeviceXv2                     ((HalFuncInOut)hal_functions_[ID_UnlockDeviceXv2].function)
#define HAL_UnlockC092                          ((HalFuncInOut)hal_functions_[ID_UnlockC092].function)
#define HAL_HilCommand                          ((HalFuncInOut)hal_functions_[ID_HilCommand].function)
#define HAL_PollJStateRegFR57xx                 ((HalFuncInOut)hal_functions_[ID_PollJStateRegFR57xx].function)
#define HAL_PollJStateReg                       ((HalFuncInOut)hal_functions_[ID_PollJStateReg].function)

#define HAL_IsJtagFuseBlown                     ((HalFuncInOut)hal_functions_[ID_IsJtagFuseBlown].function)
#define HAL_ResetXv2                            ((HalFuncInOut)hal_functions_[ID_ResetXv2].function)


#define HAL_GetDcoFrequency                     ((HalFuncInOut)hal_functions_[ID_GetDcoFrequency].function)
#define HAL_GetFllFrequency                     ((HalFuncInOut)hal_functions_[ID_GetFllFrequency].function)
#define HAL_WriteFramQuickXv2                   ((HalFuncInOut)hal_functions_[ID_WriteFramQuickXv2].function)
#define HAL_SendJtagMailboxXv2                  ((HalFuncInOut)hal_functions_[ID_SendJtagMailboxXv2].function)

#define HAL_ReadAllCpuRegsNon1377Xv2             ((HalFuncInOut)hal_functions_[ID_ReadAllCpuRegsNon1377Xv].function)
#define HAL_SingleStepJStateXv2                   ((HalFuncInOut)hal_functions_[ID_SingleStepJStateXv2].function)
#endif
 