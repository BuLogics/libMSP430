/*
 * hal.h
 *
 * Copyright (C) 2007 - 2011 Texas Instruments Incorporated - http://www.ti.com/ 
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
 @page hal Hardware Abstraction Layer (HAL)
 This page introduces the user to the topic.
*/

/**
 @file hal.h
 @brief Hardware Abstraction Layer (HAL)

 This file defines and exports all the objects and symbols required to use
 the Hardware Abstraction Layer (HAL).
*/

#ifndef _HAL_H_
#define _HAL_H_

/// Configurations values for StartJtag
enum INTERFACE_TYPE
{
  /// 4 Wire JTAG protocol used
  JTAG_IF = 0,
  /// 2 Wire (Spy-bi-wire) JTAG protocol used
  SPYBIWIRE_IF,
  /// 2 Wire Devices accessed by 4wire JTAG protocol
  SPYBIWIREJTAG_IF
};

// Event types  send async to DLL First word in message 
enum EVENT_TYPE_FLAGS
{
    // set if breakpoint is hit
    BP_HIT_FLAG = 0x1,
    // State Sotrage Event
    STATE_STORAGE_FLAG = 0x2,
    //JSTATE Capute
    JSTATE_CAPTURE_FLAG = 0x4,
	// Variable watch event
    VARIABLE_WATCH_FLAG = 0x8
};

/// FLASH erase type.
enum ERASE_TYPE
{
  /// Erase a segment.
  ERASE_SEGMENT = 0,  
  /// Erase all MAIN memory.
  ERASE_MAIN,     
  /// Erase all MAIN and INFORMATION memory.
  ERASE_ALL,
  /// Erase all MAIN and IP PROTECTED memory.
  ERASE_TOTAL,
};


enum MAILBOX_MODE
{
	LONG_MODE = 0, 
	WORD_MODE,
};

#define NO_FLAG			0x00
#define LOCK_INFO_A_FLAG	0x01

#define HAL_FUNCTION(x) short x (unsigned short flags)
typedef short (*HalFuncInOut)(unsigned short);

union _chain_Configuration{
  struct __chain_Configuration{
           unsigned char Num      : 7;
           unsigned char isMsp430 : 1;
  }bitAccess;
  unsigned char byteAccess;
};

struct _DeviceSettings_
{
    unsigned long  clockControlType;    
    unsigned short stopFLL;
};
typedef struct _DeviceSettings_ DeviceSettings;

struct _DevicePowerSettings_
{
    unsigned long powerTestRegMask;
    unsigned long enableLpmx5TestReg;
    unsigned long disableLpmx5TestReg;

    unsigned short powerTestReg3VMask;
    unsigned short enableLpmx5TestReg3V;
    unsigned short disableLpmx5TestReg3V;    
};
typedef struct _DevicePowerSettings_ DevicePowerSettings;

#ifndef HAL_REC
#define HAL_REC
struct _HalRec_
{
  unsigned short id;
  void  *function;
};
typedef struct _HalRec_ HalRec;
#endif

extern void _init_Hal(void);

#define MACRO_LIST                           \
    MACRO(Init)                              \
    MACRO(SetVcc)                            \
    MACRO(GetVcc)                            \
    MACRO(StartJtag)                         \
    MACRO(StartJtagActivationCode)           \
    MACRO(StopJtag)                          \
    MACRO(Configure)                         \
    MACRO(GetFuses)                          \
    MACRO(BlowFuse)                          \
    MACRO(WaitForEem)                        \
    MACRO(BitSequence)                       \
    MACRO(GetJtagId)                         \
    MACRO(SetDeviceChainInfo)                \
    MACRO(SetChainConfiguration)             \
    MACRO(GetNumOfDevices)                   \
    MACRO(GetInterfaceMode)                  \
    MACRO(SyncJtag_AssertPor_SaveContext)    \
    MACRO(SyncJtag_Conditional_SaveContext)  \
    MACRO(RestoreContext_ReleaseJtag)        \
    MACRO(ReadMemBytes)                      \
    MACRO(ReadMemWords)                      \
    MACRO(ReadMemQuick)                      \
    MACRO(WriteMemBytes)                     \
    MACRO(WriteMemWords)                     \
    MACRO(EemDataExchange)                   \
    MACRO(EemDataExchangeAFE2xx)             \
    MACRO(SingleStep)                        \
    MACRO(ReadAllCpuRegs)                    \
    MACRO(WriteAllCpuRegs)                   \
    MACRO(Psa)                               \
    MACRO(ExecuteFunclet)                    \
    MACRO(ExecuteFuncletJtag)                \
    MACRO(GetDcoFrequency)                   \
    MACRO(GetDcoFrequencyJtag)               \
    MACRO(GetFllFrequency)                   \
    MACRO(GetFllFrequencyJtag)               \
    MACRO(WaitForStorage)                    \
    MACRO(SyncJtag_AssertPor_SaveContextX)   \
    MACRO(SyncJtag_Conditional_SaveContextX) \
    MACRO(RestoreContext_ReleaseJtagX)       \
    MACRO(ReadMemBytesX)                     \
    MACRO(ReadMemWordsX)                     \
    MACRO(ReadMemQuickX)                     \
    MACRO(WriteMemBytesX)                    \
    MACRO(WriteMemWordsX)                    \
    MACRO(EemDataExchangeX)                  \
    MACRO(SingleStepX)                       \
    MACRO(ReadAllCpuRegsX)                   \
    MACRO(WriteAllCpuRegsX)                  \
    MACRO(PsaX)                              \
    MACRO(ExecuteFuncletX)                   \
    MACRO(GetDcoFrequencyX)                  \
    MACRO(GetFllFrequencyX)                  \
    MACRO(WaitForStorageX)                    \
    MACRO(BlowFuseXv2)                       \
    MACRO(BlowFuseFram)                      \
    MACRO(SyncJtag_AssertPor_SaveContextXv2) \
    MACRO(SyncJtag_Conditional_SaveContextXv2)\
    MACRO(RestoreContext_ReleaseJtagXv2)      \
    MACRO(ReadMemWordsXv2)                    \
    MACRO(ReadMemQuickXv2)                    \
    MACRO(WriteMemWordsXv2)                   \
    MACRO(EemDataExchangeXv2)                 \
    MACRO(SingleStepXv2)                      \
    MACRO(ReadAllCpuRegsXv2)                  \
    MACRO(WriteAllCpuRegsXv2)                 \
    MACRO(PsaXv2)                             \
    MACRO(ExecuteFuncletXv2)                  \
    MACRO(UnlockDeviceXv2)                    \
    MACRO(MagicPattern)			              \
    MACRO(UnlockC092)			              \
    MACRO(HilCommand)                         \
    MACRO(PollJStateReg)                      \
    MACRO(PollJStateRegFR57xx)                \
    MACRO(IsJtagFuseBlown)                    \
    MACRO(ResetXv2)                           \
    MACRO(WriteFramQuickXv2)                  \
    MACRO(SendJtagMailboxXv2)                 \
    MACRO(SingleStepJStateXv2)

      
#define MACRO(x)  ID_##x,
enum hal_id
{
    MACRO(Zero)  
    MACRO_LIST
    NUM_MACROS
};
#undef MACRO


#define HAL_FUNCTIONS_DEFAULT_SIZE    NUM_MACROS
#define HAL_FUNCTIONS_SIZE   (NUM_MACROS+2)

extern HalRec hal_functions_[HAL_FUNCTIONS_SIZE];

#endif

