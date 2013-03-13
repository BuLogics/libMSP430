/*
 * hal.c
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
//! \file hal.c
//! \author  Detlef Fink (09/21/2010)
//!  

#include "hw_compiler_specific.h"
#include "HalGlobalVars.h"
#include "edt.h"
#include "hal.h"
#include "hal_ref.h"
#include "stream.h"
#include "string.h"


// global variables to handle JTAG chain
VAR_AT(union _chain_Configuration chain_Configuration[32], HAL_ADDR_VAR_CHAIN_CONFIGURATION);
REQUIRED(chain_Configuration)
VAR_AT(unsigned short activeDevice, HAL_ADDR_VAR_ACTIVE_DEVICE);
REQUIRED(activeDevice)
VAR_AT(unsigned char numOfDevices, HAL_ADDR_VAR_NUM_OF_DEVICES);
REQUIRED(numOfDevices)
VAR_AT(unsigned short TCE, HAL_ADDR_VAR_TCE);
REQUIRED(TCE)
VAR_AT(DeviceSettings deviceSettings, HAL_ADDR_VAR_DEVICE_SETTINGS);
REQUIRED(deviceSettings)
VAR_AT(DevicePowerSettings devicePowerSettings, HAL_ADDR_VAR_DEVICE_POWER_SETTINGS);
REQUIRED(devicePowerSettings)
VAR_AT(unsigned short altRomAddressForCpuRead, HAL_ADDR_VAR_ROM_FOR_CPU_READ);
REQUIRED(altRomAddressForCpuRead)

// global variables to handle Emulation clock control
VAR_AT(unsigned short _hal_mclkCntrl0, HALL_ADDR_VAR_HAL_MCLK_CNTRL0);
REQUIRED(_hal_mclkCntrl0)
VAR_AT(HalRec hal_functions_[HAL_FUNCTIONS_SIZE], HAL_ADDR_VAR_HAL_FUNCTION); 

#define MACRO(x)  {ID_##x, (void*)_hal_##x },
CONST_AT(HalRec hal_functions_default_[HAL_FUNCTIONS_DEFAULT_SIZE], HAL_ADDR_CONST_HAL_FUNCTION_DEFAULTS) =
{
    MACRO(Zero)  
    MACRO_LIST
};
#undef MACRO

/*
CONST_AT(HalRec hal_functions_default_[HAL_FUNCTIONS_DEFAULT_SIZE], HAL_ADDR_CONST_HAL_FUNCTION_DEFAULTS) =
{
    { 0x00, (void*)_hal_Zero },
    { 0x01, (void*)_hal_Init },
    { 0x02, (void*)_hal_SetVcc },
    { 0x03, (void*)_hal_GetVcc },
    { 0x04, (void*)_hal_StartJtag },
    { 0x05, (void*)_hal_StopJtag },
    { 0x06, (void*)_hal_Configure },
    { 0x06, (void*)_hal_GetFuses },
    { 0x07, (void*)_hal_BlowFuse },
    { 0x08, (void*)_hal_WaitForEem },
    { 0x09, (void*)_hal_BitSequence },
    { 0x0A, (void*)_hal_GetJtagId },
    { 0x0B, (void*)_hal_SetDeviceChainInfo },
    { 0x0C, (void*)_hal_SetChainConfiguration }, 
    { 0x0D, (void*)_hal_GetNumOfDevices },
    { 0x0E, (void*)_hal_GetInterfaceMode},
    // MSP430 architecture
    { 0x0E, (void*)_hal_SyncJtag_AssertPor_SaveContext },
    { 0x0F, (void*)_hal_SyncJtag_Conditional_SaveContext },
    { 0x10, (void*)_hal_RestoreContext_ReleaseJtag },
    { 0x11, (void*)_hal_ReadMemBytes },
    { 0x12, (void*)_hal_ReadMemWords },
    { 0x13, (void*)_hal_ReadMemQuick },
    { 0x14, (void*)_hal_WriteMemBytes },
    { 0x15, (void*)_hal_WriteMemWords },
    { 0x16, (void*)_hal_WriteMemQuick },
    { 0x17, (void*)_hal_EemDataExchange },
    { 0x18, (void*)_hal_SingleStep },
    { 0x19, (void*)_hal_ReadAllCpuRegs },
    { 0x1A, (void*)_hal_WriteAllCpuRegs },
    { 0x1B, (void*)_hal_Psa },
    { 0x1C, (void*)_hal_SetFrequency },
    { 0x1D, (void*)_hal_EraseFlash },
    { 0x1E, (void*)_hal_WriteFlashBlock },
    { 0x1F, (void*)_hal_WriteFlashWord },
    { 0x20, (void*)_hal_ExecFunclet },
    // MSP430X architecture
    { 0x21, (void*)_hal_SyncJtag_AssertPor_SaveContextX },
    { 0x22, (void*)_hal_SyncJtag_Conditional_SaveContextX },
    { 0x23, (void*)_hal_RestoreContext_ReleaseJtagX },
    { 0x24, (void*)_hal_ReadMemBytesX },
    { 0x25, (void*)_hal_ReadMemWordsX },
    { 0x26, (void*)_hal_ReadMemQuickX },
    { 0x27, (void*)_hal_WriteMemBytesX },
    { 0x28, (void*)_hal_WriteMemWordsX },
    { 0x29, (void*)_hal_WriteMemQuickX },
    { 0x2A, (void*)_hal_EemDataExchangeX },
    { 0x2B, (void*)_hal_SingleStepX },
    { 0x2C, (void*)_hal_ReadAllCpuRegsX },
    { 0x2D, (void*)_hal_WriteAllCpuRegsX },
    { 0x2E, (void*)_hal_PsaX },
    { 0x2F, (void*)_hal_SetFrequencyX },
    { 0x30, (void*)_hal_EraseFlashX },
    { 0x31, (void*)_hal_WriteFlashBlockX },
    { 0x32, (void*)_hal_WriteFlashWordX },
    { 0x33, (void*)_hal_ExecFuncletX },
    // MSP430Xv2 architecture
    { 0x34, (void*)_hal_SyncJtag_AssertPor_SaveContextXv2 },
    { 0x35, (void*)_hal_SyncJtag_Conditional_SaveContextXv2 },
    { 0x36, (void*)_hal_RestoreContext_ReleaseJtagXv2 },
    { 0x37, (void*)_hal_ReadMemBytesXv2 },
    { 0x38, (void*)_hal_ReadMemWordsXv2 },
    { 0x39, (void*)_hal_ReadMemQuickXv2 },
    { 0x3A, (void*)_hal_WriteMemBytesXv2 },
    { 0x3B, (void*)_hal_WriteMemWordsXv2 },
    { 0x3C, (void*)_hal_WriteMemQuickXv2 },
    { 0x3D, (void*)_hal_EemDataExchangeXv2 },
    { 0x3E, (void*)_hal_SingleStepXv2 },
    { 0x3F, (void*)_hal_ReadAllCpuRegsXv2 },
    { 0x40, (void*)_hal_WriteAllCpuRegsXv2 },
    { 0x41, (void*)_hal_PsaXv2 },
    { 0x42, (void*)_hal_SetFrequencyXv2 },
    { 0x43, (void*)_hal_EraseFlashXv2 },
    { 0x44, (void*)_hal_WriteFlashBlockXv2 },
    { 0x45, (void*)_hal_WriteFlashWordXv2 },
    { 0x46, (void*)_hal_ExecFuncletXv2 },
    { 0x47, (void*)_hal_ExecuteFuncletXv2 },
    { 0x48, (void*)_hal_UnlockDeviceXv2 },
};*/

REQUIRED(_init_Hal)


void _init_Hal(void)
{
    unsigned char i;

    for(i=0; i < (sizeof(hal_functions_)/sizeof(HalRec)); i++)
    {
        if(i < (sizeof(hal_functions_default_)/sizeof(HalRec)))
        {
            hal_functions_[i].id = hal_functions_default_[i].id;
            hal_functions_[i].function = hal_functions_default_[i].function;
        }
        else
        {
            hal_functions_[i].id = 0xFFFF; 
            hal_functions_[i].function = NULL;
        }
    }
}

HAL_FUNCTION(_hal_Zero)
{
    return 0;
}

HAL_FUNCTION(_hal_Init)
{
    EDT_Init();
    return 0;
}

HAL_FUNCTION(_hal_SetVcc)
{
    unsigned short vcc;    
    STREAM_get_word(&vcc);   
    EDT_SetVcc(vcc);    
    return 0;
}

HAL_FUNCTION(_hal_GetVcc)
{
    unsigned short vcc = 0;
    unsigned short ext_vcc = 0;

    EDT_GetVcc(&vcc, &ext_vcc);
    STREAM_put_word(vcc);
    STREAM_put_word(ext_vcc);
    return 0;
}

HAL_FUNCTION(_hal_GetFuses)
{
    decl_out
    config_fuses
    
    SetReg_8Bits(0);
    STREAM_put_byte((unsigned char)lOut);
    return 0;
}

/* EOF */
