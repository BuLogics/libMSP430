/*
 * HalGlobalVars.h
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


//! \ingroup MODULHAL
//! \file HalGlobalVars.h
//! \brief
//!
//! \author  Detlef Fink (03/10/2011)

// var addresses                      
// unsigned char                      
#define HAL_ADDR_VAR_TCLK_SAVED       0x1100
// unsigned char[16]
#define HAL_ADDR_VAR_MCLK_MODULES     0x1102
// *to_struc
#define HAL_ADDR_VAR_STREAM_FUNCS     0x1112
// struct (10 Bytes)
#define HAL_ADDR_VAR_HAL_INFOS_IN_RAM 0x1114
// union (char) 32 Bytes
#define HAL_ADDR_VAR_CHAIN_CONFIGURATION 0x111E
// unsigned short
#define HAL_ADDR_VAR_ACTIVE_DEVICE 0x113E
// unsigned char
#define HAL_ADDR_VAR_NUM_OF_DEVICES 0x1140
// unsigned char
#define HAL_ADDR_ENTI2TDO 0x1141
// unsigned short
#define HAL_ADDR_VAR_TCE 0x1142
// unsigned short
#define HALL_ADDR_VAR_HAL_MCLK_CNTRL0 0x1144
// 512
#define HAL_ADDR_VAR_HAL_FUNCTION 0x1146
// unsigned short
#define HAL_ADDR_VAR_QPROTOCOL_ID 0x1346
// unsigned short
#define HAL_ADDR_VAR_B_VCC_ON 0x134A
// unsigned char*
#define HL_ADDR_JTAG_PORT_OUT 0x134C
// 12 bytes
#define HAL_ADDR_VAR_EDT_DISTINCT_METHODS 0x134E
// unsigned long
#define HAL_ADDR_VAR_DEVICE_FLAGS 0x136C
// unsigned char *
#define HAL_ADDR_TSTCTRL_PORT 0x1370
// unsigned short
#define HAL_ADDR_VAR_DELAY_LOOP_US 0x1374
// unsigned short
#define HAL_ADDR_VAR_DELAY_LOOP_MS 0x1378
// 4 byte
#define HAL_ADDR_VAR_DEVICE_SETTINGS 0x137C
// 20byte
#define HAL_ADDR_VAR_DEVICE_POWER_SETTINGS 0x1382
// unsigned char
#define HAL_ADDR_CURRENT_INSTR 0x1396
// unsigned short
#define HAL_ADDR_VAR_ROM_FOR_CPU_READ 0x1398


// const addresses
// short[5]
#define HAL_ADDR_CONST_HAL_INFOS 0x253A
// 512 bytes
#define HAL_ADDR_CONST_HAL_FUNCTION_DEFAULTS 0x2544
// 18 bytes
#define HAL_ADDR_CONST_ETD_COMMON_METHODS 0x2748
