/*
 * ConfigureParameters.h
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
 @file ConfigureParameters.h
 @brief Contains configure flags for setting up the HAL

 This file defines all the flags that are used for setting up the HIL, specifically where
 special handling is required.
*/

#ifndef _CONFIGPARAMETERS_H_
#define _CONFIGPARAMETERS_H_

#define CONFIG_PARAM_ENHANCED_PSA               0x0001
#define CONFIG_PARAM_PSA_TCKL_HIGH              0x0002
#define CONFIG_PARAM_DEFAULT_CLK_CONTROL        0x0003

// Power settings for test_reg
#define CONFIG_PARAM_POWER_TESTREG_MASK         0x0004
#define CONFIG_PARAM_TESTREG_ENABLE_LPMX5       0x0005
#define CONFIG_PARAM_TESTREG_DISABLE_LPMX5      0x0006

// Power settings for test_reg_3V
#define CONFIG_PARAM_POWER_TESTREG3V_MASK       0x0007
#define CONFIG_PARAM_TESTREG3V_ENABLE_LPMX5     0x0008
#define CONFIG_PARAM_TESTREG3V_DISABLE_LPMX5    0x0009

#define CONFIG_PARAM_CLK_CONTROL_TYPE           0x000A

#define CONFIG_PARAM_JTAG_SPEED                 0x000B

#define CONFIG_PARAM_SFLLDEH					0x000C

#define CONFIG_NO_BSL                           0x000D

#define CONFIG_ALT_ROM_ADDR_FOR_CPU_READ        0x000E

#endif

