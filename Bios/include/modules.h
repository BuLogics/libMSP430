/**
* \ingroup MODULMACROS
*
* \file modules.h
*
* \brief  ET wrapper module definitions
* This file defines and exports all module definitions
*
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

/**
 @page modules ET wrapper module definitions
 This page introduces the user to the topic.
*/

#ifndef _ETW_MODULES_H_
#define _ETW_MODULES_H_

#define ETKEY			0x9600
#define ETKEYSEL		0x0110
#define ETCLKSEL		0x011E

/*#define ETWPID_EMPTY   0x00
//#define ETWPID_TA2     0x8F
//#define ETWPID_TA3     0x8E
//#define ETWPID_TB7     0x9D
//#define ETWPID_TA5     0x91
#define ETWPID_CRC16   0x09
#define ETWPID_WDT_A   0x0A
#define ETWPID_USB     0x40
#define ETWPID_AES     0x60

#define ETWPID_TA2_0   0x8B
#define ETWPID_TA2_1   0x8C
#define ETWPID_TA2_2   0x8D
#define ETWPID_TA3_0   0x8E
#define ETWPID_TA3_1   0x8F
#define ETWPID_TA3_2   0x90
#define ETWPID_TA5_0   0x91
#define ETWPID_TA5_1   0x92
#define ETWPID_TA5_2   0x93
#define ETWPID_TA7_0   0x94
#define ETWPID_TA7_1   0x95
#define ETWPID_TA7_2   0x96

#define ETWPID_TD3_0   0x74
#define ETWPID_TD3_1   0x75
#define ETWPID_TD3_2   0x76
#define ETWPID_TD3_3   0x77

#define ETWPID_TB3_0   0x97
#define ETWPID_TB3_1   0x98
#define ETWPID_TB3_2   0x99
#define ETWPID_TB5_0   0x9A
#define ETWPID_TB5_1   0x9B
#define ETWPID_TB5_2   0x9C
#define ETWPID_TB7_0   0x9D
#define ETWPID_TB7_1   0x9E
#define ETWPID_TB7_2   0x9F

#define ETWPID_USCI0   0x28
#define ETWPID_USCI1   0x29
#define ETWPID_USCI2   0x2A
#define ETWPID_USCI3   0x2B
#define ETWPID_eUSCIA0 0x2C
#define ETWPID_eUSCIA1 0x2D
#define ETWPID_eUSCIB0 0x30
#define ETWPID_RTC     0x8A
#define ETWPID_BTRTC   0x8A

#define ETWPID_COMP_B  0xA8
#define ETWPID_LCD_B   0xB0

#define ETWPID_APOOL   0xB5

#define ETWPID_RF1A    0xBC
#define ETWPID_RF1B    0xBD
#define ETWPID_RF2A    0xBE
#define ETWPID_RF2B    0xBF

#define ETWPID_DAC12_0 0xC0
#define ETWPID_DAC12_1 0xC1
#define ETWPID_SD16A_4 0xD4
#define ETWPID_ADC10_A 0xD6
#define ETWPID_ADC12_A 0xD8*/

/*#define ETWPID_WDT_A	0x000A
#define ETWPID_TA3_1	0x008F
#define ETWPID_TB7_0	0x009D
#define ETWPID_TA5_0	0x0091
#define ETWPID_USCI0	0x0028
#define ETWPID_USCI1	0x0029
#define ETWPID_USCI2	0x002A
#define ETWPID_USCI3	0x002B
#define ETWPID_RTC		0x0080
#define ETWPID_ADC12_A	0x00D8*/


/*#define ETWNAME_WDT_A	"Watchdog Timer"
#define ETWNAME_TA3_1	"Timer1_A3"
#define ETWNAME_TB7_0	"Timer0_B7"
#define ETWNAME_TA5_0	"Timer0_A5"
#define ETWNAME_USCI0	"USCI0"
#define ETWNAME_USCI1	"USCI1"
#define ETWNAME_USCI2	"USCI2"
#define ETWNAME_USCI3	"USCI3"
#define ETWNAME_RTC		"RTC"
#define ETWNAME_ADC12_A	"ADC12A"*/

#endif
