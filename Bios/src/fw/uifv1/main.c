/**
* \ingroup MODULBIOS
*
* \file main.c
*
* \brief This file is the firmware start point of the USB Flash Emulation Tool.
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

//! \defgroup MODULBIOS BIOS (modul)
//! \defgroup MODULHAL HAL (modul)
//! \defgroup MODULMACROS HAL-macros (modul)
//! \defgroup MODULMACROSX HAL-macros X (modul)
//! \defgroup MODULMACROSXV2 HAL-macros Xv2 (modul)
//! \defgroup MODULHIL HIL-macros (modul)

//! \mainpage Firmware MSP430-UIF (for MSP430.DLL V3)
//! This project include the code for MSP-FET430UIF for using with MSP430_DLLv3. The firmware is splitted in three parts:
//! \li safecore -> \ref safecore_page1
//! \li bios (core) -> \ref bios_page1
//! \li HAL -> \ref hal_page1
//! \li HIL -> \ref hil_page1
//!  
//!  two additional parts:
//!  \li uif update -> \ref update_page1
//!  \li uif downgrade -> \ref downgrate_page1
//!  
//!  and for internal use:
//!  \li uif eeprom update -> \ref eepromupdate_page1

//! \page bios_page1 BIOS (CORE)
//! \brief This file is the firmware start point of the USB Flash Emulation Tool
//! \author  Detlef Fink (09/21/2010)
//!
//! <b>compile and link</b>\n
//! The bios use only 3,5k memory (7 segments in MSP430). It must be linked with a adapted linker file (Bios_lnk430F1612.xcl).\n
//! For core-updates exclude safecore.s43 from linking.
//!
//! <b>files</b>\n
//! \li safecore.s43
//! \li main.c
//! \li bios.c
//! \li v3_0p.c
//! \li v3_0p_hw_uif.c
//! \li stream.c



#include "bios.h"
#include "../v3_0p.h"

void main ( void );

//! main function, starts the UIF
void main ( void )
{
    biosSetWdt(0);
    biosInitSystem();
    biosInitCom(BAUDRATE);
    biosHalInterfaceInit();
    biosLedOff(BIOS_LED_MODE);
    biosLedOn(BIOS_LED_POWER);
    biosmainLoop();
}


