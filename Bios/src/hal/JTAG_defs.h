/*
 * JTAG_defs.h
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

/*========================================================================*\
|                                                                          |
| MSP430.h                                                                 |
|                                                                          |
| This file contains the JTAG instruction and control bit definitions for  |
| the MSP430                                                               |
|--------------------------------------------------------------------------|
| Project:              MSP430 JTAG interfcae                              |
| Developed using:      MS Visual C++ 5.0                                  |
|--------------------------------------------------------------------------|
| Version:              1.2.0                                              |
| Initial Version:      20 / 07 / 02                                       |
| Last Change:          01 / 22 / 04                                       |
|--------------------------------------------------------------------------|
| Version history:                                                         |
| Version: 1.1.5                                                           |
| 12/02/2003 UPSF Added mirrowed JTAG-instr. table for uC execution        |
| 12/02/2003 UPSF Changed JMP_$ to JMP_D                                   |
| 22/02/2003 UPSF Renamed IEx, I2C and ADC10 Register definitions          |
| Version: 1.1.5.8                                                         |
| 16.03.2004 UPSF Changed Register Names to xxx_Address                    |
| Version: 1.2.0.1                                                         |
| 19.08.2004 UPSF Added FCTL1_ADDRESS definition                           |
| 01.10.2004 UPSF Added FCTLx_ADDRESS, LOCKA_BIT definition                |
| Version: 2.1.4.13                                                        |
| 19.08.2004 UPSF Added MOVA_IMM_PC definition                             |
| Version: 2.1.4.20                                                        |
| 02.09.2005 WLUT Added IR_EMEX_DATA_EXCHANGE32 instruction definition     |
|                 Expandedn BPMASK_DONTCARE definition to 32bit            |
| Version: 2.3.4.0                                                         |
| 24.04.2008 WLUT Added definitions for 5xx support                        |
|                                                                          |
|--------------------------------------------------------------------------|
| Designed 2002 by Texas Instruments                                       |
\*========================================================================*/

/*------------------------------------------------------------------------*\
| Remarks:                                                                 |
|                                                                          |
\*------------------------------------------------------------------------*/

#ifndef _JTAG_defs_H_
#define _JTAG_defs_H_

// #defines. ------------------------------------------------------------------
#define FUSECHECK_DELAY         50

#define JTAGVERSION             0x89
#define JTAGVERSION8D           0x8D
#define JTAGVERSION91           0x91
#define JTAGVERSION95           0x95
#define JTAGVERSION99           0x99

#if !uController_uif
// Instructions to access the Emex registers
#define   IR_EMEX_DATA_EXCHANGE   0x09
#define   IR_EMEX_READ_TRIGGER    0x0A
#define   IR_EMEX_READ_CONTROL    0x0B
#define   IR_EMEX_WRITE_CONTROL   0x0C
#define   IR_EMEX_DATA_EXCHANGE32 0x0D

#define IR_TEST_REG               0x2A //Select the 32-bit JTAG test register
#define IR_TEST_3V_REG            0x2F 
// Instructions for the address register
#define IR_ADDR_HIGH_BYTE      0x81
#define IR_ADDR_LOW_BYTE      0x82
#define IR_ADDR_16BIT         0x83
#define IR_ADDR_CAPTURE         0x84
#define IR_DATA_TO_ADDR         0x85
#define IR_CAPTURE_CPU_REG      0x86
#define IR_DEVICE_ID             0x87

// Instructions for the data register
#define IR_DATA_16BIT         0x41
#define IR_DATA_CAPTURE         0x42
#define IR_DATA_QUICK         0x43
#define IR_DATA_PSA               0x44
#define IR_DATA_16BIT_OPT      0x45
#define IR_SHIFT_OUT_PSA      0x46
#define IR_DTA                 0x47

// Instructions for the breakpoint logic
#define IR_BP_CNTL_16BIT      0x09
#define IR_BP_CNTL_CAPTURE      0x0A
#define IR_BP1_16BIT         0x0B
#define IR_BP1_CAPTURE         0x0C
#define IR_BP2_16BIT         0x0D
#define IR_BP2_CAPTURE         0x0E

// Instructions for the FLASH register
#define IR_FLASH_16BIT_UPDATE   0x19
#define IR_FLASH_CAPTURE      0x1A
#define IR_FLASH_16BIT_IN      0x1B
#define IR_FLASH_UPDATE         0x1C
// Bits of the FLASH register
#define FLASH_SESEL1         0x0080
#define FLASH_TMR             0x0800

// Instructions for the control signal register
#define IR_CNTRL_SIG_HIGH_BYTE   0x11
#define IR_CNTRL_SIG_LOW_BYTE   0x12
#define IR_CNTRL_SIG_16BIT      0x13
#define IR_CNTRL_SIG_CAPTURE   0x14
#define IR_CNTRL_SIG_RELEASE   0x15
#define IR_COREIP_ID             0x17
#define IR_JSTATE_ID             0x18

// Bits of the control signal register
#define CNTRL_SIG_READ         0x0001
#define CNTRL_SIG_CPU_HALT      0x0002
#define CNTRL_SIG_INTR_REQ      0x0004
#define CNTRL_SIG_HALT_JTAG      0x0008
#define CNTRL_SIG_BYTE         0x0010
#define CNTRL_SIG_CPU_OFF      0x0020
#define CNTRL_SIG_MCLKON      0x0040
#define CNTRL_SIG_INSTRLOAD      0x0080
#define CNTRL_SIG_TMODE         0x0100
#define CNTRL_SIG_TCE         0x0200
#define CNTRL_SIG_TCE1         0x0400
#define CNTRL_SIG_PUC         0x0800
#define CNTRL_SIG_CPU         0x1000
#define CNTRL_SIG_TAGFUNCSAT   0x2000
#define   CNTRL_SIG_SWITCH      0x4000
#define CNTRL_SIG_STOP_SEL      0x8000
#define CNTRL_SIG_CPUSUSP  (0x0001<<8)
#define CNTRL_SIG_CPUOFF   (0x0001<<5)
#define CNTRL_SIG_INTREQ   (0x0001<<2)
#define CNTRL_SIG_HALT     (0x0001<<1)

// Instructions for the fuse control
#define IR_CNTRL             0x21
#define IR_PREPARE_BLOW         0x22
#define IR_EX_BLOW             0x24

// Instructions for the configuration fuse register
#define IR_CONFIG_FUSES         0x29

// Instructions for the control of the Embedded Signal Processing Cell
#define IR_DUAL_8BIT         0x31
#define IR_DUAL_CAPTURE         0x32
#define IR_SELECT_MAIN         0x33
#define IR_SELECT_ESP         0x34

// Bypass instruction
#define IR_BYPASS             0xFF

// Accept Key Instruction for SPMA devices
#define IR_ACCEPT_KEY         0x59

#define IR_JMB_EXCHANGE       0x61
#define IR_JMB_WRITE_32BIT_MODE     0x88
#define IR_TDO_EVENT          0x64
#define IR_TDO_EVENT_CTL      0x65
#else // uController_uif
// Instructions to access the Emex registers
#define   IR_EMEX_DATA_EXCHANGE   0x90  // 09
#define   IR_EMEX_READ_TRIGGER    0x50  // 0A
#define   IR_EMEX_READ_CONTROL    0xD0  // 0B
#define   IR_EMEX_WRITE_CONTROL   0x30  // 0C
#define   IR_EMEX_DATA_EXCHANGE32 0xB0

#define IR_TEST_REG               0x54  // 2A //Select the 32-bit JTAG test register
#define IR_TEST_3V_REG            0xF4  // 16 bit 3 volt test reg
// Instructions for the address register
#define IR_ADDR_HIGH_BYTE      0x81  // 81
#define IR_ADDR_LOW_BYTE      0x41  // 82
#define IR_ADDR_16BIT         0xC1  // 83
#define IR_ADDR_CAPTURE         0x21  // 84
#define IR_DATA_TO_ADDR         0xA1  // 85
#define IR_CAPTURE_CPU_REG      0x61  // 86
#define IR_DEVICE_ID             0xE1 // 87

// Instructions for the data register
#define IR_DATA_16BIT         0x82  // 41
#define IR_DATA_CAPTURE         0x42  // 42
#define IR_DATA_QUICK         0xC2  // 43
#define IR_DATA_PSA             0x22  // 44
#define IR_DATA_16BIT_OPT      0xA2  // 45
#define IR_SHIFT_OUT_PSA      0x62  // 46
#define IR_DTA                0xE2  // 47

// Instructions for the breakpoint logic
#define IR_BP_CNTL_16BIT      0x90  // 09
#define IR_BP_CNTL_CAPTURE      0x50  // 0A
#define IR_BP1_16BIT         0xD0  // 0B
#define IR_BP1_CAPTURE         0x30  // 0C
#define IR_BP2_16BIT         0xB0  // 0D
#define IR_BP2_CAPTURE         0x70  // 0E

// Instructions for the FLASH register
#define IR_FLASH_16BIT_UPDATE   0x98  // 19
#define IR_FLASH_CAPTURE      0x58  // 1A
#define IR_FLASH_16BIT_IN      0xD8  // 1B
#define IR_FLASH_UPDATE         0x38  // 1C
// Bits of the FLASH register
#define FLASH_SESEL1         0x0080
#define FLASH_TMR             0x0800

// Instructions for the control signal register
#define IR_CNTRL_SIG_HIGH_BYTE   0x88  // 11
#define IR_CNTRL_SIG_LOW_BYTE   0x48  // 12
#define IR_CNTRL_SIG_16BIT      0xC8  // 13
#define IR_CNTRL_SIG_CAPTURE   0x28  // 14
#define IR_CNTRL_SIG_RELEASE   0xA8  // 15
#define IR_COREIP_ID             0xE8 // 17
#define IR_JSTATE_ID             0x46 // 62

// Bits of the control signal register
#define CNTRL_SIG_READ         0x0001
#define CNTRL_SIG_CPU_HALT      0x0002
#define CNTRL_SIG_INTR_REQ      0x0004
#define CNTRL_SIG_HALT_JTAG      0x0008
#define CNTRL_SIG_BYTE         0x0010
#define CNTRL_SIG_CPU_OFF      0x0020
#define CNTRL_SIG_MCLKON      0x0040
#define CNTRL_SIG_INSTRLOAD      0x0080
#define CNTRL_SIG_TMODE         0x0100
#define CNTRL_SIG_TCE         0x0200
#define CNTRL_SIG_TCE1         0x0400
#define CNTRL_SIG_PUC         0x0800
#define CNTRL_SIG_CPU         0x1000
#define CNTRL_SIG_TAGFUNCSAT   0x2000
#define   CNTRL_SIG_SWITCH      0x4000
#define CNTRL_SIG_STOP_SEL      0x8000
#define CNTRL_SIG_CPUSUSP  (0x0001<<8)
#define CNTRL_SIG_CPUOFF   (0x0001<<5)
#define CNTRL_SIG_INTREQ   (0x0001<<2)
#define CNTRL_SIG_HALT     (0x0001<<1)

// Instructions for the fuse control
#define IR_CNTRL             0x84  // 21
#define IR_PREPARE_BLOW         0x44  // 22
#define IR_EX_BLOW             0x24  // 24

// Instructions for the configuration fuse register
#define IR_CONFIG_FUSES         0x94  // 29

// Instructions for the control of the Embedded Signal Processing Cell
#define IR_DUAL_8BIT         0x8C  // 31
#define IR_DUAL_CAPTURE         0x4C  // 32
#define IR_SELECT_MAIN         0xCC  // 33
#define IR_SELECT_ESP         0x2C  // 34

// Bypass instruction
#define IR_BYPASS             0xFF

// Accept Key Instruction for SPMA devices
#define IR_ACCEPT_KEY         0x9A  // 59

#define IR_JMB_EXCHANGE       0x86  // 61
#define IR_JMB_WRITE_32BIT_MODE     0x11
#define IR_TDO_EVENT          0x26  // 64
#define IR_TDO_EVENT_CTL      0xA6  // 65

#endif

#define EEMEV                 0x0100
// Registers in the EMEX logic
#define MX_WRITE         0      // Write offset
#define MX_READ            1      // Read offset

// Breakpoint block
// EMEX address = BP number * Block size + Register offset + R/W offset
// Note: In Volker's new EEM documentation, this block is called the Memory Bus Trigger
#define MX_BLKSIZE             8      // Block size
#define MX_BP                0x0000 // Breakpoint value offset
#define MX_CNTRL             0x0002 // Control offset
#define MX_MASK                0x0004 // Mask offset
#define MX_COMB                0x0006 // Combination offset

// Control block
#define   MX_EEMVER               0x0087
#define   MX_CPUSTOP              0x0080
#define   MX_GENCNTRL             0x0082
#define MX_GCLKCTRL               0x0088
#define MX_MCLKCNTL0              0x008A
#define MX_TRIGFLAG               0x008E

// Settings of the Breakpoint block Control register
#define   BPCNTRL_MAB             0x0000
#define   BPCNTRL_MDB             0x0001
#define   BPCNTRL_RW_DISABLE       0x0000
#define   BPCNTRL_RW_ENABLE       0x0020
#define   BPCNTRL_EQ             0x0000
#define   BPCNTRL_GE             0x0008
#define   BPCNTRL_LE             0x0010
#define   BPCNTRL_FREE          0x0018
#define   BPCNTRL_DMA_DISABLE       0x0000
#define   BPCNTRL_DMA_ENABLE       0x0040
// With BPCNTRL_DMA_DISABLE and BPCNTRL_RW_DISABLE
#define   BPCNTRL_IF             0x0000
#define   BPCNTRL_IFHOLD          0x0002
#define   BPCNTRL_NIF             0x0004
#define   BPCNTRL_BOTH          0x0006
// With BPCNTRL_DMA_DISABLE and BPCNTRL_RW_ENABLE
#define   BPCNTRL_NIF_READ       0x0000
#define   BPCNTRL_NIF_WRITE       0x0002
#define   BPCNTRL_READ          0x0004
#define   BPCNTRL_WRITE          0x0006
// Special consideration for future data breakpoints and the F12x2:
// - Must handle DMA access (Bit 6 of control register)
// - Read/Write Enable (Bit 5 of control register) is not implemented in the F12x2
// With BPCNTRL_DMA_ENABLE and BPCNTRL_RW_DISABLE
#define   BPCNTRL_NIF_NDMA      0x0000
#define   BPCNTRL_DMA             0x0002
#define   BPCNTRL_NDMA         0x0004
#define   BPCNTRL_WRITE_NDMA      0x0006
// With BPCNTRL_DMA_ENABLE and BPCNTRL_RW_ENABLE
#define   BPCNTRL_NIF_READ_NDMA   0x0000
#define   BPCNTRL_READ_NDMA      0x0002
#define   BPCNTRL_READ_DMA      0x0004
#define   BPCNTRL_WRITE_DMA      0x0006

// Settings of the Breakpoint block Mask register
#define   BPMASK_WORD             0x0000
#define BPMASK_HIGHBYTE         0x00FF
#define BPMASK_LOWBYTE         0xFF00
#define BPMASK_DONTCARE         0xFFFFFFFF

// MSP430_Debug.h contains definitions for the following:
//  Bits of the Control block General Control register
//  Bits of the Control block General Clock Control register (F41x)
//  Bits of the Control block General Clock Control register (F43x/F44x)

// Bits of the device Status register (R2)
#define STATUS_REG_C            0x0001
#define STATUS_REG_Z            0x0002
#define STATUS_REG_N            0x0004
#define STATUS_REG_GIE          0x0008
#define STATUS_REG_CPUOFF       0x0010
#define STATUS_REG_OSCOFF       0x0020
#define STATUS_REG_SCG0         0x0040
#define STATUS_REG_SCG1         0x0080
#define STATUS_REG_V            0x0100

#define WDTCTL_ADDRESS          0x120  // Watchdog Timer control register address for 1xx/2xx/4xx family
#define WDTCTL_ADDRESS_5XX      0x15C  // Watchdog Timer control register address for 5xx family
#define WDTPW_DEF               0x5a   // Watchdog Timer password.
#define WDTHOLD_DEF             0x80   // Watchdog Timer hold.
#define WDTSSEL_ACLK            0x03   // Watchdog Timer Clock Source
#define SYSBSLC_ADDRESS         0x182  // SYS Module Bootstrap Loader control register address
#define SYSBSLPE                0x8000 // SYS - BSL Memory protection enalbed

#define FLASHPW_DEF             0xa5   // Flash password.

// MSP430 Instructions.
#define JMP_OPCODE              0x3C00   // JMP OPCODE
#define JMP_D                   0x3fff   // JMP $
#define JMP_D_2                 0x3ffe   // JMP $-2
#define JMP_D_4                 0x3ffd   // JMP $-4
#define BIS_B_IMM_1             0xd3d2   // BIS.B #1,
#define BIC_B_IMM_1             0xc3d2   // BIC.B #1,
#define MOV_IMM_PC              0x4030   // MOV #<val>,PC
#define MOVA_IMM_PC             0x0080   // MOVA #<val>,PC
#define MOVA_IMM_SP             0x0180   // MOVA #<val>,SP
#define MOVA_R14_ABS            0x0E60   // MOVA R14,&abs
#define MOVA_R15_ABS            0x0F60   // MOVA R15,&abs

// MSP430 Address.
#define IE1_ADDRESS             0x0000
#define IE2_ADDRESS             0x0001

#define ADC10DTC0_ADDRESS       0x0048
#define ADC10CTL0_ADDRESS       0x01b0
#define ADC10CTL1_ADDRESS       0x01b2

#define I2CCTL_ADDRESS          0x0070
#define I2CTCTL_ADDRESS         0x0071
#define I2CDR_ADDRESS           0x0076

#define FCTL1_ADDRESS           0x0128
#define FCTL2_ADDRESS           0x012A
#define FCTL3_ADDRESS           0x012C // FCTL3 register address for 1xx/2xx/4xx families
#define LOCKA_BIT               0x0040

#define BYTE_REG_START_ADDR     0x0000
#define WORD_REG_START_ADDR     0x0100
#define LAST_PERIPHERAL_ADDR    0x01ff
//#define DATA_START_ADDR          0x0200
//#define DATA_END_ADDR         0   x09ff
// ROM address (for use in work-around to RAM-corrupted-during-JTAG-access bug).
#define ROM_ADDR              0x0c04
#define FLASH_START_ADDR           0x1000
//#define FLASH_END_ADDR           0xffff

#define MAIN_SEGMENT_SIZE      512      // Segments are normally 512 bytes in size.
#define FIRST_60K_SEGMENT_SIZE  256      // However, the first segment of 60K devices is 256 bytes in size.
//#define INFO_SEGMENT_SIZE      128      // And Information segments are 128 bytes in size.

//JMB DR Requests
#define DR_JMB_PASSWORD_EXCHANGE_REQUEST     0x1E1E  
#define MAGIC_PATTERN                        0xA55A

#endif /* _JTAG_defs_H_ */
