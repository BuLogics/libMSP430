/**
* \ingroup MODULHAL
*
* \file edt.h
*
* \brief <FILEBRIEF>
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
 @page edt Enhanced Device Test layer (EDT)
 The EDT layer is sort of abstraction of the target devices' JTAG interface.
 Beside some initialization and configuration functions it provides both
 high level functions to access the JTAG Instruction/Data registers plus
 a set of low level symbols to manipulate individual signals of the JTAG
 interface. The EDT layer is described in detail via @ref edt.h.

 Proceed to the @ref hal.  
*/

/**
 @file edt.h
 @brief Enhanced Device Test layer (EDT)

 This file defines and exports all the objects and symbols required to use
 the Enhanced Device Test layer (EDT).

 All access to EDT high level functions and low level symbols is realized
 through function/variable pointers. This enables straight forward update
 strategies as none of the resources are accessed via absolute addresses.
 Refer to @ref update for more details on this topic.
*/
#ifndef _EDT_H_
#define _EDT_H_
#include "JTAG_defs.h"

/**
 @brief Structure of pointers to all exported EDT functions.
*/
struct edt_common_methods {
  short (*Init)(void);
  short (*SetVcc)(unsigned short);
  short (*GetVcc)(unsigned short*, unsigned short*);
  short (*SetProtocol)(unsigned short);
  void  (*SetPsaSetup)(unsigned short);
  void  (*SetPsaTCLK)(unsigned short);
  short (*Open)(unsigned char state);
  short (*Close)(void);
  void  (*Delay_1us)(unsigned short);
  void  (*Delay_1ms)(unsigned short);
  short (*Loop)(unsigned short);
  void  (*EntrySequences)(unsigned char);

  void (*SetJtagBits)(unsigned char);   // Set discreet JTAG bits according to the input
  unsigned char (*GetJtagBits)(void);   // Read the current output of the JTAG port

  void (*SetTgtCtrlBits)(unsigned char);// Set discreet Target Control bits according to the input
  unsigned char (*GetTgtCtrlBits)(void);// Read the current output of the TargetControl port

  void (*SetTestBits)(unsigned char);   // Set discreet Test Control bits according to the input
  unsigned char (*GetTestBits)(void);   // Read the current output of the Test Control port
  
  void (*SetReset)(unsigned char);      // Set the Reset pin to the specified value
  void (*SetTest)(unsigned char);       // Set the Test pin to the specified value
  void (*SetJtagSpeed)(unsigned short);
};
typedef struct edt_common_methods edt_common_methods_t;

struct edt_distinct_methods 
{
    short (*TapReset)(void);
    short (*CheckJtagFuse)(void);
    unsigned short (*EnumChain)(void);
    unsigned char (*Instr)(unsigned char);
    unsigned char (*SetReg_XBits08)(unsigned char);
    unsigned short (*SetReg_XBits16)(unsigned short);
    unsigned long (*SetReg_XBits20)(unsigned long);
    unsigned long (*SetReg_XBits32)(unsigned long);
    unsigned long long (*SetReg_XBits64)(unsigned long long);
    void (*Tclk)(unsigned char);
    void (*SetupPsa)(unsigned long);
    void (*StepPsa)(unsigned long);
    void (*EndPsa)(void);
    short (*BlowFuse)(unsigned char); // Blow the JTAG acces fuse
};
typedef struct edt_distinct_methods edt_distinct_methods_t;

/**
 @brief Constant structure holding all EDT function pointers.

 One constant structure of type \a edt_common_methods. It holds all EDT
 function pointers that are independant from the low level JTAG communication
 protocol (4-wire JTAG or Spy-Bi-Wire).
 Another variable structure of type \a edt_distinct_methods. It holds all EDT
 function pointers that are remapped depending on the low level JTAG prtocol
 selection (4-wire JTAG or Spy-Bi-Wire).

 @warning It is not recommended to use symbols of this structure in other
          modules direclty! Use the hash define symbols instead.
*/
extern const edt_common_methods_t _edt_Common_Methods;
extern edt_distinct_methods_t _edt_Distinct_Methods;
extern unsigned short setPCclockBeforeCapture;

#define EDT_Init           (*_edt_Common_Methods.Init)
#define EDT_SetJtagSpeed   (*_edt_Common_Methods.SetJtagSpeed)
#define EDT_SetVcc         (*_edt_Common_Methods.SetVcc)
#define EDT_GetVcc         (*_edt_Common_Methods.GetVcc)
#define EDT_SetProtocol    (*_edt_Common_Methods.SetProtocol)
#define EDT_SetPsaSetup    (*_edt_Common_Methods.SetPsaSetup)
#define EDT_SetPsaTCLK     (*_edt_Common_Methods.SetPsaTCLK)
#define EDT_Open           (*_edt_Common_Methods.Open)
#define EDT_Close          (*_edt_Common_Methods.Close)
#define EDT_Delay_1us      (*_edt_Common_Methods.Delay_1us)
#define EDT_Delay_1ms      (*_edt_Common_Methods.Delay_1ms)
#define EDT_Loop           (*_edt_Common_Methods.Loop)
#define EDT_EntrySequences (*_edt_Common_Methods.EntrySequences)
#define EDT_SetJtagBits    (*_edt_Common_Methods.SetJtagBits)
#define EDT_GetJtagBits    (*_edt_Common_Methods.GetJtagBits)
#define EDT_SetTgtCtrlBits (*_edt_Common_Methods.SetTgtCtrlBits)
#define EDT_GetTgtCtrlBits (*_edt_Common_Methods.GetTgtCtrlBits)
#define EDT_SetTestBits (*_edt_Common_Methods.SetTestBits)
#define EDT_GetTestBits (*_edt_Common_Methods.GetTestBits)

#define EDT_SetReset    (*_edt_Common_Methods.SetReset)
#define EDT_SetTest     (*_edt_Common_Methods.SetTest)

#define EDT_TapReset       (*_edt_Distinct_Methods.TapReset)
#define EDT_CheckJtagFuse  (*_edt_Distinct_Methods.CheckJtagFuse)
#define EDT_EnumChain      (*_edt_Distinct_Methods.EnumChain)
#define EDT_Instr          (*_edt_Distinct_Methods.Instr)
#define EDT_SetReg_XBits   (*_edt_Distinct_Methods.SetReg_XBits)
#define EDT_SetReg_XBits08 (*_edt_Distinct_Methods.SetReg_XBits08)
#define EDT_SetReg_XBits16 (*_edt_Distinct_Methods.SetReg_XBits16)
#define EDT_SetReg_XBits20 (*_edt_Distinct_Methods.SetReg_XBits20)
#define EDT_SetReg_XBits32 (*_edt_Distinct_Methods.SetReg_XBits32)
#define EDT_SetReg_XBits64 (*_edt_Distinct_Methods.SetReg_XBits64)
#define EDT_Tclk           (*_edt_Distinct_Methods.Tclk)
#define EDT_SetupPsa       (*_edt_Distinct_Methods.SetupPsa)
#define EDT_StepPsa        (*_edt_Distinct_Methods.StepPsa)
#define EDT_EndPsa         (*_edt_Distinct_Methods.EndPsa)
#define EDT_BlowFuse       (*_edt_Distinct_Methods.BlowFuse)

// further definitions and macros

#define decl_io             short lIn; short lOut;
#define decl_in             short lIn;
#define decl_out            short lOut  = 0;
#define decl_out_long       unsigned long lOut_long  = 0;
#define decl_out_long_long  unsigned long long lOut_long_long = 0;
#define decl_state          short lPass = 0; short lFail = 0;
#define decl_pass           short lPass = 0;
#define decl_fail           short lFail = 0;
#define decl_isInstrLoad    short lIsInstrLoad = -1;
#define decl_instrLoad      short lInstrLoad = -1;
#define decl_jtagMailboxIn  short jtagMailboxIn = -1;
#define decl_jtagMailboxOut short jtagMailboxOu = -1;

// JTAG instruction register access
#define bypass              { lOut = EDT_Instr(0xFF); } 
#define cntrl_sig_high_byte { EDT_Instr(IR_CNTRL_SIG_HIGH_BYTE); }
#define cntrl_sig_low_byte  { EDT_Instr(IR_CNTRL_SIG_LOW_BYTE); }
#define cntrl_sig_capture   { lOut = EDT_Instr(IR_CNTRL_SIG_CAPTURE); }
#define cntrl_sig_16bit     { EDT_Instr(IR_CNTRL_SIG_16BIT); }
#define cntrl_sig_release   { EDT_Instr(IR_CNTRL_SIG_RELEASE); }
#define addr_16bit          { EDT_Instr(IR_ADDR_16BIT); }
#define addr_capture        { EDT_Instr(IR_ADDR_CAPTURE); }
#define data_16bit          { EDT_Instr(IR_DATA_16BIT); }
#define data_capture        { EDT_Instr(IR_DATA_CAPTURE); }
#define data_to_addr        { EDT_Instr(IR_DATA_TO_ADDR); }
#define data_quick          { EDT_Instr(IR_DATA_QUICK); }
#define config_fuses        { EDT_Instr(IR_CONFIG_FUSES); }
#define eem_data_exchange   { EDT_Instr(IR_EMEX_DATA_EXCHANGE); }
#define eem_data_exchange32 { EDT_Instr(IR_EMEX_DATA_EXCHANGE32); }
#define eem_read_control    { EDT_Instr(IR_EMEX_READ_CONTROL); }
#define eem_write_control   { EDT_Instr(IR_EMEX_WRITE_CONTROL); }
#define eem_data_exchange32 { EDT_Instr(IR_EMEX_DATA_EXCHANGE32); }
#define eem_read_trigger    { EDT_Instr(IR_EMEX_READ_TRIGGER); }
#define data_psa            { EDT_Instr(IR_DATA_PSA); }
#define shift_out_psa       { EDT_Instr(IR_SHIFT_OUT_PSA); }
#define flash_16bit_update  { EDT_Instr(IR_FLASH_16BIT_UPDATE); }
#define tclk_0_inst         { EDT_Instr(0x3A); }//3A	
#define tclk_1_inst         { EDT_Instr(0x3B); }//3B
#define tclk_toggle_inst    { EDT_Instr(0x39); }//39 
#define jmb_exchange        { EDT_Instr(IR_JMB_EXCHANGE); }
#define device_ip_pointer   { EDT_Instr(IR_DEVICE_ID);}
#define core_ip_pointer     { EDT_Instr(IR_COREIP_ID);}
#define jstate_read         { EDT_Instr(IR_JSTATE_ID);}
#define test_reg            { EDT_Instr(IR_TEST_REG);}
#define test_reg_3V         { EDT_Instr(IR_TEST_3V_REG);}


// JTAG data register access
#define SetReg_8Bits(x)   { lOut      = (short)EDT_SetReg_XBits08(x); }
#define SetReg_8Bits_(x)  {             EDT_SetReg_XBits08(x); }
#define SetReg_16Bits(x)  { lOut      = (short)EDT_SetReg_XBits16(x); }
#define SetReg_16Bits_(x) {             EDT_SetReg_XBits16(x); }
#define SetReg_20Bits(x)  { lOut_long = EDT_SetReg_XBits20(x); }
#define SetReg_20Bits_(x) {             EDT_SetReg_XBits20(x); }
#define SetReg_32Bits(x)  { lOut_long = EDT_SetReg_XBits32(x); }
#define SetReg_32Bits_(x) {             EDT_SetReg_XBits32(x); }
#define SetReg_64Bits(x)  { lOut_long_long = EDT_SetReg_XBits64(x); }
#define SetReg_64Bits_(x) {             EDT_SetReg_XBits64(x); }


// JTAG macros (used for code inlining)
#define isInstrLoad      { cntrl_sig_capture                                        \
                           lIsInstrLoad = 0;                                        \
                           SetReg_16Bits(0);                                        \
                           if((lOut & (CNTRL_SIG_INSTRLOAD | CNTRL_SIG_READ)) != \
                              (CNTRL_SIG_INSTRLOAD | CNTRL_SIG_READ))               \
                           {                                                        \
                               lIsInstrLoad = -1;                                   \
                           } }

#define instrLoad        { unsigned short i;             \
                           decl_isInstrLoad              \
                           lInstrLoad = 0;               \
                           cntrl_sig_low_byte            \
                           SetReg_8Bits_(CNTRL_SIG_READ) \
                           TCLKset1                      \
                           for(i = 0; i < 10; i++)       \
                           {                             \
                               isInstrLoad               \
                               if(lIsInstrLoad == 0)     \
                               {                         \
                                   break;                \
                               }                         \
                               TCLK                      \
                           }                             \
                           if(i == 10)                   \
                           {                             \
                               --lInstrLoad;             \
                           } }

#define halt_cpu          { data_16bit             \
                            SetReg_16Bits_(0x3FFF) \
                            TCLKset0               \
                            cntrl_sig_16bit        \
                            SetReg_16Bits_(0x2409) \
                            TCLKset1 }

#define release_cpu       { TCLKset0               \
                            cntrl_sig_16bit        \
                            SetReg_16Bits_(0x2401) \
                            addr_capture           \
                            TCLKset1 }
#define ReadMemWord(a,d)  { halt_cpu               \
                            TCLKset0               \
                            addr_16bit             \
                            SetReg_16Bits(a)       \
                            data_to_addr           \
                            TCLKset1               \
                            TCLKset0               \
                            SetReg_16Bits(0)       \
                            d = lOut;              \
                            release_cpu }

#define ReadMemWordX(a,d) { halt_cpu               \
                            TCLKset0               \
                            addr_16bit             \
                            SetReg_20Bits(a)       \
                            data_to_addr           \
                            TCLKset1               \
                            TCLKset0               \
                            SetReg_16Bits(0)       \
                            d = lOut;              \
                            release_cpu }

#define ReadMemWordXv2(a,d){decl_out               \
                            TCLKset0               \
                            addr_16bit             \
                            SetReg_20Bits(a)       \
                            TCLKset1               \
                            TCLKset0               \
                            data_capture           \
                            SetReg_16Bits(0);      \
                            d = lOut;              \
                            TCLKset1               \
                            TCLKset0               \
                            TCLKset1 }        
          
#define WriteMemWord(a,v) { halt_cpu               \
                            TCLKset0               \
                            cntrl_sig_low_byte     \
                            SetReg_8Bits_(0x08)    \
                            addr_16bit             \
                            SetReg_16Bits_(a)      \
                            data_to_addr           \
                            SetReg_16Bits_(v)      \
                            TCLKset1               \
                            release_cpu } 

#define WriteMemWordX(a,v) {halt_cpu               \
                            TCLKset0               \
                            cntrl_sig_low_byte     \
                            SetReg_8Bits_(0x08)    \
                            addr_16bit             \
                            SetReg_20Bits_(a)      \
                            data_to_addr           \
                            SetReg_16Bits_(v)      \
                            TCLKset1               \
                            release_cpu }

#define WriteMemWordXv2(a,v){TCLKset0              \
                            cntrl_sig_16bit        \
                            SetReg_16Bits_(0x0500) \
                            addr_16bit             \
                            SetReg_20Bits_(a)      \
                            TCLKset1               \
                            data_to_addr           \
                            SetReg_16Bits_(v)      \
                            TCLKset0               \
                            cntrl_sig_16bit        \
                            SetReg_16Bits_(0x0501) \
                            TCLKset1               \
                            TCLKset0               \
                            TCLKset1    }

#define WriteMemByte(a,v){halt_cpu                              \
                            TCLKset0                            \
                            addr_16bit                          \
                            SetReg_16Bits_(a)                   \
                            data_to_addr                        \
                            SetReg_8Bits_(v)                    \
                            TCLKset1                            \
                            TCLKset0                            \
                            release_cpu }


#define ReadCpuReg(n,d)   { short op;              \
                            cntrl_sig_16bit        \
                            SetReg_16Bits(0x3401)  \
                            data_16bit             \
                            op = ((n << 8) & 0x0F00) | 0x4082; \
                            SetReg_16Bits(op)      \
                            TCLKset0               \
                            data_capture           \
                            TCLKset1               \
                            data_16bit             \
                            SetReg_16Bits(0x00fe)  \
                            TCLKset0               \
                            data_capture           \
                            TCLKset1               \
                            TCLKset0               \
                            TCLKset1               \
                            SetReg_16Bits(0)       \
                            d = lOut;              \
                            TCLKset0               \
                            cntrl_sig_16bit        \
                            SetReg_16Bits(0x2401)  \
                            TCLKset1 }


#define ReadCpuReg_uShort(n,d)   { short op;       \
                            cntrl_sig_16bit        \
                            SetReg_16Bits(0x3401)  \
                            data_16bit             \
                            op = ((n << 8) & 0x0F00) | 0x4082; \
                            SetReg_16Bits(op)      \
                            TCLKset0               \
                            data_capture           \
                            TCLKset1               \
                            data_16bit             \
                            SetReg_16Bits(0x00fe)  \
                            TCLKset0               \
                            data_capture           \
                            TCLKset1               \
                            TCLKset0               \
                            TCLKset1               \
                            SetReg_16Bits(0)       \
                            d = (unsigned short)lOut;              \
                            TCLKset0               \
                            cntrl_sig_16bit        \
                            SetReg_16Bits(0x2401)  \
                            TCLKset1 }

#define ReadCpuRegX(n,d)  { short op;              \
                            unsigned short Rx_l;   \
                            unsigned short Rx_h;   \
                            cntrl_sig_high_byte    \
                            SetReg_16Bits(0x34)    \
                            op = ((n << 8) & 0x0F00) | 0x60; \
                            data_16bit             \
                            TCLKset1               \
                            SetReg_16Bits(op)      \
                            TCLKset0               \
                            data_capture           \
                            TCLKset1               \
                            data_16bit             \
                            TCLKset1               \
                            SetReg_16Bits(0x00fc)  \
                            TCLKset0               \
                            data_capture           \
                            TCLKset1               \
                            SetReg_16Bits(0)       \
                            Rx_l = lOut;           \
                            TCLKset0               \
                            TCLKset1               \
                            SetReg_16Bits(0)       \
                            Rx_h = lOut;           \
                            TCLKset0               \
                            cntrl_sig_high_byte    \
                            SetReg_8Bits(0x24)     \
                            TCLKset1               \
                            d = ((unsigned long)Rx_h<<16) + Rx_l; }


#define ReadCpuRegXv2(n,d){ unsigned short Rx_l;   \
                            unsigned short Rx_h;   \
                            TCLKset0               \
                            data_16bit             \
                            TCLKset1               \
                            SetReg_16Bits(n)       \
                            cntrl_sig_16bit        \
                            SetReg_16Bits(0x1401)  \
                            data_16bit             \
                            TCLK                   \
                            if (altRomAddressForCpuRead)\
                            {                      \
                                SetReg_16Bits(0x0ff6) \
                            }                      \
                            else                   \
                            {                      \
                                SetReg_16Bits(0x018c) \
                            }                      \
                            TCLK                   \
                            SetReg_16Bits(0x3ffd)  \
                            TCLKset0               \
                            if (altRomAddressForCpuRead)\
                            {                      \
                                cntrl_sig_16bit    \
                                SetReg_16Bits(0x0501)  \
                            }                      \
                            data_capture           \
                            TCLKset1               \
                            SetReg_16Bits(0)       \
                            Rx_l = lOut;           \
                            TCLK                   \
                            SetReg_16Bits(0)       \
                            Rx_h = lOut;           \
                            TCLK                   \
                            TCLK                   \
                            TCLK                   \
                            if (!altRomAddressForCpuRead)\
                            {                      \
                                cntrl_sig_16bit    \
                                SetReg_16Bits(0x0501)  \
                            }                      \
                            TCLKset0               \
                            data_capture           \
                            TCLKset1               \
                            d = ((unsigned long)Rx_h<<16) + Rx_l; }


#define WriteCpuReg(n,s)   { short op;             \
                            cntrl_sig_16bit        \
                            SetReg_16Bits_(0x3401) \
                            data_16bit             \
                            op = (0x4030 | n);     \
                            SetReg_16Bits_(op)     \
                            TCLKset0               \
                            data_capture           \
                            TCLKset1               \
                            data_16bit             \
                            SetReg_16Bits_(s)      \
                            TCLKset0               \
                            data_capture           \
                            TCLKset1               \
                            data_16bit             \
                            SetReg_16Bits_(0x3ffd) \
                            TCLKset0               \
                            data_capture           \
                            TCLKset1               \
                            TCLKset0               \
                            cntrl_sig_16bit        \
                            SetReg_16Bits_(0x2401) \
                            TCLKset1 }

#define WriteCpuRegX(n,s)  { short op = 0x0080 | n | ((s>>8) & 0x0F00); \
                            cntrl_sig_high_byte    \
                            SetReg_8Bits_(0x34)    \
                            data_16bit             \
                            TCLKset1               \
                            SetReg_16Bits_(op)     \
                            TCLKset0               \
                            data_capture           \
                            TCLKset1               \
                            data_16bit             \
                            TCLKset1               \
                            SetReg_16Bits_(s&0xFFFF)\
                            TCLKset0               \
                            data_capture           \
                            TCLKset1               \
                            data_16bit             \
                            TCLKset1               \
                            SetReg_16Bits_(0x3ffd) \
                            TCLKset0               \
                            data_capture           \
                            TCLKset1               \
                            TCLKset0               \
                            cntrl_sig_high_byte    \
                            SetReg_8Bits_(0x24)    \
                            TCLKset1 }

/* !BUGFIX BTT1722 removed 
#define WriteCpuRegXv2(n,s){TCLKset0                \
                            data_16bit              \
                            TCLKset1                \
                            SetReg_16Bits_(n)       \
                            cntrl_sig_16bit         \
                            SetReg_16Bits_(0x1401)  \
                            data_16bit              \
                            TCLK                    \
                            SetReg_16Bits_(s)       \
                            TCLK                    \
                            SetReg_16Bits_(0x3ffd)  \
                            TCLK                    \
                            TCLKset0                \
                            addr_capture            \
                            SetReg_20Bits(0x00000)  \
                            TCLKset1                \
                            TCLK                    \
                            cntrl_sig_16bit         \
                            SetReg_16Bits_(0x0501)  \
                            TCLKset0                \
                            data_capture            \
                            TCLKset1 }
// !BUGFIX BTT1722 removed  */
// BUGFIX BTT1722 added 
#define WriteCpuRegXv2(n,s){TCLKset0                \
                            data_16bit              \
                            TCLKset1                \
                            SetReg_16Bits_(n)       \
                            cntrl_sig_16bit         \
                            SetReg_16Bits_(0x1401)  \
                            data_16bit              \
                            TCLK                    \
                            SetReg_16Bits_(s)       \
                            TCLK                    \
                            SetReg_16Bits_(0x3ffd)  \
                            TCLK                    \
                            TCLKset0                \
                            addr_capture            \
                            SetReg_20Bits(0x00000)  \
                            TCLKset1                \
                            cntrl_sig_16bit         \
                            SetReg_16Bits_(0x0501)  \
                            TCLK                    \
                            TCLKset0                \
                            data_capture            \
                            TCLKset1 }
// !BUGFIX BTT1722


#define isInstrFetch      { cntrl_sig_capture      \
                            SetReg_16Bits(0)       \
                            if(lOut & 0x0080)      \
                            {                      \
                              lPass++;             \
                            }                      \
                            else                   \
                            {                      \
                              lFail++;             \
                            }                      \
                          }

//    cycle(MOV_IMM_PC); // MOV #<pc>, PC 4030
//    cycle((WORD)pc);
//    if (cycleEnd() != STATUS_OK)
/*   cntrlSigHighByte(CNTRL_SIG_TCE1 | CNTRL_SIG_CPU); // Enable CPU control.

 HIL_JTAG_IR_(IR_DATA_16BIT);
   HIL_TCLK(1);                   // added for 2xx support
   HIL_JTAG_DR16(data);
   HIL_TCLK(0);
   HIL_JTAG_IR_(IR_DATA_CAPTURE);
   HIL_TCLK(1); */
/*
#define SetPc(x)          { cntrl_sig_16bit        \
                            SetReg_16Bits_(0x3401) \
                            data_16bit             \
                            TCLKset1               \
                            SetReg_16Bits_(0x4030) \
                            TCLKset0               \
                            data_capture           \
                            TCLKset1               \
                            data_16bit             \
                            TCLKset1               \
                            SetReg_16Bits_(x)      \
                            TCLKset0               \
                            data_capture           \
                            TCLKset1               \
                            TCLKset0               \
                            cntrl_sig_16bit        \
                              SetReg_16Bits_(0x2401) \
                            TCLKset1               \
                           }                          

*/
#define SetPc(x)          { cntrl_sig_16bit        \
                            SetReg_16Bits_(0x3401) \
                            data_16bit             \
                            SetReg_16Bits_(0x4030) \
                            TCLKset0               \
                            TCLKset1               \
                            SetReg_16Bits_(x)      \
                            if (setPCclockBeforeCapture) {TCLK}    \
                            TCLKset0               \
                            addr_capture           \
                            if (!setPCclockBeforeCapture) {TCLKset1}   \
                            TCLKset0               \
                            cntrl_sig_16bit        \
                            SetReg_16Bits_(0x2401) \
                            TCLKset1               \
                          }
/*
#define SetPc(x)          { cntrl_sig_16bit        \
                            SetReg_16Bits_(0x3401) \
                            data_16bit             \
                            SetReg_16Bits_(0x4030) \
                            TCLKset0              \
                            TCLKset1               \
                            SetReg_16Bits_(x)      \
                            TCLKset0               \
                            addr_capture           \
                            TCLKset1               \
                            TCLKset0               \
                            cntrl_sig_16bit        \
                            SetReg_16Bits_(0x2401) \
                            TCLKset1               \
                          }
*/
#define SetPcJtagBug(x)   {                         \
                            data_16bit              \
                            SetReg_16Bits_(0x4030)  \
                            TCLKset1                \
                            TCLKset0                \
                            SetReg_16Bits_(x)       \
                            TCLKset1                \
                            TCLKset0                \
                            TCLKset1                \
                            TCLKset0                \
                        }

#define SetPcX(x)       {   unsigned short pc_high = (unsigned short)(0x80 | (((x)>>8) & 0x0F00));\
                            unsigned short pc_low  = (unsigned short)((x) & 0xFFFF); \
                            cntrl_sig_high_byte    \
                            SetReg_8Bits_(0x34)    \
                            data_16bit             \
                            SetReg_16Bits_(pc_high)\
                            TCLKset0               \
                            TCLKset1               \
                            SetReg_16Bits_(pc_low) \
                            TCLKset0               \
                            addr_capture           \
                            TCLKset1               \
                            TCLKset0               \
                            cntrl_sig_high_byte    \
                            SetReg_8Bits_(0x24)     \
                            TCLKset1               \
                        }

#define SetPcXv2(Mova,X) {  cntrl_sig_capture      \
                            SetReg_16Bits_(0x0000) \
                            TCLKset0;              \
                            data_16bit             \
                            TCLKset1               \
                            SetReg_16Bits(Mova)    \
                            TCLKset0               \
                            cntrl_sig_16bit        \
                            SetReg_16Bits(0x1400)  \
                            data_16bit             \
                            TCLKset0               \
                            TCLKset1               \
                            SetReg_16Bits(X)       \
                            TCLKset0               \
                            TCLKset1               \
                            SetReg_16Bits(0x4303)  \
                            TCLKset0               \
                            addr_capture           \
                            SetReg_20Bits_(0x00000)}

#define SyncJtag()        { cntrl_sig_16bit                    \
                            SetReg_16Bits(0x2401)              \
                            cntrl_sig_capture                  \
                            i = 50;                            \
                            do                                 \
                            {                                  \
                              SetReg_16Bits(0x0000)            \
                              i--;                             \
                            }                                  \
                            /* \bugfix Günther - if lOut = 0xFFFF then timeout will never occur \
                              causing the firmware to hang indefinately (as well as the DLL). \
                              Added extra braces so that the timeout is honored */\
                            while(((lOut == 0xFFFF) || !(lOut & 0x0200)) && i);  \
                          }

#define SyncJtagX()        { cntrl_sig_16bit                    \
                            SetReg_16Bits(0x2401)              \
                            cntrl_sig_capture                  \
                            SetReg_16Bits(0x0000)              \
                            TCLKset1                           \
                            if (!(lOut & 0x0200))              \
                            {                                  \
                                cntrl_sig_high_byte            \
                                SetReg_8Bits(0x34)             \
                                eem_data_exchange32            \
                                SetReg_32Bits_(0x89)           \
                                SetReg_32Bits(0)               \
                                eem_data_exchange32            \
                                SetReg_32Bits_(0x88)           \
                                SetReg_32Bits(lOut|0x40)       \
                                eem_data_exchange32            \
                                SetReg_32Bits_(0x88)           \
                                SetReg_32Bits(lOut)            \
                            }                                  \
                            cntrl_sig_capture                  \
                            i = 50;                            \
                            do                                 \
                            {                                  \
                              SetReg_16Bits(0x0000)            \
                              i--;                             \
                            }                                  \
                            while(((lOut == 0xFFFF) || !(lOut & 0x0200)) && i);\
                          }

#define SyncJtagXv2()     { cntrl_sig_16bit                    \
                            SetReg_16Bits(0x1501)              \
                            cntrl_sig_capture                  \
                            i = 50;                            \
                            do                                 \
                            {                                  \
                              SetReg_16Bits(0x0000)            \
                              i--;                             \
                            }                                  \
                            while(((lOut == 0xFFFF) || !(lOut & 0x0200)) && i);  \
                          }

#define Shift_Bits(NumOfBits,TdiVal){\
                            unsigned short tempVar;           \
                            if(TdiVal)                        \
                              TDIset1;                        \
                            else                              \
                              TDIset0;                        \
                            for(tempVar = NumOfBits;tempVar > 0; tempVar--)\
                            {\
                               TCKset0;\
                               TCKset1;\
                            }\
                          }

#define EnableLpmx5()   { if(devicePowerSettings.powerTestReg3VMask)  \
                          {                                           \
                            test_reg_3V                               \
                            SetReg_16Bits(0x0000);                    \
                                                                      \
                            test_reg_3V                               \
                            SetReg_16Bits_(lOut &                     \
                              ~devicePowerSettings.powerTestReg3VMask|\
                              devicePowerSettings.enableLpmx5TestReg3V)\
                                                                      \
                            EDT_Delay_1ms(20);                        \
                          }                                           \
                                                                      \
                          if(devicePowerSettings.powerTestRegMask)    \
                          {                                           \
                            test_reg                                  \
                            SetReg_32Bits(0x00000000);                \
                                                                      \
                            test_reg                                  \
                            SetReg_32Bits_(lOut_long &                \
                              ~devicePowerSettings.powerTestRegMask|  \
                              devicePowerSettings.enableLpmx5TestReg) \
                                                                      \
                            EDT_Delay_1ms(20);                        \
                          }                                           \
                        }

#define DisableLpmx5()   { if(devicePowerSettings.powerTestReg3VMask) \
                          {                                           \
                            test_reg_3V                               \
                            SetReg_16Bits(0x0000);                    \
                                                                      \
                            test_reg_3V                               \
                            SetReg_16Bits_(lOut &                     \
                              ~devicePowerSettings.powerTestReg3VMask|\
                              devicePowerSettings.disableLpmx5TestReg3V)\
                                                                      \
                            EDT_Delay_1ms(20);                        \
                          }                                           \
                                                                      \
                          if(devicePowerSettings.powerTestRegMask)    \
                          {                                           \
                            test_reg                                  \
                            SetReg_32Bits(0x00000000);                \
                                                                      \
                            test_reg                                  \
                            SetReg_32Bits_(lOut_long &                \
                              ~devicePowerSettings.powerTestRegMask|  \
                              devicePowerSettings.disableLpmx5TestReg)\
                                                                      \
                            EDT_Delay_1ms(20);                        \
                          }                                           \
                        }


#define OUT1RDY 0x0008
#define OUT0RDY 0x0004
#define IN1RDY  0x0002
#define IN0RDY  0x0001

#define JMB32B  0x0010
#define OUTREQ  0x0004
#define INREQ   0x0001

#define i_ReadJmbOut(X) { unsigned short sJMBINCTL;           \
                          unsigned short sJMBOUT0, sJMBOUT1;  \
                          sJMBINCTL = 0;                      \
                          jmb_exchange                        \
                          SetReg_16Bits(sJMBINCTL)            \
                          if(lOut & OUT1RDY)                  \
                          {                                   \
                              sJMBINCTL |= JMB32B + OUTREQ;   \
                              SetReg_16Bits(sJMBINCTL)        \
                              SetReg_16Bits(0)                \
                              sJMBOUT0 = lOut;                \
                              SetReg_16Bits(0)                \
                              sJMBOUT1 = lOut;                \
                              X = ((unsigned long)sJMBOUT1<<16) + sJMBOUT0; \
                          }                                   \
                         }

#define i_WriteJmbIn(X) { unsigned short sJMBINCTL;                   \
                          unsigned short sJMBIN0;                     \
                          unsigned long Timeout = 0;                  \
                          jtagMailboxIn = 0;                          \
                          sJMBIN0 = (unsigned short)(X & 0x0000FFFF); \
                          sJMBINCTL = INREQ;                          \
                          jmb_exchange                                \
                          do                                          \
                          {                                           \
                            SetReg_16Bits(0x0000)                     \
                            Timeout++;                                \
                            if(Timeout >= 3000)                       \
                            {                                         \
                                jtagMailboxIn = 1;                    \
                                break;                                \
                            }                                         \
                          }                                           \
                          while(!(lOut & IN0RDY) && Timeout < 3000);  \
                          if(Timeout < 3000)                          \
                          {                                           \
                            SetReg_16Bits(sJMBINCTL)                  \
                            SetReg_16Bits(sJMBIN0)                    \
                          }                                           \
                        }

#define i_WriteJmbIn32(X,Y) { unsigned short sJMBINCTL;                \
                          unsigned short sJMBIN0,sJMBIN1;              \
                          unsigned long Timeout = 0;                   \
                          jtagMailboxIn = 0;                           \
                          sJMBIN0 = (unsigned short)(X & 0x0000FFFF);  \
                          sJMBIN1 = (unsigned short)(Y & 0x0000FFFF);  \
                          sJMBINCTL =  JMB32B | INREQ;                 \
                          jmb_exchange                                 \
                          do                                           \
                          {                                            \
                            SetReg_16Bits(0x0)                         \
                            Timeout++;                                 \
                            if(Timeout >= 3000)                        \
                            {                                          \
                                jtagMailboxIn = 1;                     \
                                break;                                 \
                            }                                          \
                          }                                            \
                          while(!(lOut & IN1RDY) && Timeout < 3000);   \
                          if(Timeout < 3000)                           \
                          {                                            \
                            sJMBINCTL = 0x11;                          \
                            SetReg_16Bits(sJMBINCTL)                   \
                            SetReg_16Bits(sJMBIN0)                     \
                            SetReg_16Bits(sJMBIN1)                     \
                          }                                            \
                        }


#define i_InitStreamJmbIn32() { unsigned short sJMBINCTL;              \
                          unsigned long Timeout = 0;                   \
                          jtagMailboxIn = 0;                           \
                          jmb_exchange                                 \
                          do                                           \
                          {                                            \
                            SetReg_16Bits(0x0)                         \
                            Timeout++;                                 \
                            if(Timeout >= 3000)                        \
                            {                                          \
                                jtagMailboxIn = 1;                     \
                                break;                                 \
                            }                                          \
                          }                                            \
                          while(!(lOut & IN1RDY) && Timeout < 3000);   \
                          if(Timeout < 3000)                           \
                          {                                            \
                            sJMBINCTL = 0x11;                          \
                            SetReg_16Bits(sJMBINCTL)                   \
                          }                                            \
                        }

#define i_StreamDataJmbIn32(X,Y) { unsigned short sJMBINCTL;           \
                          unsigned short sJMBIN0,sJMBIN1;              \
                          unsigned long Timeout = 0;                   \
                          jtagMailboxIn = 0;                           \
                          sJMBIN0 = (X );                              \
                          sJMBIN1 = (Y );                              \
                          Timeout =0;                                  \
                          sJMBINCTL = 0x11;                            \
                          do                                           \
                          {                                            \
                            SetReg_16Bits(sJMBINCTL)                   \
                            Timeout++;                                 \
                          }                                            \
                          while(!(lOut & IN1RDY)&& Timeout <1500);     \
                          Timeout =0;                                  \
                          do                                           \
                          {                                            \
                            SetReg_16Bits(sJMBIN0)                     \
                            Timeout++;                                 \
                          }                                            \
                          while(lOut != 0xFFFF && Timeout <1500);      \
                            if(Timeout >= 1500){                       \
                                 jtagMailboxIn = 1;                    \
                                  break;   }                           \
                          Timeout =0;                                  \
                          do                                           \
                          {                                            \
                            SetReg_16Bits(sJMBIN1)                     \
                             Timeout++;                                \
                          }                                            \
                          while(lOut != 0xFFFF && Timeout <1500);      \
                            if(Timeout >= 1500){                       \
                                 jtagMailboxIn = 1;                    \
                                 break;   }                            \
                       }                                                                                                                                  
#endif
// this last rising edge in the SetPc macro is required for debugging,
// otherwise the EEM will trigger immediately on the instruction fetch signal
