/**
* \ingroup MODULMACROSXV2
*
* \file ExecuteFuncletXv2.c
*
* \brief  Execute a funclet on the target
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

#include "hw_compiler_specific.h"
#include "HalGlobalVars.h"
#include "arch.h"
#include "edt.h"
#include "hal.h"
#include "hal_ref.h"
#include "stream.h"
#include "stddef.h"
#include "error_def.h"
#include "JTAG_defs.h"

#define startAddrOfs FlashWrite_o_[4]

#define REG_ADDRESS 5
#define REG_SIZE    6
#define REG_LOCKA   8
#define REG_TYPE    9

#define TIMEOUT_COUNT   300u
/*
#define AddrL        FlashWrite_o_[6]
#define AddrH        FlashWrite_o_[7]
#define LengthL      FlashWrite_o_[8]
#define LengthH      FlashWrite_o_[9]
#define LockA        FlashWrite_o_[10]
*/

extern unsigned short altRomAddressForCpuRead;


static void setFuncletRegisters(const unsigned long* registerData)
{
    unsigned long  Rx;
    unsigned short Mova;
    unsigned short Rx_l;
    unsigned short Register;

    DIAG_SUPPRESS(Pe550)
    decl_out_long
    DIAG_DEFAULT(Pe550)

    Mova  = 0x0080;
    Rx = registerData[0];
    Register = REG_ADDRESS;
    Mova += (unsigned short)((Rx>>8) & 0x00000F00);
    Mova += (Register & 0x000F);
    Rx_l  = (unsigned short)Rx;
    WriteCpuRegXv2(Mova, Rx_l);
    
    Mova  = 0x0080;
    Rx = registerData[1];
    Register = REG_SIZE;
    Mova += (unsigned short)((Rx>>8) & 0x00000F00);
    Mova += (Register & 0x000F);
    Rx_l  = (unsigned short)Rx;
    WriteCpuRegXv2(Mova, Rx_l);
    
    Mova  = 0x0080;
    Rx = registerData[2]; // erase type
    Register = REG_TYPE;
    Mova += (unsigned short)((Rx>>8) & 0x00000F00);
    Mova += (Register & 0x000F);
    Rx_l  = (unsigned short)Rx;
    WriteCpuRegXv2(Mova, Rx_l);
    
    Mova  = 0x0080;
    Rx = registerData[3];
    Register = REG_LOCKA;
    Mova += (unsigned short)((Rx>>8) & 0x00000F00);
    Mova += (Register & 0x000F);
    Rx_l  = (unsigned short)Rx;
    WriteCpuRegXv2(Mova, Rx_l);
}

/**
  WriteFlashBlockXv2
  Write words (16bit values) to a memory mapped Flash location (in Block mode).
  inData:  <tgtStart(16)> <tgtLen(16)> <addr(32)> <length(32)> <data(16)>{*}
  outData: <tgtLen_used(16)>
  tgtStart: target memory start address used to load erase code to.
  tgtLen: available target memory 
  addr: the address to start writing to
  length: number of words to write
  data: data to write to the given location
  tgtLen_used: actual amount of target memory used to execute erase code
  (must be <= length)
*/

HAL_FUNCTION(_hal_ExecuteFuncletXv2)
{
    static unsigned long  lLen;
    static unsigned short ret_len = 0;
    
    static unsigned long registerBackups[4] = {0};
    
    unsigned short tgtStart     =0x0 ;
    unsigned long Addr          =0x0 ;
    unsigned short LockA        =0x0 ;
    unsigned short usType       =0x0 ;

    unsigned short startAddr;
    unsigned short Mova;
    static unsigned short allignNeed = 0x0;
    static unsigned short dataL;
    static unsigned short dataH;
    decl_out
    decl_jtagMailboxIn
    short ret_value = 0;
    StreamSafe stream_tmp;
  
    if(flags & MESSAGE_NEW_MSG)
    {
        // get target RAM start
        if(STREAM_get_word(&tgtStart) != 0)
        {
            return(HALERR_EXECUTE_FUNCLET_NO_RAM_START);
        }
        STREAM_discard_bytes(2); // Ram size
        /// get RAM Start + Code offset   
        if(STREAM_get_word(&startAddr)!= 0) 
        {
            return(HALERR_EXECUTE_FUNCLET_NO_OFFSET);
        }
        // get start addres as long value
        if(STREAM_get_long(&Addr) != 0)
        {
            return(HALERR_EXECUTE_FUNCLET_NO_ADDRESS);
        }
        // get length ot be flashed
        if(STREAM_get_long(&lLen) != 0)
        {
            return(HALERR_EXECUTE_FUNCLET_NO_LENGTH);    
        }   
        if(STREAM_get_word(&usType) != 0)  
        {
            return(HALERR_EXECUTE_FUNCLET_NO_TYPE);
        }
        // lock A handling 
        if(STREAM_get_word(&LockA) == -1)
        {
            return(HALERR_EXECUTE_FUNCLET_NO_LOCKA);
        }

        if(STREAM_discard_bytes(4) == -1)
        {
            return(HALERR_EXECUTE_FUNCLET_NO_LOCKA);
        }

        /*Mova = 0x0060 | (REG_ADDRESS << 8) & 0x0F00;
        ReadCpuRegXv2(Mova, registerBackups[0])
        Mova = 0x0060 | (REG_SIZE << 8) & 0x0F00;
        ReadCpuRegXv2(Mova, registerBackups[1])
        Mova = 0x0060 | (REG_TYPE << 8) & 0x0F00;
        ReadCpuRegXv2(Mova, registerBackups[2])
        Mova = 0x0060 | (REG_LOCKA << 8) & 0x0F00;
        ReadCpuRegXv2(Mova, registerBackups[3])*/
        
        {
            unsigned long registerValues[4] = {Addr, lLen, usType, LockA};        
            setFuncletRegisters(registerValues);
        }
        
        //WriteMemWordXv2(0xFFFE,0xFFFF)     // Source = MCLK, DIV = 5.

        // i_SetPcRel 
        SetPcXv2(0x0080, startAddr);
        TCLKset1
        // prepare release & release
        cntrl_sig_16bit
        SetReg_16Bits_(0x0401)
        addr_capture
        cntrl_sig_release            
        {
            unsigned long Jmb = 0;
            unsigned long Timeout = 0;          
            do
            {
                EDT_Delay_1ms(5);    // Delay determined experimentally on FR5729 device             
                i_ReadJmbOut(Jmb)
                Timeout++;
            }      
            while(Jmb != 0xABADBABE && Timeout < TIMEOUT_COUNT);
            if(Timeout >= TIMEOUT_COUNT)
            {
                ret_value = HALERR_EXECUTE_FUNCLET_EXECUTION_TIMEOUT;
            }
        }
        if(lLen%2)
        {
             lLen--;  // allign lLen        
             allignNeed = 1; 
        }
        //i_InitStreamJmbIn32()
    }

    // Excecute funclet    
    for(; lLen && (ret_value == 0); lLen--)
    {
      if (lLen%2)
      {
          ret_value = STREAM_get_word(&dataH);
          //i_StreamDataJmbIn32(dataL,dataH) 
          i_WriteJmbIn32(dataL,dataH)   
      }
      else
      {
          ret_value = STREAM_get_word(&dataL);
      }
    }
    
    if(allignNeed && ret_value == 0 && lLen == 0 && flags & MESSAGE_LAST_MSG )
    {
         ret_value = STREAM_get_word(&dataL);
         i_WriteJmbIn32(dataL,0xAAAA);   
         
         if(jtagMailboxIn == 1)
         {
            return (HALERR_EXECUTE_FUNCLET_FINISH_TIMEOUT);
         }       
         allignNeed = 0;
    }         
   
    if(flags & MESSAGE_LAST_MSG )//|| lLen == 0)
    { // ExitFlashWrite
        unsigned long Timeout = 0;
        unsigned long Jmb = 0;
        do
        {
            EDT_Delay_1ms(10);     // Delay determined experimentally on FR5729 device                   
	    i_ReadJmbOut(Jmb)
	    Timeout++;
        }
        while(Jmb != 0xCAFEBABE && Timeout < TIMEOUT_COUNT);
        if(Timeout >= TIMEOUT_COUNT)
        {
            ret_value = HALERR_EXECUTE_FUNCLET_FINISH_TIMEOUT;
                        //Setup values for watchdog control regsiters
            
            unsigned char DummyIn[8] = {WDTCTL_ADDRESS_5XX & 0xFF,(WDTCTL_ADDRESS_5XX >> 8) & 0xFF,
                                    WDTHOLD_DEF,WDTPW_DEF,0,0,0,0};
            
            STREAM_internal_stream(DummyIn, sizeof(DummyIn), NULL, 0, &stream_tmp);
            HAL_SyncJtag_Conditional_SaveContextXv2(MESSAGE_NEW_MSG | MESSAGE_LAST_MSG);
            STREAM_external_stream(&stream_tmp);
            
            Mova = 0x0060 | (10 << 8) & 0x0F00;
            ReadCpuRegXv2(Mova, registerBackups[0])
            Mova = 0x0060 | (11 << 8) & 0x0F00;
            ReadCpuRegXv2(Mova, registerBackups[1])
            
            if( registerBackups[1] != 0 && registerBackups[0] != 0xFFFE)
            {
                ret_value = -10;
            }
                   
        }
        {
            //Setup values for watchdog control regsiters
            unsigned char DummyIn[8] = {WDTCTL_ADDRESS_5XX & 0xFF,(WDTCTL_ADDRESS_5XX >> 8) & 0xFF,
                                    WDTHOLD_DEF,WDTPW_DEF,0,0,0,0};
            
            STREAM_internal_stream(DummyIn, sizeof(DummyIn), NULL, 0, &stream_tmp);
            HAL_SyncJtag_Conditional_SaveContextXv2(MESSAGE_NEW_MSG | MESSAGE_LAST_MSG);
            STREAM_external_stream(&stream_tmp);
            
        }
        //setFuncletRegisters(registerBackups);
        
        STREAM_put_word(ret_len);
    }
    else if(ret_value == 1)
    {
        STREAM_out_change_type(RESPTYP_ACKNOWLEDGE);
        ret_value = 0;
    }
    else
    {
         //Setup values for watchdog control regsiters
         unsigned char DummyIn[8] = {WDTCTL_ADDRESS_5XX & 0xFF,(WDTCTL_ADDRESS_5XX >> 8) & 0xFF,
                                     WDTHOLD_DEF,WDTPW_DEF,0,0,0,0};
      
         STREAM_internal_stream(DummyIn, sizeof(DummyIn), NULL, 0, &stream_tmp);
         HAL_SyncJtag_Conditional_SaveContextXv2(MESSAGE_NEW_MSG | MESSAGE_LAST_MSG);
         STREAM_external_stream(&stream_tmp);
         
         //setFuncletRegisters(registerBackups);
         
         ret_value = HALERR_EXECUTE_FUNCLET_EXECUTION_ERROR;
    }  
    return(ret_value);
}
