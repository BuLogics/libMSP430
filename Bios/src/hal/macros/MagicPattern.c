/**
* \ingroup MODULMACROS
*
* \file MagicPattern.c
*
* \brief Get control over the device
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

#include "error_def.h"
#include "arch.h"
#include "edt.h"
#include "hal.h"
#include "stream.h"
#ifdef DB_PRINT
#include "debug.h"
#endif

/**
  MagicPattern 
  This function will try to get control over the device. It will try to reset and
  stop the device.
  It will automatically switch between the different JTAG protocols also it handles
  the LPMx.5 

  inData:  void
  outData: <chainLen(8)>
  outData: <jtagId(8)>

  protocol: 0 = 4-wire Jtag
  jtagId: the Jtag identifier value
*/

#ifdef uController_uif
extern void JSBW_EntrySequences(unsigned char states);
extern void JSBW_TapReset(void);
extern void JSBW_MagicPattern(void);
extern void JSBW_JtagUnlock(void);
extern void jRelease(void);
#endif

extern DevicePowerSettings devicePowerSettings;

HAL_FUNCTION(_hal_MagicPattern)
{
    short ret_value = HALERR_UNDEFINED_ERROR; 
    short i = 0;    
    decl_out
    decl_out_long
    
    unsigned short id=0;
    unsigned char chainLen;
    unsigned short protocol = JTAG_IF;
    
    while( i < 4)
    {  
        decl_jtagMailboxIn
        if( i == 0 || i == 1)
        {
           protocol = SPYBIWIRE_IF;
        }
        else
        {   
           protocol = JTAG_IF;
        }

        EDT_SetProtocol(protocol);

        // run entry sequnce but pull rst low during the sequence to wake-up the device
        EDT_Open(RSTLOW);
        EDT_TapReset();      
        chainLen = 1; //;(unsigned char)EDT_EnumChain();
        EDT_CheckJtagFuse();
        // put in magic pattern to stop user code execution
        i_WriteJmbIn(MAGIC_PATTERN)
            
        if(jtagMailboxIn == 1)
        {
            ret_value = (HALERR_JTAG_MAILBOX_IN_TIMOUT);
        }
        
        //EDT_Close();
        // run entry sequnce but pull rst high during the sequence 
        EDT_Open(RSTHIGH);
        EDT_TapReset();      
        chainLen = 1; //;(unsigned char)EDT_EnumChain();
        EDT_CheckJtagFuse();

        id = EDT_Instr(IR_CNTRL_SIG_CAPTURE);     
        if (id == JTAGVERSION95 || id == JTAGVERSION91 || id == JTAGVERSION99)
        {
            //Disable Lpmx5 and power gaiting settings 
            if(id == JTAGVERSION91)
            {
                // just disable JTAG i lock
                test_reg_3V                               
                SetReg_16Bits(0x4020);
            }
            if(id == JTAGVERSION99)
            {
                test_reg_3V                               
                SetReg_16Bits(0x4020);                                
                test_reg                                  
                SetReg_32Bits(0x00010000);     
            }            

            STREAM_put_byte((unsigned char)chainLen);  
            STREAM_put_byte((unsigned char)id);
            STREAM_put_byte((unsigned char)protocol);
                        
            return 0;
        }   
        i++;
    } 
    
    // Force wakeup in SBW4 mode for LPMx.5 because JTAG pins are locked by LPMx.5
    // This case happens if the device is woken up, but goes into LPMx.5 very quickly
    // and the device wasn't under JTAG control from a previous debug session
#ifdef uController_uif
    
    i = 0;
    protocol = SPYBIWIREJTAG_IF;
    EDT_SetProtocol(protocol);
    
    while( i < 3)
    {
        //  feet in JSBW magic pattern Use real RST and TST pin of TOOL      
        JSBW_EntrySequences(0);        
        JSBW_TapReset();
        JSBW_MagicPattern();
        EDT_Delay_1ms(10);
        jRelease();          
        
        // disable JTAG lock
        JSBW_EntrySequences(2);   
        JSBW_TapReset();
        JSBW_JtagUnlock();
        EDT_Delay_1ms(10);        
        
        // reconnect in normal JTAG 4 wire mode            
        EDT_Open(RSTHIGH);
        EDT_TapReset();      
        chainLen = 1; //;(unsigned char)EDT_EnumChain();
        EDT_CheckJtagFuse();
        
        id = EDT_Instr(IR_CNTRL_SIG_CAPTURE);
        if (id == JTAGVERSION95 || id == JTAGVERSION91 || id == JTAGVERSION99)
        {
            // Disable JTAG lock
            // DisableLpmx5();
            
            STREAM_put_byte((unsigned char)chainLen);  
            STREAM_put_byte((unsigned char)id);
            STREAM_put_byte((unsigned char)protocol);            
            return 0;
        }   
        i++;            
    }       
#endif
   
    return(ret_value);
}

