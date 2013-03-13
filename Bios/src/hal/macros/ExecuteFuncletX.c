/**
* \ingroup MODULMACROS
*
* \file ExecuteFuncletX.c
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

//! Funclet parameters:
//!
//!   Parameter       Register
//!   =========       ========
//!   Address         R5
//!   Size            R6
//!   LockA           R8
//!   Type            R9
//!
//!   General Purpose Registers used by funclet
//!   =========================================
//!   The following registers are used by the funclet for general purpose. The
//!   debugger is responsible for restoring the value of the registers after 
//!   completion.
//!
//!   R10             Memory pointer
//!   R11             Compare register/Outer loop counter
//!   R12             Inner delay loop counter
//!
//!   Funclet memory map
//!   ==================
//!   The Funclet uses a very specifc memory map, so that the debugger can check
//!   for specific values on the MAB to track the progress of the funclet. The
//!   following table documents this memory map:
//!
//!   Section:        Location:       Size:     Remarks:
//!   --------        ---------       --------  --------
//!   Initialization  0000h - 003Eh   32 words  Initializaiton of variables
//!   WaitforDeadLoop 0040h - 0046h    3 words  Waiting for the debugger acknowledge
//!   CodeBody        0048h - 00ECh   84 words  Code body which performs actual task
//!   Stop            00EEh            1 word   End of program
//!   ControlWord     00F0h            1 word   Control word to comms between debugger and funclet
//!   FCTL1Value      00F2h            1 word   Saved value for restoring later
//!   FCTL2Value      00F4h            1 word   Saved value for restoring later
//!   FCTL3Value      00F6h            1 word   Saved value for restoring later
//!   UserData        0100h - xxxxh    -        Specific data to be written to flash
//!
/*---------------------------------------------------------------------------*/


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

#define REG_ADDRESS 5
#define REG_SIZE    6
#define REG_LOCKA   8
#define REG_TYPE    9

#define REG_GP1     10  // General purpose registers used by the funclet
#define REG_GP2     11
#define REG_GP3     12

#define WAIT_FOR_DEAD_START (0x22)  // Code position where the WaitForDead loop starts
#define WAIT_FOR_DEAD_END   (0x2C)  // Code position where the WaitForDead loop ends
#define EXECUTE_FUNCLET     (0x2E)  // Location of actual funclet body
#define FINISHED_OFFSET     (0x68)  // Location of the final jmp $ instruction of the funclet
#define CONTROL_WORD_OFFSET (0x6A)  // Location of the control word
#define DATA_OFFSET         (0x6C)  // Location where data starts

static void setFuncletRegisters(const unsigned long* registerData)
{
    WriteCpuRegX(REG_ADDRESS, registerData[0])
    WriteCpuRegX(REG_SIZE, registerData[1])
    WriteCpuRegX(REG_TYPE, registerData[2])
    WriteCpuRegX(REG_LOCKA, registerData[3])
	WriteCpuRegX(REG_GP1, registerData[4]) 
	WriteCpuRegX(REG_GP2, registerData[5]) 
	WriteCpuRegX(REG_GP3, registerData[6]) 
}

HAL_FUNCTION(_hal_ExecuteFuncletX)
{
    short ret_value = 0;
    static unsigned long  lLen;
    static unsigned short ret_len = 0;
    static unsigned long Addr = 0x0;
    static unsigned short memSize =0x0;
    static unsigned short LockA =0x0;
    static unsigned short usType =0x0;
    static unsigned short startAddr;
    static unsigned short R12_BCSLTC1;
    static unsigned short R11_DCO;   

    
    static unsigned long registerBackups[7] = {0};

    static unsigned short FCTL1Value;
    static unsigned short FCTL2Value;
    static unsigned short FCTL3Value;     

    unsigned short tmpFCTL3Value;    
    
    unsigned short tgtStart     =0x0;
    unsigned short data         =0x0;
    unsigned short timeOut      =3000;
    decl_out
    decl_out_long

    StreamSafe stream_tmp;
  
    if(flags & MESSAGE_NEW_MSG)
    {
        // get target RAM start
        if(STREAM_get_word(&tgtStart) != 0)
        {
            return(HALERR_EXECUTE_FUNCLET_NO_RAM_START);
        }
       /// get available RAM size (ram - funclet size)
        if(STREAM_get_word(&memSize)!= 0) 
        {
            return(HALERR_EXECUTE_FUNCLET_NO_RAM_SIZE);
        }        
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
        
        if(STREAM_get_word(&R11_DCO) == -1)
        {
            return(HALERR_EXECUTE_FUNCLET_NO_LOCKA);
        }
        
        if(STREAM_get_word(&R12_BCSLTC1) == -1)
        {
            return(HALERR_EXECUTE_FUNCLET_NO_LOCKA);
        }

        ReadCpuRegX(REG_ADDRESS, registerBackups[0])
        ReadCpuRegX(REG_SIZE, registerBackups[1])
        ReadCpuRegX(REG_TYPE, registerBackups[2])
        ReadCpuRegX(REG_LOCKA, registerBackups[3]) 
		ReadCpuRegX(REG_GP1, registerBackups[4]) 
		ReadCpuRegX(REG_GP2, registerBackups[5]) 
		ReadCpuRegX(REG_GP3, registerBackups[6]) 

        // Setup the Flash Controller
        // Read FCTL registers for later restore
        ReadMemWordX(0x128,FCTL1Value)
        ReadMemWordX(0x12A,FCTL2Value)            
        ReadMemWordX(0x12C,FCTL3Value)
        // Restore password byte
        FCTL1Value ^= 0x3300;
        FCTL2Value ^= 0x3300;
        FCTL3Value ^= 0x3300;
        
        WriteMemWordX(0x12A,0xA544)     // Source = MCLK, DIV = 5.
        WriteMemWordX(0x12C,LockA)      // Set LockA
        ReadMemWordX(0x12C,tmpFCTL3Value)    // Read out register again
              
        if((LockA & 0xff) != (tmpFCTL3Value & 0xff))
        {   // Value of lockA is not as expected, so toggle it
            WriteMemWordX(0x12C,LockA | 0x40);
        }

        {
            unsigned long registerValues[7] = {Addr, lLen, usType, LockA, 0, R11_DCO, R12_BCSLTC1};
            setFuncletRegisters(registerValues);
            WriteCpuRegX(2, 0);
        }

        // i_SetPcRel 
        SetPcX(startAddr);
        // prepare release & release
        cntrl_sig_16bit
        SetReg_16Bits_(0x0401)
        addr_capture
        cntrl_sig_release
        
        // Poll until the funclet reaches the WaitForDead loop, 
        // ie. it is ready to process data
        do
        {
            EDT_Delay_1ms(1);
            addr_capture
            SetReg_20Bits(0)
            timeOut--;
        } 
        while(!((lOut_long >= (startAddr + WAIT_FOR_DEAD_START)) && (lOut_long <= (startAddr + WAIT_FOR_DEAD_END))) && timeOut);
    }

    while (lLen && (ret_value == 0) && timeOut)
    {
      unsigned short writePos = startAddr + DATA_OFFSET;
      const unsigned short writeEndPos = writePos + (2*lLen < memSize ? 2*lLen : memSize);
      unsigned short numWordsToWrite = 0;

      SetPcX(startAddr + FINISHED_OFFSET);
      
      // The device has limited RAM available to write updates to,
      // make sure we stay within this limit: Total memory size - 96 bytes for the funclet
      halt_cpu
      TCLKset0
      cntrl_sig_low_byte
      SetReg_16Bits_(0x08)
          
      while( (writePos < writeEndPos) && (ret_value == 0) )
      {
          ret_value = STREAM_get_word(&data);
          addr_16bit
          SetReg_20Bits_(writePos)
          data_to_addr
          SetReg_16Bits_(data);
          TCLKset1
          TCLKset0
          writePos += 2;
      }
      
      release_cpu
        
      numWordsToWrite = (writePos - (startAddr + DATA_OFFSET)) / 2;

	  WriteCpuRegX(REG_SIZE, numWordsToWrite)
      WriteCpuRegX(REG_ADDRESS, Addr)

      Addr += 2 * numWordsToWrite;
      lLen -= numWordsToWrite;

      SetPcX(startAddr + EXECUTE_FUNCLET);
      cntrl_sig_release

      // Poll until the funclet reaches the Stop loop, 
      // ie. it is finished
      timeOut = 3000;
      do
      {
          EDT_Delay_1ms(1);
          addr_capture
          SetReg_20Bits(0)
          timeOut--;
      }
      while(!(lOut_long == (startAddr + FINISHED_OFFSET)) && timeOut);
    }
    
    if(flags & MESSAGE_LAST_MSG )
    { // ExitFlashWrite
        {   //Setup values for watchdog control regsiters
            unsigned char DummyIn[8] = {WDTCTL_ADDRESS & 0xFF,(WDTCTL_ADDRESS >> 8) & 0xFF,
                                        WDTHOLD_DEF,WDTPW_DEF,0,0,0,0};
            STREAM_internal_stream(DummyIn, sizeof(DummyIn), NULL, 0, &stream_tmp);
            HAL_SyncJtag_Conditional_SaveContextX(MESSAGE_NEW_MSG | MESSAGE_LAST_MSG);
            STREAM_external_stream(&stream_tmp);
        }
        setFuncletRegisters(registerBackups);
        // Restore the Flash controller registers
        WriteMemWordX(0x128,FCTL1Value)
        WriteMemWordX(0x12A,FCTL2Value)
        WriteMemWordX(0x12C,FCTL3Value)
        ReadMemWordX(0x12C,tmpFCTL3Value)    // Read out register again
        
        if((FCTL3Value & 0xff) != (tmpFCTL3Value & 0xff))
        {   // Value of lockA is not as expected, so toggle it
            WriteMemWordX(0x12C,FCTL3Value | 0x40);
        }            
        
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
         unsigned char DummyIn[8] = {WDTCTL_ADDRESS & 0xFF,(WDTCTL_ADDRESS >> 8) & 0xFF,
                                        WDTHOLD_DEF,WDTPW_DEF,0,0,0,0};
         STREAM_internal_stream(DummyIn, sizeof(DummyIn), NULL, 0, &stream_tmp);
         HAL_SyncJtag_Conditional_SaveContextX(MESSAGE_NEW_MSG | MESSAGE_LAST_MSG);
         STREAM_external_stream(&stream_tmp);
         
         setFuncletRegisters(registerBackups);
         // Restore the Flash controller registers
         WriteMemWordX(0x128,FCTL1Value)
         WriteMemWordX(0x12A,FCTL2Value)
         WriteMemWordX(0x12C,FCTL3Value)
         ReadMemWordX(0x12C,tmpFCTL3Value)    // Read out register again
        
         if((FCTL3Value & 0xff) != (tmpFCTL3Value & 0xff))
         {   // Value of lockA is not as expected, so toggle it
             WriteMemWordX(0x12C,FCTL3Value | 0x40);
         }                         
         
         ret_value = HALERR_EXECUTE_FUNCLET_EXECUTION_ERROR;
    }  
    return(ret_value);
}
