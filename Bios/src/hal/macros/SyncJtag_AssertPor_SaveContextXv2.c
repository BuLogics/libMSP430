/**
* \ingroup MODULMACROSXV2
*
* \file SyncJtag_AssertPor_SaveContextXv2.c
*
* \brief Sync with device, assert a Power On Reset and save processor context
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

#include "arch.h"
#include "edt.h"
#include "hal.h"
#include "modules.h"
#include "stream.h"
#include "EEM_defs.h"
#include "hw_compiler_specific.h"
#include "HalGlobalVars.h"
#include "error_def.h"

extern unsigned short _hal_mclkCntrl0;
VAR_AT(unsigned char mclk_modules[16], HAL_ADDR_VAR_MCLK_MODULES);

static unsigned char clkModuleMapping[] =
{
	0x00, 0x0a, 0x1e, 0x40, 0x60,   0x8b, 0x8c, 0x8d, 0x8e, 0x8f, 
	0x90, 0x88, 0x91, 0x92, 0x93, 	0x94, 0x95, 0x96, 0x74, 0x75, 
	0x76, 0x77, 0x97, 0x98, 0x99, 	0x9a, 0x9b, 0x9c, 0x9d, 0x9e, 
	0x9f, 0x28, 0x29, 0x2a, 0x2b, 	0x2c, 0x2d, 0x2e, 0x2f, 0x30,
	0x8a, 0x8a, 0xa8, 0xa8, 0xb0,	0xb5, 0xbc, 0xbd, 0xbe, 0xbf,
	0xc0, 0xc1, 0xd4, 0xd5, 0xd6,	0xd6, 0xd8,	0x8e, 0x8f, 0x31
};

#define CLOCK_MAPPING_TABLE_SIZE (sizeof(clkModuleMapping)/sizeof(*clkModuleMapping))

/*#define wait_for_synch    { decl_out                           \
                            cntrl_sig_capture                             \
                            SetReg_16Bits(0)                              \
                            while(!(lOut & 0x0200))             \
                            {                                             \
                              SetReg_16Bits(0)                            \
                            };                                            \
                          }*/

#define wait_for_synch    { decl_out                                      \
                            int i = 0;                                    \
                            lPASS = lFAIL = 0;                            \
                            cntrl_sig_capture                             \
                            SetReg_16Bits(0)                              \
                            while(!(lOut & 0x0200) && i < 50)             \
                            {                                             \
                              SetReg_16Bits(0)                            \
                              i++;                                        \
                            };                                            \
                            if(i >= 50)                                   \
                            {                                             \
                              lFAIL++;                                    \
                            }                                             \
                            else                                          \
                            {                                             \
                              lPASS++;                                    \
                            }                                             \
                          }

/**
  SyncJtag_AssertPor_SaveContextXv2
  inData:  <wdtAddr(16)> <wdtCtrl(16)>
  outData: <wdtCtrl(16)> <PC(32)> <SR(16)>
  wdtAddr: watchdog timer address
  wdtCtrl: watchdog timer control value
  PC: program counter register (R0)
  SR: status register (R2)
*/

extern DevicePowerSettings devicePowerSettings;

HAL_FUNCTION(_hal_SyncJtag_AssertPor_SaveContextXv2)
{
  decl_out
  decl_out_long
  unsigned short MyOut[4];
  int i;
  unsigned short address;
  unsigned short wdtVal;
  
  long lPASS = 0;
  long lFAIL = 0;

    // -------------------------Power mode handling start ----------------------  
    unsigned short id = EDT_Instr(IR_CNTRL_SIG_CAPTURE);  
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
    // -------------------------Power mode handling end ------------------------
    // enable clock control before sync
    // switch all functional clocks to JCK = 1 and stop them
    eem_data_exchange32
    SetReg_32Bits_(GENCLKCTRL + WRITE);
    SetReg_32Bits_(MCLK_SEL3 + SMCLK_SEL3 + ACLK_SEL3 + STOP_MCLK + STOP_SMCLK + STOP_ACLK);
    // enable Emualtion Clocks
    eem_write_control
    SetReg_16Bits_(EMU_CLK_EN + EEM_EN);

    {
      cntrl_sig_16bit
      // release RW and BYTE control signals in low byte, set TCE1 & CPUSUSP(!!) & RW
      SetReg_16Bits_(0x1501)		
      wait_for_synch
    }
    if(lPASS > 0 && lFAIL == 0)
    {    
        // provide one more clock to empty the pipe
        TCLK
          
        cntrl_sig_16bit
        // release CPUFLUSH(=CPUSUSP) signal and apply POR signal
        SetReg_16Bits_(0x0C01)
        EDT_Delay_1ms(40); 
          
        // release POR signal again
        SetReg_16Bits_(0x0401) // disable fetch of CPU // changed from 401 to 501
        TCLK
        // drive save adress into pc
        //addr_16bit
        //SetReg_16Bits_(0xFFF0)  
        // dreiver end
        TCLK
        TCLK
        // TWO more to release CPU internal POR delay signals
        TCLK
        TCLK
        // set CPUFLUSH signal
        cntrl_sig_16bit
        SetReg_16Bits_(0x0501)
        TCLK
    
        // set EEM FEATURE enable now!!!
        eem_write_control
        SetReg_16Bits_(EMU_FEAT_EN + EMU_CLK_EN + CLEAR_STOP);
        
        // Check that sequence exits on Init State
        cntrl_sig_capture
        SetReg_16Bits_(0x0000);
    //    lout == 0x0301,0x3fd3
        
        // hold Watchdog Timer
        STREAM_get_word(&address);
        STREAM_get_word(&wdtVal);
        
        ReadMemWordXv2(address,MyOut[0]);
        wdtVal |= (MyOut[0] & 0xFF); // set original bits in addition to stop bit
        WriteMemWordXv2(address, wdtVal);
        
        // Capture MAB - the actual PC value is (MAB - 4)
        addr_capture
        SetReg_20Bits(0)
        /*****************************************/
        /* Note 1495, 1637 special handling      */
        /*****************************************/
        if(lOut_long == 0xFFFE)
        {
          ReadMemWordXv2(0xFFFE,MyOut[1]);
          MyOut[2] = 0;
        }
        /*********************************/
        else
        {
          lOut_long -= 4;
          MyOut[1] = (unsigned short)(lOut_long & 0xFFFF);
          MyOut[2] = (unsigned short)(lOut_long >>16);
        }
        // Status Register should be always 0 after a POR
        MyOut[3] = 0;
        STREAM_discard_bytes(1);
        for(i=0; i<16; i++)
        {
          unsigned short v;
          unsigned char clkModule = 0;
          STREAM_get_byte(&mclk_modules[i]);
          
          //Map received index to actual clock module
          if ( mclk_modules[i] < CLOCK_MAPPING_TABLE_SIZE )
          {
              clkModule = clkModuleMapping[ mclk_modules[i] ];
          }

          if(clkModule != 0)
          {
            if(_hal_mclkCntrl0 & (0x0001 << i))
              v = 1;
            else
              v = 0;
            WriteMemWordXv2(ETKEYSEL, ETKEY + clkModule);
            WriteMemWordXv2(ETCLKSEL, v);
          }
        }
        // switch back system clocks to original clock source but keep them stopped
        eem_data_exchange32
        SetReg_32Bits_(GENCLKCTRL + WRITE);
        SetReg_32Bits_(MCLK_SEL0 + SMCLK_SEL0 + ACLK_SEL0 + STOP_MCLK + STOP_SMCLK + STOP_ACLK);
        // configure cycle counter
        // Increment on all bus cycles (including DMA cycles)
        // Start when CPU released from JTAG/EEM
        // Stop when CPU is stopped by EEM or under JTAG control
                     
        SetReg_32Bits_(CCNT0CTL + WRITE);
        SetReg_32Bits_(CCNT_RST + CCNTMODE5);
    
        STREAM_put_bytes((unsigned char*)MyOut,8);
        
        return 0; // retrun status OK
    }
    else
    {
        return HALERR_UNDEFINED_ERROR; // return errer in case if syc fails
    }
}

