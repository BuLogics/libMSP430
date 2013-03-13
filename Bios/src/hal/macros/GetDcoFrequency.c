/**
* \ingroup MODULMACROS
*
* \file GetDcoFrequency.c
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

#include "arch.h"
#include "edt.h"
#include "hal.h"
#include "hal_ref.h"
#include "stream.h"
#include "JTAG_defs.h"
#include "error_def.h"
#include "../include/hw_compiler_specific.h"

#define FlashUpperBoarder 2140000ul // 2,14 MHz
#define FlashLowerBoarder 1410000ul // 1,41 MHz

static unsigned short Time = 0; 
/* clock on pin
static unsigned short loopDco[] =
{
    0x40B2, 0x5A80, 0x0120, 0xC232, 0x45C2, 0x0056, 0x46C2, 0x0057,
    0x43C2, 0x0058, 0x40F2, 0x0010, 0x0022, 0x40F2, 0x0010, 0x0026,
    0xEA0A, 0xE909, 0x5319, 0x23FE, 0x531A, 0x23FC, 0x4303, 0x3FFF
};
*/

static unsigned short loopDco[] =
{
    0x40b2, 0x5a80, 0x0120, 0xc232, 0x40f2, 0x0087, 0x0057, 0x45c2, 
    0x0056, 0x46c2, 0x0057, 0x43c2, 0x0058, 0xea0a, 0xe909, 0x5319, 
    0x23fe, 0x531a, 0x23fc, 0x4303, 0x3fff
};
static const unsigned short sizeLoopDco = (unsigned short)(sizeof(loopDco)/sizeof(*loopDco));

static unsigned short loopFll[] =
{
    0x40b2, 0x5a80, 0x0120, 0xc232, 0xd072, 0x0040, 0x4032, 0x0040, 
    0x40f2, 0x0080, 0x0052, 0x43c2, 0x0050, 0x45c2, 0x0051, 0x40f2, 
    0x0080, 0x0053, 0x43c2, 0x0054, 0xea0a, 0xe909, 0x5319, 0x23fe, 
    0x531a, 0x23fc, 0x4303, 0x3fff
    /*0x40b2, 0x5a80, 0x0120, 0xc232, 0xd072, 0x0040, 0x4032, 0x0040, 
    0x40f2, 0x0080, 0x0052, 0x43c2, 0x0050, 0x45c2, 0x0051, 0x40f2, 
    0x0080, 0x0053, 0x43c2, 0x0054, 0xd0f2, 0x0010, 0x0022, 0xd0f2, 
    0x0010, 0x0026, 0xea0a, 0xe909, 0x5319, 0x23fe, 0x531a, 0x23fc, 
    0x4303, 0x3fff*/
};
static const unsigned short sizeLoopFll = (unsigned short)(sizeof(loopFll)/sizeof(*loopFll));


#pragma vector=TIMERA1_VECTOR
__interrupt void TIMER_A_ISR_(void)
{
    Time++;
    TACTL &= ~TAIFG;
}

static void StartTimer()
{
    TACTL =  0;                                          // STOP Timer
    TACTL &= ~CCIFG;                                     // Clear the interrupt flag  
    TACTL =  TASSEL_2;                            // Timer is runnig at 1 Mhz
    TACCR0 = 0x2E9;                                      // Load CCR0 with delay... (1ms delay)         
    TAR = 0; 
    TACTL |= TACLR + MC_1 + TAIE;                               // Start Timer
}

static void StopTimer()
{
    TACTL &= ~MC_1;
    TACTL &= ~TAIE;                               // Start Timer
}


static unsigned long (*ReadCounterRegsFunc)() = 0;
static void (*WriteRegFunc)(int, unsigned long) = 0;
static void (*SetPCFunc)(unsigned long) = 0;
static void (*WriteRamFunc)(unsigned short, unsigned short*, unsigned short) = 0;
static void (*ReadRamFunc)(unsigned short, unsigned short*, unsigned short) = 0;
static HalFuncInOut SyncFunc = 0;


static void readFromRam(unsigned short address, unsigned short* buffer, unsigned short numWords)
{
    decl_out
    while (numWords-- > 0)
    {
        ReadMemWord(address, *buffer);  
        address += 2;
        ++buffer;
    }
}

static void writeToRam(unsigned short address, unsigned short* data, unsigned short numWords)
{
    while (numWords-- > 0)
    {
        WriteMemWord(address, *data);  
        address += 2;
        ++data;
    }
}

static void readFromRamX(unsigned short address, unsigned short* buffer, unsigned short numWords)
{
    decl_out
    decl_out_long
    while (numWords-- > 0)
    {
        ReadMemWordX(address, *buffer);  
        address += 2;
        ++buffer;
    }
}

static void writeToRamX(unsigned short address, unsigned short* data, unsigned short numWords)
{
    while (numWords-- > 0)
    {
        WriteMemWordX(address, *data);
        address += 2;
        ++data;
    }
}

static unsigned long readCounterRegisters()
{
    decl_out
    unsigned short r9, r10;
    unsigned long counter = 0;

    ReadCpuReg_uShort(9, r9);
    ReadCpuReg_uShort(10, r10);
    counter = r10;
    return (counter << 16) | r9;
}

static unsigned long readCounterRegistersX()
{
    decl_out
    unsigned long r9, r10;

    ReadCpuRegX(9, r9);
    ReadCpuRegX(10, r10);
    return (r10 << 16) | r9;
}

static void writeRegister(int r, unsigned long value)
{
    WriteCpuReg(r, value);
}

static void writeRegisterX(int r, unsigned long value)
{
    WriteCpuRegX(r, value);
}

static void setPC(unsigned long address)
{
    SetPc(address);
}

static void setPCX(unsigned long address)
{
    SetPcX(address);
}

static void setPCJtag(unsigned long address)
{
    SetPcJtagBug(address);
    TCLKset1
}



static unsigned long measureFrequency(unsigned short RamStart, unsigned short DCO, unsigned short BCS1)
{
    unsigned long startTime;
    unsigned long stopTime;
    unsigned long elapseTime = 0;
    unsigned long counter = 0;
    unsigned long freq = 0;

    WriteRegFunc(5, DCO);
    WriteRegFunc(6, BCS1);
    
    SetPCFunc(RamStart);
    cntrl_sig_release
    
    StartTimer(); // Time the delay (as we could be interrupted, etc.).
    startTime = Time; // System timestamp (in milliseconds).

    __delay_cycles(240000ul); //~30ms

    stopTime = Time;
    StopTimer(); 
    
    { 
        StreamSafe stream_tmp;
        unsigned char DummyIn[8] = {0x20,0x01,0x80,0x5A,0,0,0,0}; // wdtAddr, wdtCtrl
        STREAM_internal_stream(DummyIn, sizeof(DummyIn), 0, 0, &stream_tmp);
        SyncFunc(MESSAGE_NEW_MSG | MESSAGE_LAST_MSG);
        STREAM_external_stream(&stream_tmp);
    }

    elapseTime = startTime < stopTime ? stopTime - startTime : startTime - stopTime; // System time can wrap.

    counter = ReadCounterRegsFunc();

    // Calculate a device speed (cycles/second). Multiply counter value by 3 (3 clock cycles per loop),
    // and add 31 cycles for funclet setup, then normalize to one second.
    Time = 0; 
    freq = ((counter * 3 + 31) * 1000) / elapseTime;

    return freq;
}


short findDcoSettings(unsigned short jtagBug)
{
    unsigned short MAXRSEL   = 0x7;
    unsigned short DCO  = 0x0;
    unsigned short BCS1 = 0x6;
    unsigned short BackupRam[30];
    unsigned short RamStart = 0;
    unsigned long  DcoFreq = 0;
      
    // get target RAM start
    if(STREAM_get_word(&RamStart) < 0)
    {
        return(HALERR_EXECUTE_FUNCLET_NO_RAM_START);
    }

    if(STREAM_get_word(&MAXRSEL) < 0)
    {
        return(HALERR_EXECUTE_FUNCLET_NO_MAXRSEL);
    }
    
    if (MAXRSEL == 0xF)
    {
        BCS1 = 0x9;
    }
    
    SetPCFunc(ROM_ADDR); //Prevent Ram corruption on F123/F413
      
    //----------------Backup original Ram content--------------
    ReadRamFunc(RamStart, BackupRam, sizeLoopDco);

    // ----------------Download DCO measure funclet------------
    WriteRamFunc(RamStart, loopDco, sizeLoopDco);

    // do measurement
    int allowedSteps = 40;

    do
    {
        DcoFreq = measureFrequency(RamStart, (DCO<<5), (0x80|BCS1));
        //Ram content will probably be corrupted after each measurement on devices with jtag bug
        //Reupload on every iteration
        if (jtagBug)
        {
            WriteRamFunc(RamStart, loopDco, sizeLoopDco);
        }
        
        if (DcoFreq == 0)
        {
            return (-1);
        }
        
        if (DcoFreq > FlashUpperBoarder) // Check for upper limit - 10%.
        {
            if(DCO-- == 0)
            {
                DCO = 7;
                if (BCS1-- == 0)
                {  
                    return (-1); // Couldn't get DCO working with correct frequency.
                }
            } 
        }        
        else if (DcoFreq < FlashLowerBoarder) // Check for lower limit + 10%.
        {
            if(++DCO > 7)
            {
                DCO = 0;
                if (++BCS1 > MAXRSEL)
                {
                    return (-1); // Couldn't get DCO working with correct frequency.
                }
            }
        }
        else
        {
            break; 
        }
    }
    while (--allowedSteps > 0);

    // restore Ram content 
    WriteRamFunc(RamStart, BackupRam, sizeLoopDco);

    if (allowedSteps <= 0)
    {
        return (-1); // Couldn't get DCO working with correct frequency.
    }
    // measurement end
    
    // return measured values
    STREAM_put_word( (DCO<<5) );
    STREAM_put_word( (0x80|BCS1) );
    STREAM_put_word( 0 );
    return 0;
}


short findFllSettings(unsigned short jtagBug)
{
    unsigned short first = 0, last = 27, mid, reload;
    unsigned short BackupRam[30];
    unsigned short RamStart = 0;
    unsigned long  DcoFreq = 0;
      
    // get target RAM start
    if(STREAM_get_word(&RamStart) < 0)
    {
        return(HALERR_EXECUTE_FUNCLET_NO_RAM_START);
    }

    SetPCFunc(ROM_ADDR); //Prevent Ram corruption on F123/F413

    //----------------Backup original Ram content--------------
    ReadRamFunc(RamStart, BackupRam, sizeLoopFll);
       
    // ----------------Download FLL measure funclet------------
    WriteRamFunc(RamStart, loopFll, sizeLoopFll);
   
    // Binary search through the available frequencies selecting the highest frequency < (476KHz - 10%).
    while (first + 1 < last)
    {
         mid = (last + first) / 2;
         reload = 0;
         
        // Select DCO range from 0.23MHz to 11.2MHz. Specify frequency via Ndco. Disable Modulation. Enable DCO+.
        DcoFreq = measureFrequency(RamStart, (mid << 3), 0);
        
        //Ram content will probably be corrupted after each measurement on devices with jtag bug
        //Reupload on every iteration
        if (jtagBug)
        {
            WriteRamFunc(RamStart, loopFll, sizeLoopFll);
        }


        if (DcoFreq == 0)
        {
            break;
        }
        else
        {
            if (DcoFreq > FlashUpperBoarder) // Max. Flash Controller frequency - 10%.
            {
                last = mid;
                reload = 1;
            }
            else
            {
                first = mid;
            }
        }
    }
    if (reload)
    {
          DcoFreq = measureFrequency(RamStart, (first << 3), 0);
    }

    // restore Ram content    
    WriteRamFunc(RamStart, BackupRam, sizeLoopDco);

    if (((DcoFreq < 257000 * 5) || (DcoFreq > 476000 * 5))) 
    {
        return -1;
    }
    // measurement end

    // return measured values
    STREAM_put_word(0x00);
    STREAM_put_word( (first << 3) );
    STREAM_put_word(0x80);
    STREAM_put_word(0x80);
    STREAM_put_word(0);   
    return 0;
}


HAL_FUNCTION(_hal_GetDcoFrequency)
{
    ReadCounterRegsFunc = readCounterRegisters;
    WriteRegFunc = writeRegister;
    SetPCFunc = setPC;
    WriteRamFunc = writeToRam;
    ReadRamFunc = readFromRam;
    SyncFunc = HAL_SyncJtag_Conditional_SaveContext;
    return findDcoSettings(0);
}

HAL_FUNCTION(_hal_GetDcoFrequencyJtag)
{
    ReadCounterRegsFunc = readCounterRegisters;
    WriteRegFunc = writeRegister;
    SetPCFunc = setPCJtag;
    WriteRamFunc = writeToRam;
    ReadRamFunc = readFromRam;
    SyncFunc = HAL_SyncJtag_Conditional_SaveContext;
    return findDcoSettings(1);
}

HAL_FUNCTION(_hal_GetDcoFrequencyX)
{
    ReadCounterRegsFunc = readCounterRegistersX;
    WriteRegFunc = writeRegisterX;
    SetPCFunc = setPCX;
    WriteRamFunc = writeToRamX;
    ReadRamFunc = readFromRamX;
    SyncFunc = HAL_SyncJtag_Conditional_SaveContextX;
    return findDcoSettings(0);
}

HAL_FUNCTION(_hal_GetFllFrequency)
{
    ReadCounterRegsFunc = readCounterRegisters;
    WriteRegFunc = writeRegister;
    SetPCFunc = setPC;
    WriteRamFunc = writeToRam;
    ReadRamFunc = readFromRam;
    SyncFunc = HAL_SyncJtag_Conditional_SaveContext;
    return findFllSettings(0);
}

HAL_FUNCTION(_hal_GetFllFrequencyJtag)
{
    ReadCounterRegsFunc = readCounterRegisters;
    WriteRegFunc = writeRegister;
    SetPCFunc = setPCJtag;
    WriteRamFunc = writeToRam;
    ReadRamFunc = readFromRam;
    SyncFunc = HAL_SyncJtag_Conditional_SaveContext;
    return findFllSettings(1);
}

HAL_FUNCTION(_hal_GetFllFrequencyX)
{
    ReadCounterRegsFunc = readCounterRegistersX;
    WriteRegFunc = writeRegisterX;
    SetPCFunc = setPCX;
    WriteRamFunc = writeToRamX;
    ReadRamFunc = readFromRamX;
    SyncFunc = HAL_SyncJtag_Conditional_SaveContextX;
    return findFllSettings(0);
}
