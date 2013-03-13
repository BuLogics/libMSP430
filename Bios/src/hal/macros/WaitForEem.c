/**
* \ingroup MODULMACROS
*
* \file WaitForEem.c
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
#include "stream.h"
#include "error_def.h"

/**
  WaitForEem
  Test the EEM General Debug Control (GENCTRL) register for specific bits and
  return if one of them is set.
  This is best used with the ExecLoop command type.
  inData:  <eemCtrlMask(16)>
  outData: <eemCtrl(16)>
          eemCtrlMask: the bits to look for
          eemCtrl: the read GENCTRL EEM register
*/

HAL_FUNCTION(_hal_WaitForEem)
{
  decl_out

  short RetState = HALERR_UNDEFINED_ERROR;
  unsigned long i = 0;
  unsigned long lMask = 0;
  unsigned short sGenCtrl = 0;
  unsigned short eventMask = 0;

  STREAM_get_word((unsigned short*)&lMask);
  
  lOut = EDT_Instr(IR_CNTRL_SIG_CAPTURE);
  if(lOut != JTAGVERSION && lOut != JTAGVERSION91 && lOut != JTAGVERSION95 && lOut != JTAGVERSION99)
  {
    return 2;  
  }
  
  
  // poll for bits in EEM GENCTRL register
  eem_read_control
  do
  {
    SetReg_16Bits(0x0000)
    i++;
  }
  while(!(lOut & lMask) && i < 10);
  if(i < 10)
  {
    sGenCtrl = lOut;
    
    eventMask |= BP_HIT_FLAG;
    RetState = 1;
     
    STREAM_put_word(eventMask);
    STREAM_put_word(sGenCtrl);    
  }
  else
  {
    RetState = 2;
  }
  
  return RetState;
}
