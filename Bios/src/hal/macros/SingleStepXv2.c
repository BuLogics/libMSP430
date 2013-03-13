/**
* \ingroup MODULMACROSXV2
*
* \file SingleStepXv2.c
*
* \brief Do everything that is needed to perform a SingleStep
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
#include "stddef.h"
#include "error_def.h"

#define CPUOFF 0x10

/**
  SingleStepXv2
  Same description and parameter list as SingleStep.
  Refer to SingleStep for details.
*/
HAL_FUNCTION(_hal_SingleStepXv2)
{
  decl_out
  short RetState = HALERR_UNDEFINED_ERROR;
  unsigned short i = 0;  
  StreamSafe stream_tmp;
  unsigned char MyIn[31] = {
                  /*Number of EEM Data Exchanges*/
                  0x09,
                  /*Read Trigger Block 0*/
                  0x01,0x03,0x05,0x07,0x81,
                  /*Configure Trigger Block 0 for Single Step*/
                  0x02,0x00,0x00, 0x00,0x00,
                  0x04,0xFF,0xFF, 0x0F,0x00,
                  0x06,0x01,0x00, 0x00,0x00,
                  0x80,0x01,0x00, 0x00,0x00,
  };
  unsigned char MyOut[22]; // large enough to hold all EEM registers being read
  unsigned char stream_in_tmp[22];

  for(i=0;i<sizeof(stream_in_tmp);i++)
  {
    STREAM_get_byte(&stream_in_tmp[i]);
  }
  if(!(stream_in_tmp[8] & CPUOFF))  // low byte of SR
  {
    STREAM_internal_stream(MyIn, sizeof(MyIn), MyOut, sizeof(MyOut), &stream_tmp);
    HAL_EemDataExchangeXv2(MESSAGE_NEW_MSG | MESSAGE_LAST_MSG);
    STREAM_external_stream(&stream_tmp);
  }
  STREAM_internal_stream(stream_in_tmp, sizeof(stream_in_tmp), MESSAGE_NO_OUT, 0, &stream_tmp);
  HAL_RestoreContext_ReleaseJtagXv2(MESSAGE_NEW_MSG | MESSAGE_LAST_MSG); // Data from DLL Stream
  STREAM_external_stream(&stream_tmp);

  if(!(stream_in_tmp[8] & CPUOFF))  // low byte of SR
  {
    // poll for CPU stop reaction  
    eem_read_control
    do
    {
      SetReg_16Bits(0x0000)
      i++;
    }
    while(!(lOut & 0x0080) && (i < 500));

    if(i < 500)
    {
      // take target under JTAG control
      STREAM_internal_stream(stream_in_tmp, sizeof(stream_in_tmp), MESSAGE_OUT_TO_DLL, 0, &stream_tmp); 
      RetState = HAL_SyncJtag_Conditional_SaveContextXv2(MESSAGE_NEW_MSG | MESSAGE_LAST_MSG); // In
      STREAM_external_stream(&stream_tmp);
      // restore EEM Trigger Block 0
      MyIn[0]  = 0x05;
      MyIn[1]  = 0x00;
      MyIn[2]  = MyOut[0];
      MyIn[3]  = MyOut[1];
      MyIn[4]  = MyOut[2];
      MyIn[5]  = MyOut[3];
      MyIn[6]  = 0x02;
      MyIn[7]  = MyOut[4];
      MyIn[8]  = MyOut[5];
      MyIn[9]  = MyOut[6];
      MyIn[10] = MyOut[7];
      MyIn[11] = 0x04;
      MyIn[12] = MyOut[8];
      MyIn[13] = MyOut[9];
      MyIn[14] = MyOut[10];
      MyIn[15] = MyOut[11];
      MyIn[16] = 0x06;
      MyIn[17] = MyOut[12];
      MyIn[18] = MyOut[13];
      MyIn[19] = MyOut[14];
      MyIn[20] = MyOut[15];
      MyIn[21] = 0x80;
      MyIn[22] = MyOut[16];
      MyIn[23] = MyOut[17];
      MyIn[24] = MyOut[18];
      MyIn[25] = MyOut[19];
      
      STREAM_internal_stream(MyIn, sizeof(MyIn), MESSAGE_NO_OUT, 0, &stream_tmp);
      HAL_EemDataExchangeXv2(MESSAGE_NEW_MSG | MESSAGE_LAST_MSG);
      STREAM_external_stream(&stream_tmp);
    }
  }  
  else
  {
      // First Check if we did not go into LPMx.5
      lOut = EDT_Instr(IR_CNTRL_SIG_CAPTURE);
      if((lOut == JTAGVERSION) ||
         (lOut == JTAGVERSION8D) ||
         (lOut == JTAGVERSION91) ||
         (lOut == JTAGVERSION95) ||
         (lOut == JTAGVERSION99))
      {
          // take target under JTAG control
          STREAM_internal_stream(stream_in_tmp, sizeof(stream_in_tmp), MESSAGE_OUT_TO_DLL, 0, &stream_tmp); 
          RetState = HAL_SyncJtag_Conditional_SaveContextXv2(MESSAGE_NEW_MSG | MESSAGE_LAST_MSG); // In
          STREAM_external_stream(&stream_tmp);
      }
      else
      {
          // Device stepped into LPMx.5
      }
      RetState = 0;
  }
  return RetState;
}
