/**
* \ingroup MODULBIOS
*
* \file v3_0p.h
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

#ifndef _V3_0P_HEADER_
#define _V3_0P_HEADER_

#include "stream.h"

#define V3OP_LOOP_WAIT_FLAG   0x01
#define V3OP_LOOP_ARRAY_COUNT 4
struct _V3opLoopArray_
{
  unsigned short addr;
  unsigned char  *indata;
  unsigned char  flags;
  unsigned char  msg_id;
  unsigned char  msg_type;
  unsigned char  active;
};
typedef struct _V3opLoopArray_ V3opLoopArray;
typedef HalRec (*HAL_REC_ARRAY)[];

extern V3opLoopArray v3op_loop_array_[V3OP_LOOP_ARRAY_COUNT];
extern HAL_INFOS_PTR hal_infos_;
extern HAL_INFOS no_hal_infos_;
extern HAL_REC_ARRAY hal_ptr_;
extern unsigned short v30p_stream_flags_;
 
// functions
extern short v3opRx (unsigned char *str);
extern short v3opSendException(unsigned char msg_id, unsigned short code, unsigned short *payload);
extern short v3opSetLoop(unsigned char *payload_incl_addr, unsigned char flags);
extern short v3opKillLoop(unsigned char msg_id);
extern short v3opPauseLoop(unsigned char msg_id);
extern short v3opResumeLoop(unsigned char msg_id);
extern void v3opKillAllLoops(void);
#endif
