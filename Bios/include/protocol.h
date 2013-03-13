/**
* \ingroup MODULBIOS
*
* \file protocol.h
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
 @page protocol Host (PC) - Bios Communication Protocol
 This page introduces the user to the topic.
 Proceed to the @ref hil.  
*/

/**
 @file protocol.h
 @brief Host (PC) - Bios Communication Protocol

 This file defines and exports all the objects and symbols required to use
 the Host (PC) - Bios Communication Protocol.
*/

#ifndef _BIOS_PROTOCOL_H_
#define _BIOS_PROTOCOL_H_

enum eCMDTYP
{
  CMDTYP_UPINIT         = 0x51,
  CMDTYP_UPERASE        = 0x52,
  CMDTYP_UPWRITE        = 0x53,
  CMDTYP_UPREAD         = 0x54,
  CMDTYP_UPCORE         = 0x55,
  
  CMDTYP_LEGACY         = 0x7E,
  CMDTYP_SYNC           = 0x80,
  CMDTYP_EXECUTE        = 0x81,
  CMDTYP_EXECUTELOOP    = 0x82,
  CMDTYP_LOAD           = 0x83,
  CMDTYP_LOAD_CONTINUED = 0x84,
  CMDTYP_DATA           = 0x85,
  CMDTYP_KILL           = 0x86,
  CMDTYP_MOVE           = 0x87,
  CMDTYP_UNLOAD         = 0x88,
  CMDTYP_BYPASS         = 0x89,
  CMDTYP_EXECUTEEVER    = 0x8A,
  CMDTYP_COMRESET       = 0x8B,
  CMDTYP_PAUSE_LOOP     = 0x8C,
  CMDTYP_RESUME_LOOP    = 0x8D,
};
typedef enum eCMDTYP eCMDTYP_t;


enum eRESPTYP
{
  RESPTYP_EMPTY          = 0x00,
  RESPTYP_ACKNOWLEDGE    = 0x91,
  RESPTYP_EXCEPTION      = 0x92,
  RESPTYP_DATA           = 0x93,
  RESPTYP_REQUEST        = 0x94,
  RESPTYP_STATUS         = 0x95,
};

typedef enum eRESPTYP eRESPTYP_t;

#endif
