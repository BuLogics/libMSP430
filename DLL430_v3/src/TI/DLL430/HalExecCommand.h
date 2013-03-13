/*
 * HalExecCommand.h
 *
 * Forwards calls to HalExecBuffered.
 *
 * Copyright (C) 2007 - 2011 Texas Instruments Incorporated - http://www.ti.com/ 
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

#if _MSC_VER > 1000
#pragma once
#endif
#ifndef DLL430_HALEXECCOMMAND_H
#define DLL430_HALEXECCOMMAND_H

#include <HalObjectDb.h>
#include <inttypes.h>
#include <map>

#include "HalExecBuffered.h"

#include "HalExecElement.h"
#include "FetControl.h"

#include <inttypes.h>
#include <vector>
#include <boost/ptr_container/ptr_vector.hpp>



namespace TI
{
	namespace DLL430
	{
		enum HalMsgType {
			UpInit				= 0x51,
			UpErase				= 0x52,
			UpWrite				= 0x53,
			UpRead				= 0x54,
			UpCore				= 0x55,
  
			CmdLegacy			= 0x7E,
			CmdSync				= 0x80,
			CmdExecute			= 0x81,
			CmdExecuteLoop		= 0x82,
			CmdLoad				= 0x83,
			CmdLoadContinued	= 0x84,
			CmdData				= 0x85,
			CmdKill				= 0x86,
			CmdMove				= 0x87,
			CmdUnload			= 0x88,
			CmdBypass			= 0x89,
			CmdExecuteLoopCont  = 0x8A,
			CmdComReset         = 0x8B,
			CmdPauseLoop        = 0x8C,
			CmdResumeLoop	    = 0x8D
		};

		class HalExecCommand
		{
		public:
			HalExecCommand ();
			~HalExecCommand ();

			bool send (FetControl&, IoChannel&);
			void recv (FetControl& , HalResponse& resp, uint8_t type=0);

			void setCallBack(const EventCallback& callback, uint32_t clientHandle);

			uint8_t getResponseId();

			void setAsyncMode(bool continued);
			void setTimeout(uint32_t msec);

			typedef boost::ptr_vector<HalExecElement> list_type;
			list_type elements;
		
		private:
			FetControl* fetControl;
			HalExecBufferedPtr exec;
		};

	};
};

#endif /* DLL430_HALEXECCOMMAND_H */
