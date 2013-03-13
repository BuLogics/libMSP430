/*
 * IoChannel.h 
 *
 * Interface for IoChannel derived classes.
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
#ifndef DLL430_IOCHANNEL_H
#define DLL430_IOCHANNEL_H

#include <DLL430/PortInfo.h>

#include <inttypes.h>
#include <map>
#include <vector>

#define STREAM_CORE_ZERO_VERSION    0x00
#define STREAM_CORE_ZERO_MACRO_SIZE 0x01
#define STREAM_CORE_ZERO_MACRO_ADDR 0x02
#define STREAM_CORE_ZERO_PUC_RESET  0x03

namespace TI
{
	namespace DLL430
	{

		class HalResponse;
		class FetControl;

		enum ComState {
			ComStateRcv = 0,
			ComStateEnd = 1,
			ComStateError = 2,
			ComStateDisconnect = 3
		};

		class IoChannel : public PortInfo
		{
			IoChannel(const IoChannel&);
			void operator=(const IoChannel&);
		public:
			IoChannel ();
			virtual ~IoChannel ();
			void setParent(FetControl*);
		
			virtual enum ComState poll () = 0;
			virtual int read (HalResponse& resp, uint32_t timeout) = 0;

			virtual int write (const uint8_t* payload, size_t len) = 0;

			virtual bool setObjectDbEntry (size_t entry) = 0;
			virtual size_t getFunctionAddress (unsigned long id) = 0;

			typedef std::map<unsigned long, uint16_t> function_map_type;
			void setFunctionMap (function_map_type* functionTable) { this->functionTable = functionTable; };

			virtual const char* getName () =0;
			virtual const std::string getSerial() =0;

		protected:
			FetControl* parent;
			function_map_type* functionTable;
		};
	};
};

#endif /* DLL430_IOCHANNEL_H */
