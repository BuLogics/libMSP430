/*
 * EemMemoryAccess.h
 *
 * EEM Memory Access for 16bit targets.
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
#ifndef DLL430_EEMMEMORYACCESS_H
#define DLL430_EEMMEMORYACCESS_H

#include "MemoryAreaBase.h"
#include "EemTriggerAccess.h"
#include <vector>


namespace TI
{
	namespace DLL430
	{
		enum EemRegister
		{
			// general control registers
			BREAK_REACT		= 0x80,		// CPU stop reaction register
			GENCTRL			= 0x82,		// general debug control register
			EEMVER			= 0x86,		// Emulation Module Version
			GENCLKCTRL		= 0x88,		// general clock control register
			MODCLKCTRL0		= 0x8A,		// clock module control register
			TRIGFLAG		= 0x8E,		// debug trigger register
			// state storage
			STOR_REACT		= 0x98,		// reaction register: state storage
			STOR_ADDR		= 0x9a,		// state storage memory address
			STOR_DATA		= 0x9c,		// state storage memory data
			STOR_CTL		= 0x9e,		// state storage control register
			// sequencer
			SEQ_NXTSTATE0	= 0xa0,		// sequencer next state 0 register
			SEQ_NXTSTATE1	= 0xa2,		// sequencer next state 1 register
			SEQ_CTRL		= 0xa6,		// sequencer control register
			// cycle counter
			CCNT0CTL	    = 0xb0,		// cycle counter 0 control register
			CCNT0L			= 0xb2,		// cycle counter 0 value low
			CCNT0H			= 0xb4,		// cycle counter 0 value high
			CCNT1CTL	    = 0xb8,		// cycle counter 1 control register
		};

		class EemMemoryAccess : public MemoryAreaBase
		{
		public:
			EemMemoryAccess (const std::string& name, DeviceHandleV3* devHandle, 
					uint32_t start, 
					uint32_t size, 
					uint32_t seg, 
					uint32_t banks, 
					bool mapped,
					uint8_t bits);
			~EemMemoryAccess ();

			bool doRead (uint32_t address, uint32_t* buffer, size_t count);
			bool doWrite (uint32_t address, uint32_t* buffer, size_t count);
			bool doWrite (uint32_t address, uint32_t value);

			// write given value to eem register reg
			bool writeEemRegister(EemRegister reg, uint32_t value);
			// read value of eem register reg
			bool readEemRegister(EemRegister reg, uint32_t* buffer);			

		private:
			const size_t maxAddress;
			uint8_t words;

			std::vector<uint8_t> queue;
			uint8_t queueCount;
			uint32_t* readPtr;
			size_t nsent;
			
			bool preSync ();
			bool postSync (const HalExecCommand&);
		};

	};
};

#endif /* DLL430_EEMMEMORYACCESS_H */
