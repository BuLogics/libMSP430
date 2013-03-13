/*
 * RandomMemoryAccess.h
 *
 * Memory class for accessing RAM.
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
#ifndef DLL430_RANDOMMEMORYACCESS_H
#define DLL430_RANDOMMEMORYACCESS_H



#include "MemoryAreaBase.h"
#include <boost/shared_ptr.hpp>

namespace TI
{
	namespace DLL430
	{
		class DeviceHandleV3;
		class RandomMemoryAccess : public MemoryAreaBase
		{
		public:
			RandomMemoryAccess (				
				const std::string& name,
				DeviceHandleV3* devHandle,
				uint32_t start, 
				uint32_t end, 
				uint32_t seg, 
				uint32_t banks, 
				bool mapped,
				const bool isProtected,
				MemoryManager* mm,
				uint8_t psa);
			virtual ~RandomMemoryAccess ();

			virtual bool doWrite (uint32_t address, uint32_t* buffer, size_t count);
			virtual bool doWrite (uint32_t address, uint32_t value);

			virtual bool isReadOnly();
			
			virtual bool erase () { return true; }
			bool erase (uint32_t start, uint32_t end) { return true; }
		
		protected:
			bool postSync (const HalExecCommand&);

			MemoryManager* mm;

		private:
			bool writeBytes (uint32_t address, uint32_t* buffer, size_t count);
			bool writeWords (uint32_t address, uint32_t* buffer, size_t count);
		};

	};
};

#endif /* DLL430_RANDOMMEMORYACCESS_H */
