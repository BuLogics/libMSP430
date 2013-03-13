/*
 * CpuMemoryAccess.h
 *
 * Implementaion for access of CPU registers.
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
#ifndef DLL430_CPUMEMORYACCESS_H
#define DLL430_CPUMEMORYACCESS_H

#include "MemoryCache.h"
#include "MemoryAreaBase.h"
#include <vector>

namespace TI
{
	namespace DLL430
	{
		class DeviceHandleV3;
		class CpuMemoryAccess : public MemoryCache, public MemoryAreaBase
		{
		public:
			CpuMemoryAccess (const std::string& name, DeviceHandleV3* devHandle, 
				uint32_t start, 
				uint32_t size, 
				uint32_t seg, 
				uint32_t banks, 
				bool mapped,
				uint8_t bits);
			~CpuMemoryAccess ();

			bool read  (uint32_t Register, uint32_t* buffer, size_t count);
			bool write (uint32_t Register, uint32_t* buffer, size_t count);
			bool write (uint32_t Register, uint32_t value);

			void enable (bool) { /* cannot be disabled */ };
			bool isEnabled () const { return true; };
			bool checkCache (uint32_t Register) const;
			bool markDirty (uint32_t Register, size_t count);
			bool fill (uint32_t Register, size_t count);
			bool flush (uint32_t Register, size_t count);
			void clear (uint32_t Register, size_t count);
			bool isReadOnly();
			MemoryCacheCtrl *getCacheCtrl() {return this;};

		private:
			uint8_t bytes;

			typedef uint32_t cpuType;
			uint16_t fillStatus;
			uint16_t changeStatus;
			std::vector<cpuType> localCache;
		};

	};
};

#endif /* DLL430_CPUMEMORYACCESS_H */
