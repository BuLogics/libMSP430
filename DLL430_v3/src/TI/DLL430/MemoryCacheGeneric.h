/*
 * MemoryCacheGeneric.h
 *
 * Generic implemtation for caching memory types.
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
#ifndef DLL430_MEMORYCACHEGENERIC_H
#define DLL430_MEMORYCACHEGENERIC_H

#include "MemoryCache.h"
#include <inttypes.h>
#include <vector>
#include <map>
#include <boost/shared_ptr.hpp>

namespace TI
{
	namespace DLL430
	{

		class MemoryCacheGeneric : public MemoryCache
		{
		public:
			MemoryCacheGeneric (MemoryArea* parent);

			bool read (uint32_t address, uint32_t* buffer, size_t count);
			bool write (uint32_t address, uint32_t* buffer, size_t count);
			bool write (uint32_t address, uint32_t value);
			bool sync ();
			bool erase ();
			bool erase (uint32_t start, uint32_t end);
			bool verify(uint32_t address, uint32_t* buffer, size_t count);

			bool isReadOnly();

			void enable (bool);
			bool isEnabled () const;
			bool checkCache (uint32_t address) const;
			bool markDirty (uint32_t address, size_t count);
			bool fill (uint32_t address, size_t count);
			bool flush (uint32_t address, size_t count);
			void clear (uint32_t address, size_t count);

		private:
			MemoryArea* parent;
			bool enabled;
			
			bool fillNosync (uint32_t address, size_t count);
			bool syncNeeded;

			struct readElement {
				uint32_t address;
				uint32_t* v_buffer;
				size_t count;
			};
			typedef std::vector<readElement> readElement_list;
			readElement_list readList;
			
			std::vector<uint32_t> cache;
			bool dirty;
			std::vector<uint8_t> flags;
			
			size_t size() const;
			bool checkFlag (uint32_t address, uint8_t flag) const;
			void setFlag (uint32_t address, uint8_t flag, bool set = true);
		};

	};
};

#endif /* DLL430_MEMORYCACHEGENERIC_H */
