/*
 * MemoryAreaBase.h
 *
 * Base class for all memory classes.
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
#ifndef DLL430_MEMORYAREABASE_H
#define DLL430_MEMORYAREABASE_H

#include <DLL430/MemoryManager.h>
#include "FetControl.h"
#include "MemoryCache.h"
#include <boost/ptr_container/ptr_vector.hpp>
#include <string>

namespace TI
{
	namespace DLL430
	{
		class DeviceHandleV3;
		class MemoryAreaBase : public MemoryArea
		{
		public:
			MemoryAreaBase (const std::string&, DeviceHandleV3* devHandle, 
							uint32_t start, uint32_t end, 
							uint32_t seg, uint32_t banks, 
							bool mapped, const bool protectable, 
							uint8_t psa_type);			
			virtual ~MemoryAreaBase ();

			const char* getName () const;
			MemoryError getError ();
			bool isCacheable () const;
			virtual bool isReadOnly ();
			virtual MemoryCacheCtrl* getCacheCtrl ();

			uint32_t getStart () const;
			uint32_t getEnd () const;
			uint32_t getSize () const;
			uint32_t getSegmentSize () const;
			uint32_t getBanks () const;
			bool isMapped () const;
			bool isLocked () const;
			bool lock ();
			bool unlock ();

			bool read (uint32_t address, uint32_t* buffer, size_t count);
			bool write (uint32_t address, uint32_t* buffer, size_t count);
			bool write (uint32_t address, uint32_t value);

			virtual bool doRead (uint32_t address, uint32_t* buffer, size_t count);
			virtual bool doWrite (uint32_t address, uint32_t* buffer, size_t count);
			virtual bool doWrite (uint32_t address, uint32_t value);

			virtual bool sync ();

			bool sendWithChainInfo(HalExecElement * elem, HalExecCommand * cmd);
			bool sendWithChainInfo(boost::ptr_vector<HalExecElement> * elem, HalExecCommand * cmd);

			virtual bool erase ();
			bool erase (uint32_t start, uint32_t end);
			bool verify(uint32_t address, uint32_t* buffer, size_t count);
			static uint16_t psa (uint32_t address, uint32_t* buffer, size_t count);

			bool addCache(MemoryCacheCtrl *);

		protected:
			struct Alignment
			{
				Alignment(uint32_t alignedAddress, int frontPadding, int backPadding)
					: alignedAddress(alignedAddress), frontPadding(frontPadding), backPadding(backPadding) {}

				const uint32_t alignedAddress;
				const int frontPadding;
				const int backPadding;
			};

			virtual Alignment alignData(uint32_t address, uint32_t count) const;

			virtual bool preSync () { return true; };
			virtual bool postSync (const HalExecCommand&) { return true; };

			virtual unsigned short getFlags() {return 0;}
			std::string name;
			DeviceHandleV3* devHandle;
			boost::ptr_vector<HalExecElement> elements;
			
			struct ReadElement 
			{
				ReadElement() : v_buffer(0), size(0), omitFirst(false), omitLast(false), offset(0) {}

				ReadElement(uint32_t* buffer, size_t size, bool omitFirst, bool omitLast, size_t offset) 
					: v_buffer(buffer), size(size), omitFirst(omitFirst), omitLast(omitLast), offset(offset) {}

				uint32_t* v_buffer;
				size_t size;
			
				bool omitFirst;
				bool omitLast;
				size_t offset;
			};
			typedef std::map<size_t, ReadElement> ReadElement_map;
			ReadElement_map readMap;
			uint8_t psaType;
			MemoryError err;

		private:
			uint32_t start;
			uint32_t end;
			uint32_t segmentSize;
			uint32_t banks;
			bool mapped;
			const bool isProtectable;
			bool locked;
			MemoryCacheCtrl * cache;
		};

		typedef MemoryAreaBase UncachedMemoryArea;
	};
};

#endif /* DLL430_MEMORYAREABASE_H */
