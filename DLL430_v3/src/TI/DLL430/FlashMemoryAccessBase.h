/*
 * FlashMemoryAccessBase.h
 *
 * Handles access to flash memory.
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
#ifndef DLL430_FLASHMEMORYACCESSBASE_H
#define DLL430_FLASHMEMORYACCESSBASE_H

#include "MemoryAreaBase.h"
#include <boost/shared_ptr.hpp>
#include "FuncletCode.h"

namespace TI
{
	namespace DLL430
	{

		class FlashMemoryAccessBase : public MemoryAreaBase
		{
		public:
			FlashMemoryAccessBase (
				const std::string& name,
				DeviceHandleV3* devHandle,
				uint32_t start, 
				uint32_t end, 
				uint32_t seg, 
				uint32_t banks, 
				bool mapped,
				const bool isProtected,
				MemoryManager* mm,
				uint8_t psa
			);
			virtual ~FlashMemoryAccessBase ();

			bool doRead (uint32_t address, uint32_t* buffer , size_t count);
			bool doWrite (uint32_t address, uint32_t* buffer, size_t count);
			bool doWrite (uint32_t address, uint32_t value);

			bool erase ();
			bool erase (uint32_t start, uint32_t end);


		protected:
			bool uploadFunclet(FuncletCode::Type type);
			void restoreRam();
			virtual bool preSync ();
			virtual bool postSync (const HalExecCommand&);			
			virtual Alignment alignData(uint32_t address, uint32_t count) const;

			MemoryManager* mm;
			bool erase (uint32_t start, uint32_t end, uint32_t block_size, int type);
		
		private:
			FuncletCode::Type funcletToUpload;
			std::vector<uint32_t> ramBackup;
		};

		class FlashMemoryAccess2ByteAligned : public FlashMemoryAccessBase
		{
		public:
			FlashMemoryAccess2ByteAligned (
				const std::string& name,
				DeviceHandleV3* devHandle,
				uint32_t start, 
				uint32_t end, 
				uint32_t seg, 
				uint32_t banks, 
				bool mapped,
				const bool isProtected,
				MemoryManager* mm,
				uint8_t psa
			);
		protected:
			virtual Alignment alignData(uint32_t address, uint32_t count) const;
		};
	};
};

#endif /* DLL430_FLASHMEMORYACCESSBASE_H */
