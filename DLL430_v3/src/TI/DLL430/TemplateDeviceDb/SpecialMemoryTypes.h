/*
 * SpecialMemoryTypes.h
 *
 * Memory Types whith special handling. 
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/ 
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

#ifndef TEMPLATE_DEVICE_DB_SPECIAL_MEMORY_TYPES_H
#define TEMPLATE_DEVICE_DB_SPECIAL_MEMORY_TYPES_H

#if _MSC_VER > 1000
#pragma once
#endif

#include <hal.h>

#include <FlashMemoryAccessBase.h>
#include <ReadonlyMemoryAccess.h>
#include <RandomMemoryAccess.h>

namespace TI { namespace DLL430 { 
	class MemoryManager;
	
	namespace TemplateDeviceDb { namespace Memory {
		
	class InformationFlashAccess : public TI::DLL430::FlashMemoryAccessBase
	{
	public:
		InformationFlashAccess (				
					const std::string& name,
					TI::DLL430::DeviceHandleV3* devHandle,
					uint32_t start, 
					uint32_t end, 
					uint32_t seg, 
					uint32_t banks, 
					bool mapped,
					const bool protectable, 
					MemoryManager* mm,
					uint8_t psa)
		: FlashMemoryAccessBase(
			name, devHandle,
			start, end, seg, banks
			, mapped, protectable, mm, psa
		) {}

		virtual unsigned short getFlags() {return isLocked() ? LOCK_INFO_A_FLAG : NO_FLAG;}

		virtual bool erase();
	};

	class BslFlashAccess : public TI::DLL430::FlashMemoryAccessBase
	{
		bool physicallyLocked;
	
		static const unsigned short mySysBslc = 0x182;
		static const unsigned short mySysBslcSize = 2;
	public:
		BslFlashAccess (				
					const std::string& name,
					TI::DLL430::DeviceHandleV3* devHandle,
					uint32_t start, 
					uint32_t end, 
					uint32_t seg, 
					uint32_t banks, 
					bool mapped,
					const bool protectable, 
					MemoryManager* mm,
					uint8_t psa)
		: FlashMemoryAccessBase(
			name, devHandle,
			start, end, seg, banks
			, mapped, protectable, mm, psa
		),
		physicallyLocked(true)
		{}

		virtual bool doRead(uint32_t address, uint32_t* buffer, size_t count);

		virtual bool doWrite(uint32_t address, uint32_t* buffer, size_t count);

		virtual bool doWrite (uint32_t address, uint32_t value);

		virtual bool erase ();

		virtual bool erase (uint32_t start, uint32_t end);

		bool doUnlockBslMemory();

		bool appendInvalidAccessValues(uint32_t address, uint32_t* buffer, size_t count) const;

		bool readBslPe(std::vector<uint32_t>* bslPeBuffer) const;

		bool unlockBslPeAndCheck(uint32_t bslSize);

		bool isDeviceLocked(const std::vector<uint32_t>& bslPeBuffer) const;

		uint32_t getLockedStartAddress() const;
	};

	//actual Bootcode is flash, but behaves like ROM
	class BootcodeRomAccess : public TI::DLL430::ReadonlyMemoryAccess
	{
	public:
		BootcodeRomAccess (				
					const std::string& name,
					TI::DLL430::DeviceHandleV3* devHandle,
					uint32_t start, 
					uint32_t end, 
					uint32_t seg, 
					uint32_t banks, 
					bool mapped,
					const bool protectable,
					MemoryManager* mm,
					uint8_t psa)
		: ReadonlyMemoryAccess(
			name,devHandle,
			start,end,seg,banks,
			mapped,protectable,mm,psa
		)
		{}

		virtual bool doRead (uint32_t address, uint32_t* buffer, size_t count);
		virtual bool doWrite (uint32_t address, uint32_t* buffer, size_t count) { return false; };
		virtual bool doWrite (uint32_t address, uint32_t value) { return false; };
	};
}//namespace Memory
}//namespace TemplateDeviceDb
}//namespace DLL430
}//namespace TI
#endif //TEMPLATE_DEVICE_DB_SPECIAL_MEMORY_TYPES_H

