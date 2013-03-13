/*
 * MemoryManagerV3.h
 *
 * Manages to which MemoryAreaBases to communicate dependent on address.
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
#ifndef DLL430_MEMORYMANAGERV3_H
#define DLL430_MEMORYMANAGERV3_H

#include "MemoryAreaBase.h"
#include "FuncletCode.h"

#include <DLL430/MemoryManager.h>
#include <boost/ptr_container/ptr_vector.hpp>


namespace TI
{
	namespace DLL430
	{
		class DeviceHandleV3;
		class DeviceInfo;

		class MemoryManagerV3 : public MemoryManager
		{
		public:
			MemoryManagerV3 (DeviceHandleV3*, const DeviceInfo*);
			~MemoryManagerV3 ();

			MemoryAreaBase* getMemoryArea (const char* name, size_t subIndex = 0);
			MemoryAreaBase* getMemoryArea (size_t index);
			size_t count () const;

			bool read (uint32_t address, uint32_t* buffer, size_t count);
			bool write (uint32_t address, uint32_t* buffer, size_t count);
			bool write (uint32_t address, uint32_t value);
			bool sync ();
			bool erase ();
			bool erase (uint32_t start, uint32_t end);
			bool verify(uint32_t address, uint32_t* buffer, size_t count);

			bool isReadOnly();
			bool lock(const char* name, bool action);

			bool flushAll();
			MemoryError getLastError();

			void setRamPreserveMode(bool enabled);
			bool getRamPreserveMode() const;
			bool uploadFunclet(FuncletCode::Type type);
			bool checkMinFlashVoltage() const;

		private:
			typedef boost::ptr_vector<MemoryAreaBase> memory_list_type;
		
			DeviceHandleV3* parent;
			memory_list_type types;
			MemoryError lastError;
			bool preserveRam;
		};

	};
};

#endif /* DLL430_MEMORYMANAGERV3_H */
