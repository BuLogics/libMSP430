/*
 * MemoryManagerV3.cpp
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

#include "MemoryManagerV3.h"
#include "DeviceInfo.h"
#include "MemoryAreaFactory.h"
#include "HalExecCommand.h"
#include "DeviceHandleV3.h"
#include "FetHandleV3.h"

using namespace TI::DLL430;

MemoryManagerV3::MemoryManagerV3 (DeviceHandleV3* parent, const DeviceInfo* devInfo)
 : parent(parent), lastError(MEMORY_NO_ERROR), preserveRam(true)
{
	MemoryAreaFactory fac(parent, devInfo);
	const DeviceInfo::memoryInfo_list_type& infoList = devInfo->getMemoryInfo();
	DeviceInfo::memoryInfo_list_type::const_iterator it = infoList.begin();
	for (; it != infoList.end(); ++it) 
	{
		MemoryAreaBase* area = fac.createMemoryArea(this, *it);
		if (area)
			types.push_back(area);
	}
}

MemoryManagerV3::~MemoryManagerV3 ()
{	
	while(types.size() > 0)
	{
		types.pop_back();
	}
	types.clear();
}

MemoryAreaBase* MemoryManagerV3::getMemoryArea (const char* name, size_t subIndex)
{
	memory_list_type::iterator it = this->types.begin();
	for (; it != this->types.end(); ++it) 
	{
		if (strcmp(it->getName(), name) == 0 && subIndex-- == 0)
		{
			return &(*it);
		}
	}
	return 0;
}

MemoryAreaBase* MemoryManagerV3::getMemoryArea (size_t index)
{
	return index < count() ? &this->types[index] : 0;
}

size_t MemoryManagerV3::count () const
{
	return this->types.size();
}

bool MemoryManagerV3::read (uint32_t address, uint32_t* buffer, size_t count)
{
	memory_list_type::iterator it = this->types.begin();
	
	for (; it != this->types.end(); ++it) 
	{
		if (!it->isMapped())
			continue;

		uint32_t bottom = std::max(address, it->getStart());
		uint32_t top = std::min<uint32_t>(address + count - 1, it->getEnd());
		if (bottom > top)
			continue;

		if (!it->read(bottom - it->getStart(), buffer + (bottom-address), top - bottom + 1))
		{
			lastError = it->getError();
			return false;
		}
	}
	return true;		
}

bool MemoryManagerV3::write (uint32_t address, uint32_t* buffer, size_t count)
{
	memory_list_type::iterator it = this->types.begin();
	for (; it != this->types.end(); ++it) 
	{
		if (!it->isMapped())
			continue;

		uint32_t bottom = std::max(address, it->getStart());
		uint32_t top = std::min<uint32_t>(address + count - 1, it->getEnd());
		
		if (bottom > top)
			continue;

		if (!it->write(bottom - it->getStart(), buffer+(bottom-address), top - bottom + 1))
		{
			lastError = it->getError();
			return false;
		}
	}

	return true;
}

bool MemoryManagerV3::write (uint32_t address, uint32_t value)
{
	memory_list_type::iterator it = this->types.begin();
	for (; it != this->types.end(); ++it) 
	{
		if (!it->isMapped())
			continue;

		if(it->isReadOnly())
			continue;

		if (it->getStart() <= address && address <= it->getEnd())
		{
			bool success = it->write(address - it->getStart(), value);
			if (!success)
				lastError = it->getError();

			return success;
		}
	}
	return true;
}

bool MemoryManagerV3::sync ()
{
	memory_list_type::iterator it = this->types.begin();
	for (; it != this->types.end(); ++it) 
	{
		if (!it->isMapped())
			continue;

		if (!it->sync())
			return false;
	}
	return true;

}

bool MemoryManagerV3::isReadOnly() 
{
	return false;
}

bool MemoryManagerV3::lock(const char* name, bool action)
{
	MemoryAreaBase* mab = getMemoryArea(name);
	if (NULL != mab)
	{
		return action ? mab->lock() : mab->unlock();
	}
	return true;
}

bool MemoryManagerV3::erase ()
{
	MemoryArea* main = this->getMemoryArea("main");
	MemoryArea* info = this->getMemoryArea("information");
	MemoryArea* bsl = this->getMemoryArea("boot");

	if (main && !main->erase())
		return false;

	if (info && !info->erase())
		return false;
	
	if (bsl && !bsl->erase())
		return false;

	return true;
}

bool MemoryManagerV3::erase (uint32_t start, uint32_t end)
{
	memory_list_type::iterator it = this->types.begin();
	for (; it != this->types.end(); ++it) 
	{
		if (!it->isMapped())
			continue;

		if(it->isReadOnly())
			continue;

		uint32_t bottom = std::max(start, it->getStart());
		uint32_t top = std::min(end, it->getEnd());
		if (bottom > top)
			continue;

		if (!it->erase(bottom, top))
			return false;
	}
	return true;
}

bool MemoryManagerV3::verify(uint32_t address, uint32_t* buffer, size_t count)
{
	memory_list_type::iterator it = this->types.begin();
	for (; it != this->types.end(); ++it) 
	{
		if (!it->isMapped())
			continue;

		if(it->isReadOnly())
			continue;

		uint32_t bottom = std::max(address, it->getStart());
		uint32_t top = std::min<uint32_t>(address + count - 1, it->getEnd());
		if (bottom > top)
			continue;

		if (!it->verify(bottom - it->getStart(), buffer+(bottom-address), top - bottom + 1))
			return false;
	}
	return true;
}

bool MemoryManagerV3::flushAll()
{
	for (unsigned int i = 0; i < this->count(); ++i) 
	{
		MemoryArea* area = this->getMemoryArea(i);
		MemoryCacheCtrl* ctrl = area->getCacheCtrl();
		if (ctrl) 
		{
			if (!ctrl->flush(0, area->getSize()))
				return false;

			ctrl->clear(0, area->getSize());
		}
	}
	return true;
}

MemoryError MemoryManagerV3::getLastError()
{
	MemoryError error = lastError;
	lastError = MEMORY_NO_ERROR;
	return error;
}

void MemoryManagerV3::setRamPreserveMode(bool enabled)
{
	preserveRam = enabled;
}

bool MemoryManagerV3::getRamPreserveMode() const
{
	return preserveRam;
}

bool MemoryManagerV3::uploadFunclet(FuncletCode::Type type)
{
	const FuncletCode& funclet = parent->getFunclet(type);
	const uint8_t* code = (uint8_t*)funclet.code();
	const size_t count = funclet.codeSize();

	vector<uint32_t> tmp(code, code + count); // copy funclet into vector
	MemoryArea* ram = this->getMemoryArea("system", 0);

	return ram && ram->write(0, &tmp[0], count) && ram->sync();
}

bool MemoryManagerV3::checkMinFlashVoltage() const
{
	const unsigned minFlashVcc = parent->getMinFlashVcc();

	if ( FetHandle* fetHandle = parent->getFetHandle() )
	{
		if ( const ConfigManager* configManager = fetHandle->getConfigManager() )
		{
			return configManager->getDeviceVcc() >= minFlashVcc ||
				   configManager->getExternalVcc() >= minFlashVcc;
		}
	}
	return false;
}
