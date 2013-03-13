/*
 * RandomMemoryAccess.cpp
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

#include "RandomMemoryAccess.h"
#include "HalExecCommand.h"
#include "DeviceHandleV3.h"

using namespace TI::DLL430;

RandomMemoryAccess::RandomMemoryAccess (				
				const std::string& name,
				DeviceHandleV3* devHandle,
				uint32_t start, 
				uint32_t end, 
				uint32_t seg, 
				uint32_t banks, 
				bool mapped,
				const bool isProtected, 
				MemoryManager* mm,
				uint8_t psa)
 : MemoryAreaBase(name,devHandle,start,end,seg,banks,mapped,isProtected,psa), mm(mm)
{
}

RandomMemoryAccess::~RandomMemoryAccess ()
{
}

bool RandomMemoryAccess::doWrite (uint32_t address, uint32_t* buffer, size_t count)
{
	if (count > this->getSize())
		return false;

	uint32_t start_val = 0;
	const uint32_t end_adr = address + count;
	size_t paddedCount = count;

	if(address & 1)
	{
		++paddedCount;
		if( !doRead(address - 1, &start_val, 1) || !sync() )
			return false;
	}

	uint32_t end_val = 0;
	if(end_adr & 1)
	{
		++paddedCount;
		if( !doRead(end_adr, &end_val, 1) || !sync() )
			return false;
	}

	HalExecElement* el = new HalExecElement(this->devHandle->checkHalId(ID_WriteMemWords));

	address += getStart();

	el->appendInputData32(static_cast<uint32_t>(address & 0xfffffffe));
	el->appendInputData32(static_cast<uint32_t>(paddedCount / 2));

	if(address & 1)
	{
		el->appendInputData8(static_cast<uint8_t>(start_val));
	}
	for (size_t i = 0; i < count; ++i)
	{
		el->appendInputData8(static_cast<uint8_t>(buffer[i]));
	}
	if (end_adr & 1) 
	{
		el->appendInputData16(static_cast<uint8_t>(end_val));
	}

	el->setInputMinSize(8);		// at least address and size (4+4)
	this->elements.push_back(el);

	return true;
}

bool RandomMemoryAccess::writeBytes (uint32_t address, uint32_t* buffer, size_t count)
{
	HalExecElement* el = new HalExecElement(ID_WriteMemBytes);
	el->appendInputData32(this->getStart() + address);
	el->appendInputData32(static_cast<uint32_t>(count));
	for (size_t i = 0; i < count; ++i) 
	{
		if (buffer[i] > 0xFF) 
		{
			delete el;
			return false;
		}
		el->appendInputData8(static_cast<uint8_t>(buffer[i]));
	}
	el->setInputMinSize(9);
	this->elements.push_back(el);
	return true;
}

/** each buffer element contains one _byte_ */
bool RandomMemoryAccess::writeWords (uint32_t address, uint32_t* buffer, size_t count)
{
	if (address & 0x1)
		return false;

	HalExecElement* el = new HalExecElement(this->devHandle->checkHalId(ID_WriteMemWords));
	el->appendInputData32(this->getStart() + address);
	el->appendInputData32(static_cast<uint32_t>(count/2));
	for (size_t i = 0; i+1 < count; i += 2) 
	{
		if (buffer[i] > 0xFF || buffer[i+1] > 0xFF) 
		{
			delete el;
			return false;
		}
		el->appendInputData16(static_cast<uint16_t>(buffer[i] | (buffer[i+1] << 8)));
	}
	el->setInputMinSize(10);
	this->elements.push_back(el);
	return true;
}

bool RandomMemoryAccess::doWrite (uint32_t address, uint32_t value)
{
	return this->writeBytes(address, &value, 1);
}

bool RandomMemoryAccess::postSync (const HalExecCommand& cmd)
{
	for (size_t i = 1; i < cmd.elements.size(); ++i) 
	{
		ReadElement_map::iterator it = this->readMap.find(i-1);
		if (it != this->readMap.end()) 
		{
			ReadElement r = it->second;			
			size_t size = r.size;
			if (r.omitLast)
				--size;

			const HalExecElement& el = cmd.elements.at(i);

			for (size_t i = 0, k = (r.omitFirst? 1: 0); k < size; ++k, ++i)
			{
				r.v_buffer[i] = el.getOutputAt8(k);
			}
			this->readMap.erase(it);
		}
	}
	return true;
}

bool RandomMemoryAccess::isReadOnly ()
{
	return false;
}

