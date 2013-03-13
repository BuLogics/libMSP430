/*
 * LockableRamMemoryAccess.cpp
 *
 * Handles access to lockable ram memory.
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

#include "LockableRamMemoryAccess.h"
#include "HalExecCommand.h"
#include "DeviceHandleV3.h"

using namespace TI::DLL430;
using boost::bind;
using boost::shared_ptr;

LockableRamMemoryAccess::LockableRamMemoryAccess
(
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
)
 : MemoryAreaBase(name,devHandle,start,end,seg,banks,mapped, isProtected, psa)
 , mm(mm), unlockBeforeSync(false), lockState(2,0)
{
}

LockableRamMemoryAccess::~LockableRamMemoryAccess ()
{
}

bool LockableRamMemoryAccess::erase(uint32_t start, uint32_t end, uint32_t block_size, int type)
{
	const size_t size = end - start + 1;
	eraseDummyBuffer = vector<uint32_t>(size, 0xFF);

	return write(start - getStart(), &eraseDummyBuffer[0], size) && sync();
}

bool LockableRamMemoryAccess::doRead (uint32_t address, uint32_t* buffer, size_t count)
{
	MemoryArea* cpu = mm->getMemoryArea("CPU");
	if (!cpu)
		return false;

	uint32_t pc = 0;
	cpu->read(0, &pc, 1);

	bool omitFirst = (address & 0x1);
	if (omitFirst) 
	{
		--address;
		++count;
	}
	bool omitLast = (count & 1);
	if (omitLast) 
	{
		++count;
	}	

	const hal_id readMacro = devHandle->supportsQuickMemRead() ? 
								ID_ReadMemQuick : ID_ReadMemWords;

	HalExecElement* el = new HalExecElement(this->devHandle->checkHalId(readMacro));
	el->appendInputData32(this->getStart() + address);
	el->appendInputData32(static_cast<uint32_t>(count/2));
	el->appendInputData32(pc);

	el->setOutputSize(count);

	ReadElement r(buffer, count, omitFirst, omitLast, 0);
	this->readMap[this->elements.size()] = r;
	this->elements.push_back(el);
	return true;
}

bool LockableRamMemoryAccess::doWrite (uint32_t address, uint32_t* buffer, size_t count)
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

	unlockBeforeSync = true;

	return true;
}

bool LockableRamMemoryAccess::doWrite (uint32_t address, uint32_t value)
{
	return this->doWrite(address, &value, 1);
}

bool LockableRamMemoryAccess::erase (uint32_t start, uint32_t end)
{
	return erase(start,end,this->getSegmentSize(),0);
}

bool LockableRamMemoryAccess::erase ()
{
	uint32_t bank_size=this->getSize()/this->getBanks();

	return erase(this->getStart(),this->getEnd(),bank_size,1);
}

bool LockableRamMemoryAccess::preSync ()
{
	bool success = true;

	if ( unlockBeforeSync )
	{
		success = false;

		MemoryArea* peripheral16bit = mm->getMemoryArea("peripheral16bit");
		if (peripheral16bit && 
			peripheral16bit->read(0x0190 - peripheral16bit->getStart(), &lockState[0], 2) && 
			peripheral16bit->sync())
		{	
			uint32_t tmp[2] = { 0, lockState[1] & 0x08 };
			success = peripheral16bit->write(0x0190 - peripheral16bit->getStart(), tmp, 2) && peripheral16bit->sync();
		}
	}	
	return success;
}


bool LockableRamMemoryAccess::postSync (const HalExecCommand& cmd)
{
	if ( unlockBeforeSync )
	{
		MemoryArea* peripheral16bit = mm->getMemoryArea("peripheral16bit");
		peripheral16bit->write(0x0190 - peripheral16bit->getStart(), &lockState[0], 2);
		peripheral16bit->sync();
		unlockBeforeSync = false;
	}

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
