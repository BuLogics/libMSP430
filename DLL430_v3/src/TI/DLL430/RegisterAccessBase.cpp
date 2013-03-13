/*
 * RegisterAccessBase.cpp
 *
 * Base class for all memory classes which access some type of register.
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

#include "RegisterAccessBase.h"
#include <HalObjectDb.h>
#include "HalExecCommand.h"
#include "DeviceHandleV3.h"

using namespace TI::DLL430;

RegisterAccess::RegisterAccess (
	const std::string& name,
	DeviceHandleV3* devHandle,
	uint32_t start, 
	uint32_t end, 
	uint32_t seg, 
	uint32_t banks, 
	bool mapped,
	uint8_t bits,
	const std::vector<uint8_t>& mask
)
 : MemoryAreaBase(name,devHandle,start,end,seg,banks,mapped,false/*isProtected*/ ,0xff)
 , bits(bits), mask(mask)
{
}


bool RegisterAccess::doRead(uint32_t address, uint32_t* buffer, size_t count)
{
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

	const hal_id readMacro = (bits == 8) ? ID_ReadMemBytes : ID_ReadMemWords;

	HalExecElement* el = new HalExecElement(this->devHandle->checkHalId(readMacro));
	el->appendInputData32(this->getStart() + address);
	el->appendInputData32(static_cast<uint32_t>(count/2));
	el->setOutputSize(count);

	ReadElement r(buffer, count, omitFirst, omitLast, address);
	this->readMap[this->elements.size()] = r;
	this->elements.push_back(el);

	//Patch in the watchdog value the device would have while running
	const int wdtOffset = devHandle->getWatchdogControl()->getAddress() - getStart() - address;
	
	if ( wdtOffset >= 0 && wdtOffset < (int)count )
	{
		//Force sync immediately, instead of trying to patch watchdog in postSync
		if (!sync())
			return false;

		buffer[wdtOffset] = devHandle->getWatchdogControl()->get() & 0xFF;
	}
	return true;
}

bool RegisterAccess::doWrite(uint32_t address, uint32_t value)
{
	const size_t numBytes = bits / 8;
	const uint8_t *p = reinterpret_cast<uint8_t*>(&value);
	uint32_t buffer[4] = { p[0], p[1], p[2], p[3] };
	
	return this->doWrite(address, buffer, numBytes);
}

/** each buffer element contains one _byte_ */
bool RegisterAccess::doWrite(uint32_t address, uint32_t* buffer, size_t count)
{
	//If watchdog is written, set password and stop bit
	//and store value the watchdog will have while running
	const uint8_t WDT_STOP_BIT = 0x80;
	const uint8_t WDT_PW = 0x5A;

	const int wdtOffset = devHandle->getWatchdogControl()->getAddress() - getStart() - address;
	
	if ( wdtOffset >= 0 && wdtOffset < (int)count )
	{
		devHandle->getWatchdogControl()->set( buffer[wdtOffset] );
		buffer[wdtOffset] |= WDT_STOP_BIT;
		
		if (wdtOffset + 1 < (int)count)
			buffer[wdtOffset+1] = WDT_PW;
	}


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

	const hal_id writeMacro = (bits == 8) ? ID_WriteMemBytes : ID_WriteMemWords;

	HalExecElement* el = new HalExecElement(devHandle->checkHalId(writeMacro));
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

bool RegisterAccess::postSync (const HalExecCommand& cmd)
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
				const uint8_t bitMask = (r.offset + i < mask.size()) ? mask[r.offset + i] : 0xff;
				r.v_buffer[i] = el.getOutputAt8(k) & bitMask;
			}
			this->readMap.erase(it);
		}
	}
	return true;
}
