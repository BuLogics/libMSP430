/*
 * MemoryAreaBase.cpp
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

#include "MemoryAreaBase.h"
#include "HalExecCommand.h"
#include "DeviceHandleV3.h"

using namespace TI::DLL430;

MemoryAreaBase::MemoryAreaBase (const std::string& name, 
								DeviceHandleV3* devHandle, 
								uint32_t start, 
								uint32_t size, 
								uint32_t seg, 
								uint32_t banks, 
								bool mapped,
								const bool protectable,
								uint8_t psa)

 : name(name)
 , devHandle(devHandle)
 , start(start)
 , end(start + size - 1) 
 , segmentSize(seg)
 , banks(banks)
 , mapped(mapped)
 , isProtectable(protectable) 
 , locked(isProtectable ? true : false)
 , cache(NULL)
 , psaType(psa)
 , err(MEMORY_NO_ERROR)
{
}

MemoryAreaBase::~MemoryAreaBase ()
{
	if(cache!=NULL)
		delete cache;
}

const char* MemoryAreaBase::getName () const
{
	return this->name.c_str();
}

MemoryError MemoryAreaBase::getError ()
{
	MemoryError error = err;
	err = MEMORY_NO_ERROR;
	return error;
}

bool MemoryAreaBase::isCacheable () const
{
	return cache!=NULL;
}

bool MemoryAreaBase::isReadOnly ()
{
	return false;
}

MemoryCacheCtrl* MemoryAreaBase::getCacheCtrl ()
{
	return cache;
}

bool MemoryAreaBase::addCache(MemoryCacheCtrl * mem_cache)
{
	if(cache!=NULL)
		return false;

	cache=mem_cache;
	return true;
}

uint32_t MemoryAreaBase::getStart () const
{
	return start;
}

uint32_t MemoryAreaBase::getEnd () const
{
	return end;
}

uint32_t MemoryAreaBase::getSize () const
{
	return (this->end - this->start) + 1;
}

uint32_t MemoryAreaBase::getSegmentSize () const
{
	return segmentSize;
}

uint32_t MemoryAreaBase::getBanks () const
{
	return banks;
}

bool MemoryAreaBase::isMapped () const
{
	return mapped;
}

bool MemoryAreaBase::isLocked () const
{
	return isProtectable && locked;
}

//If memory isn't lockable, return true and pretend success
//(otherwise certain IDEs will abort initialization)
bool MemoryAreaBase::lock ()
{
	if(isProtectable)
		locked = true;

	return true;
}

bool MemoryAreaBase::unlock ()
{
	if(isProtectable)
		locked = false;
	
	return true;
}

bool MemoryAreaBase::read (uint32_t address, uint32_t* buffer, size_t count)
{
	err = MEMORY_READ_ERROR;
	bool success = doRead(address, buffer, count);
	if (success)
		err = MEMORY_NO_ERROR;

	return success;
}


MemoryAreaBase::Alignment MemoryAreaBase::alignData(uint32_t address, uint32_t count) const
{
	return Alignment(address, 0, 0);
}


bool MemoryAreaBase::write (uint32_t address, uint32_t* buffer, size_t count)
{
	err = MEMORY_WRITE_ERROR;
	bool success = doWrite(address, buffer, count);
	if (success)
		err = MEMORY_NO_ERROR;

	return success;

}

bool MemoryAreaBase::write (uint32_t address, uint32_t value)
{
	err = MEMORY_WRITE_ERROR;
	bool success = doWrite(address, value);
	if (success)
		err = MEMORY_NO_ERROR;

	return success;

}

bool MemoryAreaBase::doRead (uint32_t address, uint32_t* buffer, size_t count)
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

	HalExecElement* el = new HalExecElement(this->devHandle->checkHalId(ID_ReadMemWords));
	el->appendInputData32(this->getStart() + address);
	el->appendInputData32(static_cast<uint32_t>(count/2));
	el->setOutputSize(count);

	ReadElement r(buffer, count, omitFirst, omitLast, 0);
	this->readMap[this->elements.size()] = r;
	this->elements.push_back(el);
	return true;
}

bool MemoryAreaBase::doWrite (uint32_t address, uint32_t* buffer, size_t count)
{
	return false;
}

bool MemoryAreaBase::doWrite (uint32_t address, uint32_t value)
{
	return false;
}

bool MemoryAreaBase::erase ()
{
	//return true if nothing implemented, means nothing to do
	return true;
}

bool MemoryAreaBase::erase (uint32_t start, uint32_t end)
{
	//return true if nothing implemented, means nothing to do
	return true;
}

bool MemoryAreaBase::verify(uint32_t address, uint32_t* buffer, size_t count)
{
	MemoryCacheCtrl* cc = this->getCacheCtrl(); 
	if (cc && !cc->flush(address,count))
		return false;

	if (address & 0x1) 
	{
		uint32_t tmp = 0;	
		if (!this->read(address++, &tmp, 1) || !this->sync())
			return false;

		if (buffer? tmp != *(buffer++): tmp != 0xFF)
			return false;

		--count;
	}

	if (count > 1) 
	{
		HalExecCommand cmd;
		cmd.setTimeout(20000);		// overwrite default of 3 sec
		
		HalExecElement*el = new HalExecElement(this->devHandle->checkHalId(ID_Psa));
		el->appendInputData32(static_cast<uint32_t>((address+this->getStart()) & 0xFFFFFFFF));
		el->appendInputData32(static_cast<uint32_t>((count / 2) & 0xFFFFFFFF));
		el->appendInputData8(this->psaType);
		el->setOutputSize(2);
		cmd.elements.push_back(el);
		if (!this->devHandle->send(cmd))
			return false;
		
		if (MemoryAreaBase::psa(address+this->getStart(), buffer, (size_t)((uint32_t)count&0xfffffffe)) != el->getOutputAt16(0))
			return false;				
	}

	if (count & 1) 
	{
		uint32_t tmp = 0;
		if (!this->read(address + count - 1, &tmp, 1) || !this->sync())
			return false;

		if (buffer? tmp != buffer[count - 1]: tmp != 0xFF)
			return false;
	}
	return true;
}

bool MemoryAreaBase::sendWithChainInfo(boost::ptr_vector<HalExecElement> * elem, HalExecCommand * cmd)
{	
	HalExecElement* el = new HalExecElement(ID_SetDeviceChainInfo);
	el->appendInputData16(static_cast<uint16_t>(this->devHandle->getDevChainInfo()->getBusId()));
	cmd->elements.push_back(el);
	/* this transfers all object from given elements to cmd.elements */
	cmd->elements.transfer(cmd->elements.end(), *elem);
	if (!this->devHandle->send(*cmd)) 
	{
		this->elements.transfer(elem->end(), cmd->elements);
		return false;
	}
	return true;
}

bool MemoryAreaBase::sendWithChainInfo(HalExecElement * elem, HalExecCommand * cmd)
{
	HalExecElement* el = new HalExecElement(ID_SetDeviceChainInfo);
	el->appendInputData16(static_cast<uint16_t>(this->devHandle->getDevChainInfo()->getBusId()));
	cmd->elements.push_back(el);
	/* this transfers one object to cmd.elements */
	cmd->elements.push_back(elem);
	if (!this->devHandle->send(*cmd)) 
	{
		return false;
	}
	return true;
}

bool MemoryAreaBase::sync ()
{
	if (!preSync())
		return false;

	if (this->elements.empty())
		return true;

	HalExecCommand cmd;
	cmd.setTimeout(60000);

	if (!sendWithChainInfo(&this->elements,&cmd))
		return false;

	return postSync(cmd);
}

uint16_t MemoryAreaBase::psa (uint32_t address, uint32_t* buffer, size_t count)
{
	if (address & 0x1)
		return false;

	if (count & 1)
		return false;

	/* Start value for PSA calculation */
	const uint16_t initial = static_cast<uint16_t>((address-2) & 0xFFFF);
	/* Polynom value for PSA calculation */
	const uint16_t polynom = 0x0805;

	uint16_t remainder = initial;  
	for (size_t i = 0; i < count; i += 2) {
		// Calculate the PSA (Pseudo Signature Analysis) value
		if ((remainder & 0x8000) != 0)
			remainder = ((remainder ^ polynom) << 1) | 0x0001;
		else
			remainder <<= 1;

		// if pointer is 0 then use erase check mask, otherwise data
		if (buffer == 0)
			remainder ^= static_cast<uint16_t>(0xFFFF);
		else
			remainder ^= static_cast<uint16_t>((buffer[i] & 0xFF) | ((buffer[i+1] & 0xFF) << 8));
	}

	return remainder;
}
