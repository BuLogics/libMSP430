/*
 * UsbIoChannel.cpp
 *
 * Base class for USB IOChannel devices, e.g. CDC or HID.
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

#include "UsbIoChannel.h"
#include "HalExecCommand.h"
#include "HalExecElement.h"
#include "FetControl.h"
#include <HalObjectDb.h>
#include <assert.h>
#include <vector>

#include <stdio.h>

using namespace TI::DLL430;


bool UsbIoChannel::setObjectDbEntry (size_t entry)
{
	struct halObjectDb_entry* dbEntry = NULL;
	for (unsigned long i = 0; halObjectDb[i].entry != NULL; ++i) 
	{
		if (i == entry)
			dbEntry = halObjectDb[i].entry;
	};
	if (dbEntry == NULL)
		return false;

	if (this->available.size() > 0)
		this->available.clear();

	/* This is a special function that - by definition -\
	 * is always present
	 */
	unsigned long zeroId = 0;
	UsbIoChannel::FetFunction* f = new UsbIoChannel::FetFunction;

	f->callAddress = 0x0000;
	f->object = 0;
	f->sticky = false;
	this->available.insert(zeroId, f);

	//get version and callAdress size from firmware
	uint16_t fwVersion = 0;
	HalExecCommand zeroIdCmd;
	HalExecElement* el = new HalExecElement(ID_Zero);

	el->appendInputData8(STREAM_CORE_ZERO_MACRO_SIZE);		// destination: core or macros
	el->setOutputSize(2);

	zeroIdCmd.elements.push_back(el);
	if (!this->parent->send(zeroIdCmd))
		return false;

	uint16_t entries = el->getOutputAt16(0);
	zeroIdCmd.elements.clear();
	el = new HalExecElement(ID_Zero);
	el->appendInputData8(STREAM_CORE_ZERO_MACRO_ADDR);

	el->appendInputData16(entries);
	el->setInputMinSize(4);
	el->setOutputSize(4 + (entries * 4));

	zeroIdCmd.elements.push_back(el);
	if (!this->parent->send(zeroIdCmd))
		return false;

	std::map<uint16_t, uint16_t> builtinMap;
	for (uint16_t i = 1; i <= entries; ++i) 
	{
		builtinMap[el->getOutputAt16(i * 4)] = el->getOutputAt16((i * 4) + 2) | 0x1;
	}

	for (unsigned int i = 0; dbEntry[i].id != 0; ++i) 
	{
		UsbIoChannel::FetFunction* f = new UsbIoChannel::FetFunction;
		std::map<uint16_t, uint16_t>::iterator bit = builtinMap.find(dbEntry[i].id);
		if (bit == builtinMap.end() || fwVersion < dbEntry[i].MSP430.version) 
		{
			// If function is not yet implemented in BIOS or current function in DB
			// is newer than in BIOS. Mark function in order to download it later
			f->callAddress = (uint16_t)-1;	
			f->object = dbEntry[i].MSP430.object;
		} 
		else 
		{
			// Function is already implemented in BIOS and has the same version.
			f->callAddress = bit->second;
			f->object = 0;
		}
		f->functionId = dbEntry[i].id;
		f->sticky = false;
		unsigned long id = f->functionId;
		this->available.insert(id, f);
	}
	
	return true;
}

size_t UsbIoChannel::getFunctionAddress (unsigned long id)
{
	if(id==0)
		return 0;

	if (this->available.size() == 0)
		this->setObjectDbEntry(0);

	uint16_t halId = static_cast<uint16_t>(id & 0xFFFF);
	if (this->functionTable)
	{
		function_map_type::iterator fmit = this->functionTable->find(id);
		if (fmit != this->functionTable->end())
			halId = fmit->second;
	}

	UsbIoChannel::object_map_type::iterator it = this->available.find(halId);
	if (it == this->available.end())
		return (size_t)-1;
	UsbIoChannel::FetFunction* f = it->second;
	if (id > 0 && f->callAddress == (uint16_t)-1)
		return (size_t)-1;

	return id;
}

uint16_t UsbIoChannel::createCrc(const uint8_t * buf)
{
	uint16_t crc=0x0000;
	uint8_t count=((buf[0]+1)>>1);
	if(!(buf[0]&0x01)){
		count++;
	}

	uint16_t short_val;

	for(int i=0;i<count;i++)
	{
		short_val=(buf[(i*2)+1]<<8)+buf[i*2];
		crc^=short_val;
	}
	uint16_t x=crc;
	crc ^= 0xFFFF;

	uint16_t test=x^crc;
	return crc;
}

uint16_t UsbIoChannel::createCrc(const uint8_t * buf, const uint8_t size)
{
	uint8_t count=(size+1) /2;
	
	if(size % 2 == 0){
		++count;
	}

	uint16_t crc=(buf[0]<<8)+size;
	for(int i=1; i < count; i++)
	{
		crc^=(buf[i*2]<<8)+buf[i*2-1];
	}

	return crc ^ 0xFFFF;
}

