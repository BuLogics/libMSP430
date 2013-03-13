/*
 * HalExecElement.cpp
 *
 * Buffer for sending.
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

#include "HalExecElement.h"


#include <stdio.h>

using namespace TI::DLL430;

HalExecElement::HalExecElement (uint16_t functionId, uint8_t msgType)
 : functionId(functionId)
 , msgType(msgType)
 , inMinSize(0)
 , outSize(0)
 , hasAddr(true)
{
}

HalExecElement::~HalExecElement ()
{
}

void HalExecElement::appendInputData32 (uint32_t data)
{
	this->appendInputData16(static_cast<uint16_t>(data & 0xFFFF));
	this->appendInputData16(static_cast<uint16_t>((data >> 16) & 0xFFFF));
}

void HalExecElement::appendInputData32 (uint32_t* data, size_t len)
{
	for (size_t i = 0; i < len; ++i)
		appendInputData32(data[i]);

	this->inMinSize += len;
}

void HalExecElement::appendInputData16 (uint16_t data)
{
	this->inData.push_back(static_cast<uint8_t>(data & 0xFF));	
	this->inData.push_back(static_cast<uint8_t>((data >> 8) & 0xFF));
	this->inMinSize += sizeof(data);
}

void HalExecElement::appendInputData16 (uint16_t* data, size_t len)
{
	for (size_t i = 0; i < len; ++i)
		appendInputData16(data[i]);
}

void HalExecElement::appendInputData16 (const std::vector<uint16_t>::const_iterator& begin, const std::vector<uint16_t>::const_iterator& end)
{
	std::vector<uint16_t>::const_iterator it = begin;
	for (; it < end; ++it) 
	{
		this->appendInputData16(*it);
	}
}

void HalExecElement::appendInputData8 (uint8_t data)
{
	this->inData.push_back(data);
	this->inMinSize += sizeof(data);
}

void HalExecElement::appendInputData8 (uint8_t* data, size_t len)
{
	for (size_t i = 0; i < len; ++i) 
	{
		this->inData.push_back(data[i]);
	}
	this->inMinSize += len;
}

void HalExecElement::appendInputData8 (const std::vector<uint8_t>::const_iterator& begin, const std::vector<uint8_t>::const_iterator& end)
{
	this->inData.insert(this->inData.end(), begin, end);
	this->inMinSize += end-begin;
}

void HalExecElement::setInputMinSize (size_t s)
{
	this->inMinSize = s;
}

uint16_t HalExecElement::getFunctionId() const
{
	return this->functionId;
}

void HalExecElement::setOutputSize (size_t len)
{
	this->outSize = len;
	this->outData.reserve(len);
}

const std::vector<uint8_t>& HalExecElement::getOutput () const
{
	return this->outData;
}

uint32_t HalExecElement::getOutputAt32 (size_t pos) const
{
	return (getOutputAt16(pos) | (getOutputAt16(pos+2) << 16));
}

uint16_t HalExecElement::getOutputAt16 (size_t pos) const
{
	return (getOutputAt8(pos) | (getOutputAt8(pos+1) << 8));
}

uint8_t HalExecElement::getOutputAt8 (size_t pos) const
{
	if (pos < this->outData.size())
		return this->outData[pos];

	return 0;
}

void HalExecElement::setAddrFlag(bool flag)
{
	hasAddr=flag;
}

bool HalExecElement::getAddrFlag()
{
	return hasAddr;
}

bool HalExecElement::addTransaction(uint8_t id)
{
	boost::mutex::scoped_lock lock(respRefMutex);
	respRef[id]=0x00;
	return true;
}

bool HalExecElement::remTransaction(uint8_t id)
{
	boost::mutex::scoped_lock lock(respRefMutex);
	respRef.erase(id);
	return true;
}

uint8_t HalExecElement::checkTransaction(uint8_t id, uint8_t mask)
{
	boost::mutex::scoped_lock lock(respRefMutex);
	std::map<uint8_t, uint8_t>::iterator id_it;
	id_it = this->respRef.find(id);
	if (id_it == this->respRef.end())
	{
#ifndef NDEBUG
		printf("checkTransaction: not found\n");
#endif
		return 0;
	}
	return id_it->second & mask;
}

uint8_t HalExecElement::changeTransaction(uint8_t id, uint8_t mask, bool set)
{
	boost::mutex::scoped_lock lock(respRefMutex);
	std::map<uint8_t, uint8_t>::iterator id_it;
	id_it = this->respRef.find(id);
	if (id_it == this->respRef.end())
	{
#ifndef NDEBUG
		printf("HalExecElement::changeTransaction: %i not found\n",id);
#endif
		return 0;
	}

	if(set)
		return id_it->second |= mask;
	else
		return id_it->second &= (mask^0xff);
}
