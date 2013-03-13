/*
 * HalResponse.cpp
 *
 * Container and parser for a Response frame.
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

#include "HalResponse.h"

using namespace TI::DLL430;

HalResponse::HalResponse ()
 : id(0)
 , type(0)
 , error(Error_None)
 , isComplete(false)
 , ack(true)
{
}

HalResponse::~HalResponse ()
{
}

void HalResponse::append (uint8_t* data, uint16_t len)
{
	if (!data || !len)
		return;

	this->data.reserve(this->data.size() + len);
	for (uint16_t i = 0; i < len; ++i) 
	{
		this->data.push_back(data[i]);
	}
}

void HalResponse::setId (uint8_t id)
{
	this->id = id;
}

uint8_t HalResponse::getId () const
{
	return this->id;
}

void HalResponse::setIsComplete (uint8_t id)
{
	isComplete = (id&0x80) == 0;
}

bool HalResponse::getIsComplete () const
{
	return isComplete;
}

void HalResponse::setType (uint8_t type)
{
	this->type = type;
}

uint8_t HalResponse::getType () const
{
	return this->type;
}

void HalResponse::setError (errorType error)
{
	this->error = error;
}

HalResponse::errorType HalResponse::getError () const
{
	return error;
}

void HalResponse::setAck (bool rack)
{
	this->ack=rack;
}

bool HalResponse::getAck () const
{
	return this->ack;
}

size_t HalResponse::getSize () const
{
	return this->data.size();
}

const std::vector<uint8_t>& HalResponse::get() const
{
	return this->data;
}

uint8_t HalResponse::at (size_t index) const
{
	return (index < this->data.size()) ? this->data[index] : 0;
}
