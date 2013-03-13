/*
 * EemRegisterAccess.cpp
 *
 * Handles access to eem registers through EemMemoryAccess 
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


#include <stdio.h>
#include <vector>

#include "EemRegisterAccess.h"
#include "EemMemoryAccess.h"
#include "../Exceptions/Exceptions.h"

namespace 
{
	static TI::DLL430::EemMemoryAccess* ema_ = 0;

	struct CacheEntry
	{
		CacheEntry() : value(0), valid(false) {}

		uint32_t value;
		bool valid;
	};

	std::vector<CacheEntry> cache;
}



void TI::DLL430::setEemRegisterAccess(EemMemoryAccess* ema)
{
	cache.clear();
	//0xBE currently largest used address, no odd addresses
	cache.resize((0xBE / 2) + 1); 
	ema_ = ema;
}



void TI::DLL430::writeEemRegister(uint32_t reg, uint32_t value, bool ignoreCache)
{
	const size_t slot = reg / 2;
	if (slot >= cache.size())
		cache.resize(slot);

	if (!cache[slot].valid || cache[slot].value != value || ignoreCache)
	{
		if (!(ema_ && ema_->writeEemRegister((EemRegister)reg, value) && ema_->sync()))
			throw EM_RegisterWriteException();
	}

	cache[slot].value = value;
	cache[slot].valid = true;
}

uint32_t TI::DLL430::readEemRegister(uint32_t reg)
{
	const size_t slot = reg / 2;
	if (slot > cache.size())
		cache.resize(slot);

	uint32_t value = 0;
	if (!(ema_ && ema_->readEemRegister((EemRegister)reg, &value) && ema_->sync()))
		throw EM_RegisterReadException();

	cache[slot].value = value;
	cache[slot].valid = true;

	return value;
}
