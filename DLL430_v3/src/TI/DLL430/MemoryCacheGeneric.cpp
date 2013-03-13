/*
 * MemoryCacheGeneric.cpp
 *
 * Base class for all memory classes with caching capabilities.
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
#include <stdlib.h>
#include <string.h>

#include "MemoryCacheGeneric.h"

#define CACHE_FLAG_VALID (1 << 0)
#define CACHE_FLAG_DIRTY (1 << 1)

using namespace TI::DLL430;

MemoryCacheGeneric::MemoryCacheGeneric (MemoryArea* parent)
 : parent(parent)
 , enabled(true)
 , syncNeeded(false)
 , cache(parent->getSize(), 0)
 , dirty(false)
 , flags(parent->getSize(), 0)
{
}

size_t MemoryCacheGeneric::size () const
{
	return this->parent->getSize();
}

bool MemoryCacheGeneric::read (uint32_t address, uint32_t* buffer, size_t count)
{
	if (!this->enabled)
		return this->parent->read(address, buffer, count);

	if (address+count > this->size())
		return false;

	this->fillNosync(address, count);
	if (buffer) 
	{
		readElement r;
		r.address = address;
		r.v_buffer = buffer;
		r.count = count;
		this->readList.push_back(r);
	}
	return true;
}

bool MemoryCacheGeneric::write (uint32_t address, uint32_t* buffer, size_t count)
{
	if (!this->enabled)
		return this->parent->write(address, buffer, count);

	if (address+count > this->size())
		return false;

	for (size_t i = 0; i < count && address+i < this->size(); ++i) 
	{
		this->cache[address+i] = buffer[i];
		this->setFlag(address+i, CACHE_FLAG_DIRTY | CACHE_FLAG_VALID);
	}
	
	this->dirty = true;
	return true;
}

bool MemoryCacheGeneric::write (uint32_t address, uint32_t value)
{
	if (!this->enabled)
		return this->parent->write(address, value);

	if (address >= this->size())
		return false;

	this->cache[address] = value;
	this->setFlag(address, CACHE_FLAG_DIRTY | CACHE_FLAG_VALID);
	this->dirty = true;
	return true;
}

bool MemoryCacheGeneric::isReadOnly()
{
	return this->parent->isReadOnly();
}

bool MemoryCacheGeneric::sync ()
{
	if (this->syncNeeded) 
	{
		/* the real I/O sync updates the cache... */
		if (!this->parent->sync())
			return false;

		this->syncNeeded = false;
	}

	/* ...so we can fill the buffers the the read
	 * function was called with.
	 */
	readElement_list::iterator it = this->readList.begin();
	for (; it != this->readList.end(); ++it) 
	{
		readElement r = *it;
		for (size_t i = 0; i < r.count; ++i)
			r.v_buffer[i] = this->cache[r.address+i];
	}
	this->readList.clear();
	return true;
}

bool MemoryCacheGeneric::erase ()
{
	if (this->enabled)
		this->clear(0, this->size());

	return this->parent->erase();
}

bool MemoryCacheGeneric::erase (uint32_t start, uint32_t end)
{
	uint32_t segSize = this->parent->getSegmentSize();
	start = (start / segSize) * segSize;
	end = ((start / segSize) * segSize) + (segSize - 1);

	if (this->enabled)
		this->clear(start, end - start + 1);

	return this->parent->erase(start, end);
}

bool MemoryCacheGeneric::verify(uint32_t address, uint32_t* buffer, size_t count)
{
	if (!this->flush(address, count))
		return false;

	return this->parent->verify(address, buffer, count);
}

void MemoryCacheGeneric::enable (bool enable)
{
	if (!enable && this->enabled)
		flush(0, this->size());

	this->enabled = enable;
}

bool MemoryCacheGeneric::isEnabled () const
{
	return this->enabled;
}

bool MemoryCacheGeneric::checkCache (uint32_t address) const
{
	return (this->enabled && address < this->size() && this->checkFlag(address, CACHE_FLAG_VALID));
}

bool MemoryCacheGeneric::markDirty (uint32_t address, size_t count)
{
	if (!fill(address, count))
		return false;

	while (count--)
		this->setFlag(address++, CACHE_FLAG_DIRTY);

	return true;
}

bool MemoryCacheGeneric::fillNosync (uint32_t address, size_t count)
{
	for (uint32_t i = address; i < address+count;) 
	{
		uint32_t start = i;
		uint32_t size = 0;
		while (i < this->size() &&
		       i < address+count && 
			   !this->checkFlag(i++, CACHE_FLAG_VALID))
		{
			++size;
		}
		if (size) 
		{
			if (!this->parent->read(start, &cache[start], size)) 
			{
				return false;
			}
			
			for (size_t k = 0; k < size; ++k) 
			{
				setFlag(start+k, CACHE_FLAG_VALID);
			}
			this->syncNeeded = true;
		}
	}
	return true;
}

bool MemoryCacheGeneric::fill (uint32_t address, size_t count)
{
	if (!this->enabled || address+count > this->size())
		return false;

	return fillNosync(address, count) && sync();
}

bool MemoryCacheGeneric::flush (uint32_t address, size_t count)
{
	if (address+count > this->size())
		return false;

	if (!this->enabled || !this->dirty)
		return true;

	for (uint32_t i = 0; i < count;) 
	{
		uint32_t start = i;
		uint32_t size = 0;
		while (i < count && this->checkFlag(address + i++, CACHE_FLAG_DIRTY))
			++size;

		if (size) 
		{
			if (!this->parent->write(address + start, &cache[address + start], size)) 
			{
				return false;
			}
		}
	}
	if (!this->parent->sync())
		return false;

	for (uint32_t i = 0; i < count; ++i)
		this->setFlag(address + i, CACHE_FLAG_DIRTY, false);

	this->dirty = false;
	return true;
}

void MemoryCacheGeneric::clear (uint32_t address, size_t count)
{
	if (address > this->size())
		return;

	if (address+count > this->size())
		count = this->size()-address;

	for (size_t i = 0; i < count; ++i) 
	{
		this->setFlag(address + i, CACHE_FLAG_VALID | CACHE_FLAG_DIRTY, false);
	}
	memset(&this->cache[address], 0, count);
}

bool MemoryCacheGeneric::checkFlag (uint32_t address, uint8_t flag) const
{
	return (this->flags[address] & flag) != 0;
}

void MemoryCacheGeneric::setFlag (uint32_t address, uint8_t flag, bool set)
{
	if (set)
		this->flags[address] |= flag;
	else
		this->flags[address] &= ~flag;
}
