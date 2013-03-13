/*
 * MessageData.cpp
 *
 * Encapsulate raw message data for easier access
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


#include "MessageData.h"

using namespace TI::DLL430;


MessageData::MessageData()
	: mReadPos(0), mFail(false)
{
}


MessageData::MessageData(const std::vector<uint8_t>& data) 
	: mData(data), mReadPos(0), mFail(false) 
{
}


MessageData::MessageData(const uint8_t* start, const uint8_t* end)
	: mData(start, end), mReadPos(0), mFail(false) 
{
}


MessageData::MessageData(std::vector<uint8_t>::const_iterator start, std::vector<uint8_t>::const_iterator end)
	: mData(start, end), mReadPos(0), mFail(false) 
{
}


void MessageData::reset()
{
	mReadPos = 0;
	mFail = false;
}


bool MessageData::fail() const 
{
	return mFail; 
}


MessageData& MessageData::discard(size_t count)
{
	if (mReadPos + count <= mData.size())
	{
		mReadPos += count;
	}
	else
	{
		mFail = true;
	}
	return *this;
}


void MessageData::read(void* dst, size_t count)
{
	if (mReadPos + count <= mData.size())
	{
		memcpy(dst, &mData[mReadPos], count);
		mReadPos += count;
	}
	else
	{
		mFail = true;
	}
}


void MessageData::write(const void* data, size_t count)
{
	size_t end = mData.size();
	mData.resize( end + count );
	memcpy(&mData[end], data, count);
}
