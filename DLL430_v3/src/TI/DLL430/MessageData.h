/*
 * MessageData.h
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

#if _MSC_VER > 1000
#pragma once
#endif
#ifndef DLL430_MESSAGE_DATA_H
#define DLL430_MESSAGE_DATA_H

#include <vector>
#include <string.h>
#include <stdint.h>
#include <boost/shared_ptr.hpp>


namespace TI
{
	namespace DLL430
	{

		class MessageData
		{
		public:
			MessageData();

			explicit MessageData(const std::vector<uint8_t>& data);

			MessageData(const uint8_t* start, const uint8_t* end);

			MessageData(std::vector<uint8_t>::const_iterator start, std::vector<uint8_t>::const_iterator end);
			
			template<typename T>
			MessageData& operator>>(T& t)
			{
				read(&t, sizeof(T));
				return *this;
			}

			template<typename T>
			MessageData& operator<<(const T& t)
			{
				write(&t, sizeof(T));
				return *this;
			}

			MessageData& discard(size_t count);

			void reset();

			bool fail() const;

		private:
			void read(void* dst, size_t count);

			void write(const void* data, size_t count);

			std::vector<uint8_t> mData;
			size_t mReadPos;
			bool mFail;
		};

		typedef boost::shared_ptr<MessageData> MessageDataPtr;		
	};
};

#endif /* DLL430_MESSAGE_DATA_H */
