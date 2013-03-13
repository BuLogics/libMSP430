/*
 * HalResponse.h
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

#if _MSC_VER > 1000
#pragma once
#endif
#ifndef DLL430_HALRESPONSE_H
#define DLL430_HALRESPONSE_H

#include <stddef.h>
#include <inttypes.h>
#include <vector>

namespace TI
{
	namespace DLL430
	{

		class HalResponse
		{
		public:
			enum errorType {
				Error_None,
				Error_Size,
				Error_CRC
			};

			enum responseType {
				Type_Acknoledge  = 0x91,
				Type_Exception   = 0x92,
				Type_Data        = 0x93,
				Type_DataRequest = 0x94,
				Type_Status      = 0x95
			};
		
			HalResponse ();
			~HalResponse ();
			
			void append (uint8_t* data, uint16_t len);

			void setId (uint8_t id);
			uint8_t getId () const;

			void setIsComplete (uint8_t id);
			bool getIsComplete () const;

			void setType (uint8_t type);
			uint8_t getType () const;

			void setError (errorType error);
			errorType getError () const;

			void setAck (bool ack);
			bool getAck () const;

			size_t getSize() const;
			const std::vector<uint8_t>& get() const;
			uint8_t at (size_t index) const;

		private:
			uint8_t id;
			uint8_t type;
			errorType error;
			std::vector<uint8_t> data;
			bool isComplete;
			bool ack;
		};

	};
};

#endif /* DLL430_HALRESPONSE_H */
