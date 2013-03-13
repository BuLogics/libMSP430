/*
 * Record.h
 *
 * Handles access to different memory modules by means of address.
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

#ifndef RECORD_H
#define RECORD_H

#include <inttypes.h>

namespace TI
{
	namespace DLL430
	{

		class Record
		{
		public:
	
			Record(const uint16_t* data, const uint32_t* address, const uint32_t* length, const uint32_t sections);
		
			const static uint32_t coreSignature = 0xFEDF2112;

			bool sectHasNextWord();
			bool hasNextSect();
			bool nextSection();
			bool setSection(uint32_t sect);
			bool getWordAtAdr(uint32_t address, uint16_t* retWord);
			uint16_t getNextWord();
			uint32_t getSectStartAdr();
			uint32_t getSectStartAdr(uint32_t sect);
			uint32_t getCurrentPosWord();
			uint32_t getCurrentPosByte();
			uint32_t getPosWordinSect();
			uint32_t getCurrentSect();
			uint32_t getSectLength();
			uint32_t getSectLength(uint32_t sect);
			uint32_t getSectByteLength();
			uint32_t getSectCount();
			uint32_t getNumOfAllDataWords();
			uint32_t getNumOfManageWords(bool hasCoreSignature);

		private:
			
			const uint16_t* data;
			const uint32_t* address;
			const uint32_t* length;
			const uint32_t sectCount;
			uint32_t currentSect;
			uint32_t currentWord;

			uint32_t comMaxPos(uint32_t sect);
			uint32_t comStartPos(uint32_t sect);

		};
	};
};

#endif
