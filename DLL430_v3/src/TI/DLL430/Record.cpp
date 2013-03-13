/*
 * Record.cpp
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
 
 
#include "Record.h"

using namespace TI::DLL430;

//provides access to Record Code Array
//
//params: data,	pointer to array containing flash data
//		  address, pointer to array containing the start addresses of all sections
//		  length, pointer to array containing the length of all sections
//		  sections, number of available sections
Record::Record(const uint16_t* data, const uint32_t* address, const uint32_t* length, const uint32_t sections)
: data(data)
, address(address)
, length(length)
, sectCount(sections)
, currentSect(1)
, currentWord(1)
{
}

//params: none
//returns: true, if there's a next word in current section
bool Record::sectHasNextWord()
{
	uint32_t maxPos = comMaxPos(currentSect);
	
	if(currentWord <= maxPos)
	{
		return true;
	}
	else
	{
		return false;
	}
}

//params: none
//returns: true, if there's a next section
bool Record::hasNextSect()
{
	if(currentSect <= sectCount)
	{
		return true;
	}
	else
	{
		return false;
	}
}

//params: section
//returns: sets current section to given section
//		  (nextWord returns the first word in section)
bool Record::setSection(uint32_t sect)
{
	if(sect > 0 && sect <= sectCount)
	{
		currentSect = sect;
		currentWord = comMaxPos(sect-1) + 1;
		return true;
	}
	else
	{
		return false;
	}
}

//params: none
//returns: sets current section to next section
//		  (nextWord returns the first word in the new section)
bool Record::nextSection()
{
	if(this->hasNextSect())
	{
		currentWord = comMaxPos(currentSect) + 1;
		currentSect++;
		return true;
	}
	return false;
}

//params: none
//returns: current word in section
uint16_t Record::getNextWord()
{
	uint16_t retWord = data[currentWord - 1];
	currentWord++;
	return retWord;
}

//params: address
//params: if data contains given address, 
//returns: true, if data contains given address
bool Record::getWordAtAdr(uint32_t searchAdr, uint16_t* retWord)
{
	bool isSectFound = false;
	uint32_t foundSect = 0;
	uint32_t actSect = 1;
	uint32_t tmpSearchAdr;

	while(actSect < sectCount && !isSectFound)
	{		
		uint32_t sectStart = address[actSect-1];
		uint32_t sectEnd = sectStart + (length[actSect-1]);

		tmpSearchAdr = (searchAdr - sectStart)/2 + sectStart;

		if((tmpSearchAdr <= sectEnd) && (tmpSearchAdr >= sectStart))
		{
			foundSect = actSect;
			isSectFound = true;
		}
		actSect++;
	}
	if(isSectFound)
	{
		uint32_t offset = tmpSearchAdr - address[foundSect-1];
		*retWord = data[comMaxPos(foundSect - 1) + offset];
		return true;
	}
	else
	{
		return false;
	}
}

//params: none
//returns: physical start address of current section
uint32_t Record::getSectStartAdr()
{
	return this->getSectStartAdr(currentSect);
}

//params: section
//returns: physical start address of given section
uint32_t Record::getSectStartAdr(uint32_t sect)
{
	if(sect > 0 && sect <= sectCount)
	{
		return address[sect-1];
	}
	else
	{
		return 0;
	}
}

//params: none
//returns: current position in words
uint32_t Record::getCurrentPosWord()
{
	return currentWord;
}

//params: none
//returns: current position in bytes
uint32_t Record::getCurrentPosByte()
{
	return currentWord*2;
}

uint32_t Record::getPosWordinSect()
{
	if(currentSect == 1)
	{
		return currentWord;
	}
	else
	{
		return (currentWord - comMaxPos(currentSect-1) + 1);
	}
}

//params: none
//returns: current section
uint32_t Record::getCurrentSect()
{
	return currentSect;
}

//params: none
//returns: length of current section in words
uint32_t Record::getSectLength()
{
	return this->getSectLength(currentSect);
}

//params: none
//returns: length of current section in bytes
uint32_t Record::getSectByteLength()
{
	return this->getSectLength()*2;
}

//params: section
//returns: length of given section
uint32_t Record::getSectLength(uint32_t sect)
{
	if(sect > 0 && sect <= sectCount)
	{
		return length[sect-1];
	}
	else
	{
		return 0;
	}
}

//params: none
//returns: number of all sections in array
uint32_t Record::getSectCount()
{
	return sectCount;
}

//params: none
//returns: number of all words in data array
uint32_t Record::getNumOfAllDataWords()
{
	uint32_t numWords = 0;

	for(uint32_t i = 1; i<= sectCount; i++)
	{
		numWords += length[i-1];
	}
	return numWords;
}

//params: true, if core signature will be added to output
//returns: number of additional words for section-address and -length
uint32_t Record::getNumOfManageWords(bool hasCoreSignature)
{
	uint32_t manageWords = sectCount*2 + 1;
	if(hasCoreSignature)
	{
		manageWords += 2;
	}
	return manageWords;
}

//params: section
//returns: absolute position of last word in section
uint32_t Record::comMaxPos(uint32_t sect)
{
	uint32_t offset = 0;

	for(uint32_t i = 1; i<= sect; i++)
	{
		offset += length[i-1];
	}
	return offset;
}
