/*
 * FileFunc.cpp
 *
 * File access for code image files.
 *
 * Copyright (C) 2008 - 2011 Texas Instruments Incorporated - http://www.ti.com/ 
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

#ifdef _MSC_VER
#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS //disabling warnings to  use secure c-function versions (e.g. strcpy_s) as not compatible none MS development
#endif
#endif

#include <iostream>
#include <sstream>
#include <iomanip>

#include <string>

#include <stdlib.h>
#include <string.h>

#include <DLL430/DeviceHandle.h>

#include "FileFuncImpl.h"

using namespace TI::DLL430;
using namespace std;

static const size_t INTEL_HEX_LINE_DATA_LEN = 0x10;


FileFuncImpl::FileFuncImpl()
{
	type=UNKNOWN;
}

FileFuncImpl::FileFuncImpl(const char * fname, fileType mode)
{
	type=UNKNOWN;

	if (!readOpen(fname,mode))
		type=FILE_ERROR;
}

FileFuncImpl::~FileFuncImpl()
{
}

int FileFuncImpl::readOpen(const char * fname, fileType mode)
{
	file.open((const char *)fname,ios::in);

	if (!file.is_open())
		return -1;

	if (detectFileType() != 0)		// format detection was ok
		return -2;

	if (mode != UNKNOWN)		// AUTO detection OK
	{
		if (mode != type)		// no AUTO detection, format is the expected one?
		{
			type = UNKNOWN;
			return -2;
		}
	}

	file.open((const char *)fname,ios::in);
	file.seekg(ios_base::beg);

	if ((type==UNKNOWN)||(type==FILE_ERROR))
		return -2;

	if (type==TI_TXT)
		return readTiFile();

	if (type==INTEL_HEX)
		return readIntelFile();

	return 0;
}

void FileFuncImpl::close(void)
{
	file.close();
}

void FileFuncImpl::trimWhitespace(string& str)
{
	const string whitespaces(" \t\n\r");
	size_t  start = str.find_first_not_of(whitespaces);
	size_t  end = str.find_last_not_of(whitespaces);

	if ( start == string::npos || end == string::npos )
		str.clear();

	else
		str = str.substr(start, end - start + 1);
}

fileType FileFuncImpl::getFileType(void)
{
	return type;
}

int FileFuncImpl::detectFileType()
{
	type=UNKNOWN;

	if (!file.is_open())
		return -1;

	while (!file.eof()) 
	{
		// read a line from file
		string line;
		getline(file, line);

		trimWhitespace(line);

		// ignore empty lines
		if (!line.empty())
		{
			file.close();
			if (line[0] == '@')
			{
				type = TI_TXT;
				return 0;
			}
			if (line[0] == ':')
			{
				type = INTEL_HEX;
				return 0;
			}
		}
	}
	file.close();
	return -1;				// Could not open file.
}

//-- GetAddress ---------------------------------------------------------------
// GetAddress converts an address tag "@xxxx"
// Arguments: char* Record (the line from the file)
//            LONG* wAddress (receives the address)
// Result:    BOOL (TRUE if ok, otherwise FALSE)

bool FileFuncImpl::getTiFileAddress(const string& record, uint32_t * address)
{
	// max. 6 chars (including '@')
	if (record.length() > 6)
		return false;

	stringstream stream(record.substr(1, string::npos));
	uint32_t ulAddress = 0xffffffff;
	stream >> hex >> ulAddress;

	// check for conversion error:
	if (stream.fail())
		return false;

	*address = ulAddress;
	return true;
}

//-- GetBytes -----------------------------------------------------------------
// GetBytes converts a line from the file into bytes
// Arguments: const string& Record (the line from the file)
//            WORD* ByteCnt (receives the number of values stored)
// Result:    bool (true if line is ok, otherwise false)

bool FileFuncImpl::getTiFileBytes(const string& Record, uint32_t* ByteCnt)
{
	stringstream stream(Record);
	int numBytes = 0;
	while(!stream.eof())
	{
		uint32_t byte = 0xffffffff;
		stream >> hex >> byte;

		if (stream.fail() || byte > 255 || ++numBytes > 16)
			return false;

		segment.data.push_back(byte);
		*ByteCnt = (uint32_t)segment.data.size();
	}
	return true;
}

int FileFuncImpl::readTiFile()
{
	uint32_t StartAddress=0;

	bool HaveAddress = false;
	bool LastRecordWasEOF = false;

	if (!file.is_open())
		return -1;

	segments.clear();

	segment.size = 0;
	segment.startAddress = 0;
	segment.data.clear();

	while (!file.eof()) 
	{
		// read a line from file
		string line;
		getline(file, line);
		trimWhitespace(line);

		if (!line.empty())
		{
			if (line == "q" || line == "Q") // "q" or "Q" indicates EOF.
			{
				LastRecordWasEOF = true;
				if (HaveAddress)
				{
					addSegment();
					segment.startAddress=StartAddress;
				}
			}
			else
			{
				LastRecordWasEOF = false;

				if (line[0] == '@')		// Record is possibly an address.
				{
					if (!getTiFileAddress(line, &StartAddress))	// copy to buffer
					{
						break; // Invalid address.
					}

					if (HaveAddress) // Flush the current data.
					{
						addSegment();
					}
					HaveAddress = true;							// following data starts on this address
					segment.startAddress=StartAddress;			// save given start address
				}
				else
				{
					if (!getTiFileBytes(line, &segment.size))
					{
						break; // Invalid data.
					}
				}
			}	// if (record[0] == '@')
		}	// if (strlen(record) != 0)

	}	// while(true) 
	file.close();

	return LastRecordWasEOF ? 0 : -4;
}

void FileFuncImpl::addSegment()
{
	if (!segment.data.empty())
		segments.push_back(segment);

	segment.data.clear();
	segment.size = 0;
	segment.startAddress = 0;
}

bool FileFuncImpl::gotoIntelRecordStart()
{
	while (!file.eof())
	{
		char character = '\0';
		file >> character;
		if (character == ':')
			return true;
	}
	return false;
}

namespace // Intel Hex specific functions
{
	struct IntelHeader
	{
		uint8_t size;
		uint16_t address;
		uint8_t type;
	};

	template<typename T>
	T readHexFromStream(istream& stream, T* value)
	{
		const int numCharacters = 2 * sizeof(T);
		char byteString[numCharacters];
		stream.read(byteString, numCharacters);

		stringstream conv( string(byteString, numCharacters) );
		unsigned int tmp = 0;		
		conv >> hex >> tmp;
		return *value = static_cast<T>(tmp);
	}

	IntelHeader readIntelHeader(istream& file)
	{
		IntelHeader header;
		readHexFromStream(file, &header.size);
		readHexFromStream(file, &header.address);
		readHexFromStream(file, &header.type);
		return header;
	}

	//Check CRC
	bool checkIntelCRC(const string& line, uint8_t crc)
	{
		uint8_t actualCRC = 0;
		size_t bytesInLine = line.size()/2 - 1;
		
		stringstream crcStream(line);

		for (size_t i = 0; i < bytesInLine; ++i)
		{
			uint8_t byte = readHexFromStream(crcStream, &byte);
			actualCRC += byte;
		}

		actualCRC = ~actualCRC + 1;
		return (actualCRC == crc);
	}

	void writeIntelRecord(ostream& stream, uint8_t size, uint16_t address, uint8_t type, uint32_t* data)
	{
		stream << hex << setfill('0');
		stream << ':' << setw(2) << (int)size << setw(4) << address << setw(2) << (int)type;
		uint8_t crc = size + (address & 0xFF) + (address >> 8) + type;

		for (int i = 0; i < size; ++i)
		{
			stream << setw(2) << (int)data[i];
			crc += data[i];
		}
		crc = ~crc + 1;

		stream << (int)crc << '\n';
	}

	void writeIntelSegment(ostream& stream, uint32_t* data, uint32_t address32, size_t size)
	{
		if (data == 0 || size == 0)
			return;

		uint32_t addressOffset = 0;

		while (size > 0)
		{
			uint8_t crc = 0;
			if (address32 - addressOffset > 0xFFFFF) //32bit offset
			{
				addressOffset = (address32 & 0xFFFF0000);
				uint32_t offset[2] = {(addressOffset >> 16) & 0xFF, (addressOffset >> 24)};
				writeIntelRecord(stream, 2, 0, 4, offset);
			}
			else if (address32 - addressOffset > 0xFFFF) //20bit offset
			{
				addressOffset = (address32 & 0xFFFF0);
				uint32_t offset[2] = {(addressOffset >> 4) & 0xFF, (addressOffset >> 12) & 0xFF};
				writeIntelRecord(stream, 2, 0, 2, offset);
			}

			const uint16_t address16 = static_cast<uint16_t>(address32 - addressOffset);
			uint8_t toWrite = static_cast<uint8_t>( min(size, INTEL_HEX_LINE_DATA_LEN) );
			
			const size_t align = address16 % INTEL_HEX_LINE_DATA_LEN;
			if (align > 0)
				toWrite = static_cast<uint8_t>( min(size, INTEL_HEX_LINE_DATA_LEN - align) );
			
			writeIntelRecord(stream, toWrite, address16, 0, data);

			address32 += toWrite;
			data += toWrite;
			size -= toWrite;
		}
		writeIntelRecord(stream, 0, 0, 1, 0);
	}
}


int FileFuncImpl::readIntelFile()
{	
	//Size Address Type Data CRC
	//:10 0100 00 214601360121470136007EFE09D21901 40

	bool firstData = true;
	bool eofFound = false;

	string line;

	uint32_t addressOffset = 0;
	
	while (!file.eof() && !eofFound)
	{
		if (!gotoIntelRecordStart())
			return -3;

		uint16_t offset = 0;

		getline(file, line);
		stringstream stream(line);

		IntelHeader header = readIntelHeader(stream);

		switch (header.type)
		{
		case 0: //Regular data
			readIntelData(stream, header.size, addressOffset + header.address, firstData);
			firstData = false;
			break;

		case 1: //EOF record
			eofFound = true;
			addSegment();
			break;
		
		case 2: //Extended address (20bit)
			readHexFromStream(stream, &offset);
			addressOffset = (uint32_t)offset << 4;
			break;
		
		case 3: //Only for x86 architecture, skip line
			continue;
		
		case 4: //Extended address (32bit)
			readHexFromStream(stream, &offset);
			addressOffset = (uint32_t)offset << 16;
			break;
		
		case 5: //Only for 386 architecture or higher, skip line
			continue;
		
		default: //Unknown record type, be generous and ignore
			continue;
		}

		uint8_t crc = 0;
		readHexFromStream(stream, &crc);

		if (!checkIntelCRC(line, crc))
			return -1;
	}
	return 0;
}

void FileFuncImpl::readIntelData(istream& stream, uint8_t size, uint32_t address, bool firstData)
{
	const bool newSegment = firstData || (address != segment.startAddress + segment.size);
	
	if (newSegment && !firstData)
		addSegment();

	if (newSegment)
		segment.startAddress = address;

	for (int i = 0; i < size; ++i)
	{
		uint8_t byte = readHexFromStream(stream, &byte);
		segment.data.push_back(byte);
		++segment.size;
	}
}

bool FileFuncImpl::readFirmware(const char * fname)
{
	return readOpen(fname, TI_TXT) == 0;
}

bool FileFuncImpl::readFirmware(const uint16_t* data, const uint32_t* address, 
								 const uint32_t* length, uint32_t sections)
{
	for (uint32_t i = 0; i < sections; ++i)
	{
		segment.data.clear();
		segment.size = 2 * length[i];
		segment.startAddress = address[i];

		for (uint32_t word = 0; word < length[i]; ++word, ++data)
		{
			segment.data.push_back((uint8_t)(*data & 0xFF));
			segment.data.push_back((uint8_t)(*data >> 8));
		}
		addSegment();
	}
	return true;
}

bool FileFuncImpl::readFirmware(const unsigned char * i_data, size_t i_size)
{
	i_data+=6;
	i_size-=6;
	while(i_size>0)
	{
		uint32_t s_addr=i_data[0]+(((uint32_t)i_data[1])<<8)+(((uint32_t)i_data[2])<<16)+(((uint32_t)i_data[3])<<24);
		uint32_t s_size=i_data[4]+(((uint32_t)i_data[5])<<8)+(((uint32_t)i_data[6])<<16)+(((uint32_t)i_data[7])<<24);
		i_data+=8;
		i_size-=8;
		segment.size=s_size;
		segment.startAddress=s_addr;
		
		if(i_size<segment.size)
			return false;
		
		for(size_t i=0;i<s_size;i++)
			segment.data.push_back((uint32_t)i_data[i]);
		
		this->addSegment();
		i_data+=s_size;
		i_size-=s_size;
	}
	return true;
}

size_t FileFuncImpl::getNumberOfSegments() const
{
	return segments.size();
}

bool FileFuncImpl::printSeg(const char * fname, uint32_t * buffer, uint32_t start, size_t n, fileType type)
{
	ofstream filestr(fname);
	if(!filestr.is_open())
		return false;

	int count = (int)n;
	int i;

	switch (type)
	{
	case INTEL_HEX:
		writeIntelSegment(filestr, buffer, start, n);
		break;

	case TI_TXT:
		// print first line with address
		filestr << '@' << setw(4) << hex << setfill ('0') << uppercase << (uint32_t)start << '\n';
		while (count > 0)
		{
			int bytesInLine = min(count, 16);
			for (i = 0; i < bytesInLine; ++i)
			{
				filestr << setw(2) << hex << setfill ('0') << uppercase << (uint16_t)buffer[i];
				if (i != bytesInLine-1)
					filestr << ' ';
			}
			filestr << '\n';	// line finished
			count -= 16;
			buffer += 16;
		}
		// all data written, set 'q' as end sign
		filestr << 'q' << '\n';
		break;

	default:
		filestr.close();
		return false;
	}
	filestr.close();
	return true;
}

void FileFuncImpl::printSegs()
{
	for(size_t i=0;i<segments.size();i++)
	{
		printf("[%i %6x %i]\n",i,segments[i].startAddress,segments[i].size);
		for(size_t j=0;j<segments[i].size;j++)
			printf("%02x  ",segments[i].data[j]);

		printf("\n");
	}
}

bool FileFuncImpl::writeSegs(DeviceHandle * device)
{
	if(device==NULL)
		return false;

	MemoryManager * mm=device->getMemoryManager();

	if (mm == NULL)		// identifyDevice not called?
		return false;

	for (size_t i = 0; i < segments.size(); ++i)
	{
		if (!segments[i].data.empty())
		{
			if((!mm->write(segments[i].startAddress,
				&segments[i].data[0],
				segments[i].size)))
			{
				return false;	//adr,buf,count
			}
		}
	}
	if(!mm->sync())
		return false;

	return true;
}

bool FileFuncImpl::verify(uint32_t address, uint32_t* buffer, size_t count)
{
	return false;
}

bool FileFuncImpl::verifySegs(DeviceHandle * device, bool intern)
{
	if (device == NULL)
		return false;

	MemoryManager * mm = device->getMemoryManager();

	if (mm == NULL)
		return false;

	if (!intern)
	{
		for (size_t i = 0; i < segments.size(); ++i)
		{
			if (!mm->verify(segments[i].startAddress, &segments[i].data[0], segments[i].size))
			{
				return false;
			}
		}
	}
	return true;
}

const DownloadSegment* FileFuncImpl::getFirmwareSeg(size_t index) const
{
	if (index >= segments.size())
		return NULL;

	return &(segments[index]);
}
