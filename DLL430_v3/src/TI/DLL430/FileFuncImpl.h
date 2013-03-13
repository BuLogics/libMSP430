/*
 * FileFuncImpl.h
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

#include <fstream>
#include <vector>

#include <stdint.h>
#include <vector>

#if _MSC_VER > 1000
#pragma once
#endif
#ifndef DLL430_FileFuncImpl_H 
#define DLL430_FileFuncImpl_H

#include <DLL430/FileFunc.h>

#define WORD uint16_t

using namespace std;

#define MAX_LINE_SIZE 1024

namespace TI
{
	namespace DLL430
	{
		class DeviceHandle;

		struct DownloadSegment
		{
			std::vector<uint32_t> data;
			uint32_t startAddress;
			uint32_t size;

			DownloadSegment() : startAddress(0), size(0) {}
		};

		class FileFuncImpl : FileFunc
		{
		public:
			FileFuncImpl();

			/** \brief constructor
			 * 
			 * \param fname file name
			 * \param mode expected file mode ti-txt/intel
			 * \return true if successful, else false
			 */
			FileFuncImpl(const char * fname, fileType mode);

			/** \brief destructor
			 */
			~FileFuncImpl();

			/** \brief open
			 * 
			 * open given file, test format and read it
			 *
			 * \param fname file name
			 * \param mode expected file mode ti-txt/intel
			 * \return true if reading was successful, else false
			 */
			int readOpen(const char * fname, fileType mode);

			/** \brief close
			 * 
			 * close file
			 *
			 */
			void close(void);

			/** \brief test file type
			 * 
			 * read first bytes to detect the file type
			 *
			 * \return file type ti-txt or intel-hex
			 */			
			fileType getFileType();

			/** \brief read ti file
			 * 
			 * read the file as ti-txt-file and copy data to internal buffer
			 *
			 * \return true if reading was successful, else false
			 */
			int readTiFile();

			/** \brief read intel hex file
			 * 
			 * read the file as intel-hex-file and copy data to internal buffer
			 *
			 * \return true if reading was successful, else false
			 */

			int readIntelFile();

			bool readFirmware(const char * fname);

			bool readFirmware(const unsigned char * data, size_t size);

			bool readFirmware(const uint16_t* data, const uint32_t* address, 
							  const uint32_t* length, uint32_t sections);

			size_t getNumberOfSegments() const;

			/** \brief print data segements to stdout
			 * 
			 * \param fname file name
			 * \param buffer of values to write
			 * \param number of elements in buffer
			 * \param mode expected file mode ti-txt/intel
			 * \return true if write was successful, else false
			 */
			bool printSeg(const char * fname, uint32_t * buffer, uint32_t start, size_t n, fileType type);

			/** \brief print data segements to stdout
			 * 
			 */
			void printSegs();

			/** \brief write segments
			 * 
			 * write segment information to device
			 *
			 * \return 0 if reading was successful, else -1
			 */			
			bool writeSegs(DeviceHandle * device);

			/** \brief verify segments
			 * 
			 * activate verification on device
			 *
			 * \return 0 if reading was successful, else -1
			 */
			bool verifySegs(DeviceHandle * device, bool intern);

			bool verify(uint32_t address, uint32_t* buffer, size_t count);

			const DownloadSegment* getFirmwareSeg(size_t index) const;

		private:
			ifstream file;
			fileType type;

			vector<DownloadSegment> segments;
			DownloadSegment segment;

			int detectFileType();
			void addSegment();

			void trimWhitespace(string& str);
			bool getTiFileAddress(const string& record, uint32_t* address);
			bool getTiFileBytes(const string& Record, uint32_t* ByteCnt);

			bool gotoIntelRecordStart();
			void readIntelData(istream& stream, uint8_t size, uint32_t address, bool firstData);
		};
	}
}

#endif //DLL430_FileFuncImpl_H
