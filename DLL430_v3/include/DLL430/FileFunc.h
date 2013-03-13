/*
 * FileFunc.h
 *
 * Base class for accesing code image files.
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

#if _MSC_VER > 1000
#pragma once
#endif
#ifndef DLL430_FileFunc_H
#define DLL430_FileFunc_H

using namespace std;

namespace TI
{
	namespace DLL430
	{
		enum fileType
		{
			UNKNOWN = 0,				// file type is unknown
			TI_TXT = 1,					// file type is ti-txt-file
			INTEL_HEX = 2,				// file type is intel-hex
			FILE_ERROR = 3,				// error detecting type: file corrupted or wrong file selected
		};

		class DLL430_SYMBOL FileFunc
		{
		public:
			/** \brief readOpen
			 * 
			 * open given file, test format and read it
			 *
			 * \param fname file name
			 * \param mode expected file mode ti-txt/intel
			 * \return -1 if reading was successful, else 0
			 */
			virtual int readOpen(const char * fname, fileType mode)=0;

			/** \brief close
			 * 
			 * close file
			 *
			 */
			virtual void close(void)=0;

			/** \brief test file type
			 * 
			 * read first bytes to detect the file type
			 *
			 * \return file type ti-txt or intel-hex
			 */			
			virtual fileType getFileType(void)=0;

			/** \brief read ti file
			 * 
			 * read the file as ti-txt-file and copy data to internal buffer
			 *
			 * \return true if reading was successful, else false
			 */
			virtual int readTiFile(void)=0;

			/** \brief read intel hex file
			 * 
			 * read the file as intel-hex-file and copy data to internal buffer
			 *
			 * \return true if reading was successful, else false
			 */
			virtual int readIntelFile(void)=0;
			
			/** \brief Read firmware from file
			 * 
			 * \return true if reading was successful, else false
			 */
			virtual bool readFirmware(const char * fname)=0;

			/** \brief copy firmware buffer to segment array
			 *
			 * \return true if successful, else false
			 */
			virtual bool readFirmware(const unsigned char * data, size_t size)=0;

			virtual bool readFirmware(const uint16_t* data, const uint32_t* address, 
									  const uint32_t* length, uint32_t sections) = 0;

			/** \brief print data segements to stdout
			 * 
			 * \param fname file name
			 * \param buffer of values to write
			 * \param number of elements in buffer
			 * \param mode expected file mode ti-txt/intel
			 * \return true if write was successful, else false
			 */
			virtual bool printSeg(const char * fname, uint32_t * buffer, uint32_t start, size_t n, fileType type)=0;

			/** \brief return the number of segments in the current file
			 * 
			 * returns the value of the counter of memory locations 
			 * which are copied into the segemnt structure
			 *
			 * \return true if reading was successful, else false
			 */
			virtual size_t getNumberOfSegments() const = 0;

			/** \brief print data segements to stdout
			 * 
			 */
			virtual void printSegs()=0;

			/** \brief verify segments
			 * 
			 * activate verification on device
			 *
			 * \return 0 if reading was successful, else -1
			 */
			virtual bool verify(uint32_t address, uint32_t* buffer, size_t count)=0;
		};
	}
}

#endif // DLL430_FileFunc_H
