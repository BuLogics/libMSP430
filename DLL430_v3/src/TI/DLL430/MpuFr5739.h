/*
 * MpuFr5739.h
 *
 * Functionality for configuring target device.
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
#ifndef DLL430_MPUFR5739_H
#define DLL430_MPUFR5739_H


namespace TI
{
	namespace DLL430
	{
		enum MpuAccessType { MPU_READ = 1, MPU_WRITE = 2, MPU_EXEC = 4 };

		struct MpuAccessibleSegments
		{
			MpuAccessibleSegments(uint32_t address, uint32_t count): address(address), count(count){}
			MpuAccessibleSegments(): address(0), count(0){}

			uint32_t address;
			uint32_t count;		
		};

		class MpuFr5739
		{
		public:

			MpuFr5739(			
				DeviceHandleV3* devHandlev,
				MemoryManager* mmv
			);
			virtual ~MpuFr5739() {};

			bool calculateMpuBorders();
			bool readMpuSettings();
		    bool disableMpu();
			bool isMpuEnabled();

		    std::vector<MpuAccessibleSegments> checkAccessRights(uint32_t address, uint32_t count, MpuAccessType access);
		
		private :
			static const uint32_t framCtlKey = 0xA500;
			static const uint32_t mpuRegAddress = 0x000005A0;

			DeviceHandleV3* devHandle;
			MemoryManager* mm;
			// MPU control register 0
			uint16_t MPUCTL0;
			// MPU control register 1
			uint16_t MPUCTL1;
			// MPU segmentaition register 
			uint16_t MPUSEG;
			//  MPUSAM register 
			uint16_t MPUSAM;

			uint32_t mpuBorder1;
			uint32_t mpuBorder2;
		};
	};
};
#endif /* DLL430_MPUFR5739_H */
