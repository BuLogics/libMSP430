/*
 * MpuFr5739.cpp
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

#include "ConfigManagerV3.h"
#include "DeviceHandleV3.h"
#include "FetHandleV3.h"
#include "MpuFr5739.h"

using namespace TI::DLL430;

MpuFr5739::MpuFr5739(DeviceHandleV3* devHandle, MemoryManager* mm)
	: devHandle(devHandle), mm(mm)
	, MPUCTL0(0), MPUCTL1(0)
	, MPUSEG(0), MPUSAM(0)
	, mpuBorder1(0), mpuBorder2(0)
{
	
}

bool MpuFr5739::isMpuEnabled()
{	
	readMpuSettings();

	return (MPUCTL0 & 0x1) != 0;
}

bool MpuFr5739::calculateMpuBorders()
{
	readMpuSettings();

	MemoryArea* main	 = mm->getMemoryArea("main", 0);   
	MemoryArea* info	 = mm->getMemoryArea("information", 0);
	MemoryArea* bootcode = mm->getMemoryArea("bootcode", 0);
	
	if (!main || !info || !bootcode)
	{
		return false;
	}
	const uint32_t MemorySize = (main->getEnd() - main->getStart() + 1) + 
								(info->getEnd() - info->getStart() + 1) + 
								(bootcode->getEnd() - bootcode->getStart() + 1);
	const uint32_t segmentSize = MemorySize / 32;
	const uint32_t memoryBegin = main->getEnd() - MemorySize + 1;

	this->mpuBorder1 =  memoryBegin + (this->MPUSEG & 0x1F) * segmentSize;
	this->mpuBorder2 =  memoryBegin + ((this->MPUSEG>>8) & 0x1F) * segmentSize;

	return true;
}

/* this function returns the address and length of data which could be written to a segment, take care of segment boarders etc */
std::vector<MpuAccessibleSegments> MpuFr5739::checkAccessRights(uint32_t address, uint32_t count, MpuAccessType access)
{
	std::vector<MpuAccessibleSegments> MpuSegments;

	readMpuSettings();

	MemoryArea* main	 = mm->getMemoryArea("main", 0);
	MemoryArea* info	 = mm->getMemoryArea("information", 0);
	
	if (!main || !info)
	{
		return MpuSegments;
	}

	struct SegmentRange 
	{ 
		SegmentRange(uint32_t begin = 0, uint32_t end = 0, uint16_t bit = 0) : begin(begin), end(end), accessBit(bit) {}
		uint32_t begin, end;
		uint16_t accessBit;
	};

	SegmentRange segments[] =
	{
		SegmentRange( info->getStart(), info->getEnd() + 1, (access << 12) ),
		SegmentRange( main->getStart(), mpuBorder1, (access << 0) ),
		SegmentRange( mpuBorder1, mpuBorder2, (access << 4) ),
		SegmentRange( mpuBorder2, main->getEnd() + 1, (access << 8) )
	};
	const size_t numSegments = sizeof(segments) / sizeof(*segments);

	const uint32_t endAddress = address + count;

	for (size_t i = 0; i < numSegments; ++i)
	{
		if ( (address < segments[i].end) && (endAddress > segments[i].begin) && (MPUSAM & segments[i].accessBit) )
		{
			const uint32_t writeAddress = max(address, segments[i].begin);
			const uint32_t writeEndAddress = min(endAddress, segments[i].end);
			const uint32_t writeCount = writeEndAddress - writeAddress;

			if (writeCount > 0)
			{
				MpuSegments.push_back( MpuAccessibleSegments(writeAddress, writeCount) );
			}
		}
	}	

	return MpuSegments;
}

bool MpuFr5739::readMpuSettings()
{
	/* read out  mpu settings*/
	MemoryArea* peripheral = mm->getMemoryArea("peripheral16bit", 0);
	if (!peripheral)
	{
		return false;
	}
	uint32_t tmp[8] = {0};
	if (!peripheral->read(mpuRegAddress, tmp, 8) || !peripheral->sync())
	{
		return false; 
	}
	this->MPUCTL0 = tmp[0] | (tmp[1] << 8);
	this->MPUCTL1 = tmp[2] | (tmp[3] << 8);
	this->MPUSEG  = tmp[4] | (tmp[5] << 8);
	this->MPUSAM  = tmp[6] | (tmp[7] << 8);

	return true;
}

bool MpuFr5739::disableMpu()
{
	/* MPU is not enabled just return no error */
	if ((MPUCTL0 & 0x1)==0)
	{
		return true;  
	}  
	/* Check if lock bit of MPU is set */
	if ((MPUCTL0 & 0x3)!=1)
    {  
		ConfigManager* cm = this->devHandle->getFetHandle()->getConfigManager();

		/* assert hard RST/NMI and feed in magic pattern to stop device execution */
		/* thr RST/NMI will remove the register protection */ 
		if(!cm->reset(false, 10, 0x91))
		{
			return false;
		}
		/* restart jtag connection and if needed feed in JTAG passowrd */ 
		if(cm->start() != 0x1)
		{
			return false;
		}
		if(!devHandle->reset())
		{	

			return false;
		}
	}
	MemoryArea* peripheral = mm->getMemoryArea("peripheral16bit", 0);   
	if (!peripheral)
	{
		return false;
	}
	/* mpu ist not locked (anymore), disable it set Mpuena = 0  */
	/* write also Fram MPUCTL0 key also into register to gain access */
	if (!peripheral->write(mpuRegAddress, framCtlKey)|| !peripheral->sync())
	{
		return false; 
	}

	this->readMpuSettings();
	/* MPU is not enabled just return no error */
	return (MPUCTL0 & 0x1) == 0;
}
