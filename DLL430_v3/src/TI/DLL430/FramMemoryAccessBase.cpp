/*
 * FlashMemoryAccessBase.cpp
 *
 * Memory class for accessing flash memory.
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

#include "FramMemoryAccessBase.h"
#include "HalExecCommand.h"
#include "DeviceHandleV3.h"
#include "ConfigManagerV3.h"
#include "FetHandleV3.h"
#include "MpuFr5969.h"
#include "MpuFr5739.h"
#include "ClockCalibration.h"

using namespace TI::DLL430;
using boost::bind;
using boost::shared_ptr;

#define RST_PIN 10
#define STOP_DEVICE 0xA55A
#define MAIN_ERASE_MODE 0x1A1A
#define MASS_ERASE_MODE 0x1B1B
#define LONG_MAILBOX_MODE 0x11

template<class MPU>
FramMemoryAccessBase<MPU>::FramMemoryAccessBase
(
				const std::string& name,
				DeviceHandleV3* devHandle,
				uint32_t start, 
				uint32_t end, 
				uint32_t seg, 
				uint32_t banks, 
				bool mapped,
				const bool isProtected, 
				MemoryManager* mm,
				uint8_t psa	
)
 : MemoryAreaBase(name,devHandle,start,end,seg,banks,mapped, isProtected, psa)
 , mm(mm), funcletToUpload(FuncletCode::NONE), mpu(devHandle, mm)
{
}
template<class MPU>
FramMemoryAccessBase<MPU>::~FramMemoryAccessBase ()
{
}

template<class MPU>
bool FramMemoryAccessBase<MPU>::erase(uint32_t start, uint32_t end, uint32_t block_size, int type)
{
	using boost::shared_ptr;
	using boost::bind;

	// check if valid erase type is used
	if ((type != ERASE_ALL) && (type != ERASE_MAIN) && (type != ERASE_TOTAL)&&(type != ERASE_SEGMENT))
	{
		return false;
	}
	if (block_size < 1)
	{
		return false;
	}

	// get Device RAM parameters for funclet upload
	MemoryArea* ram = mm->getMemoryArea("system", 0);
	if (!ram)
	{
		return false;
	}
	MemoryArea* main = mm->getMemoryArea("main", 0);
	if (!main)
	{
		return false;
	}

	if ((type == ERASE_MAIN) && (main->getStart() <= start) && (main->getEnd() > start)) 
	{
		HalExecCommand cmd;
		cmd.setTimeout(10000);	// overwrite 3 sec default with 10 sec
		HalExecElement* el = new HalExecElement(ID_SetDeviceChainInfo);
		el->appendInputData16(static_cast<uint16_t>(this->devHandle->getDevChainInfo()->getBusId()));
		cmd.elements.push_back(el);

		el = new HalExecElement(this->devHandle->checkHalId(ID_SendJtagMailboxXv2));
		el->appendInputData16(LONG_MAILBOX_MODE);	// Mailbox Mode
		el->appendInputData16(STOP_DEVICE);			// Data 1 Mailbox
		el->appendInputData16(MAIN_ERASE_MODE);		// Data 2 Mailbox
		el->appendInputData16(0x1);					// execute por
		cmd.elements.push_back(el);

		if (!this->devHandle->send(cmd))
		{
			return false;
		}
		ConfigManager* cm = this->devHandle->getFetHandle()->getConfigManager();

		/* assert hard RST/NMI and feed in magic pattern to stop device execution */
		/* thr RST/NMI will remove the register protection */ 
		if(!cm->reset(false, RST_PIN, devHandle->getJtagId()))
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
	else if((type == ERASE_TOTAL) && (main->getStart() <= start) && (main->getEnd() > start))
	{
		HalExecCommand cmd;
		cmd.setTimeout(10000);	// overwrite 3 sec default with 10 sec
		HalExecElement* el = new HalExecElement(ID_SetDeviceChainInfo);
		el->appendInputData16(static_cast<uint16_t>(this->devHandle->getDevChainInfo()->getBusId()));
		cmd.elements.push_back(el);

		el = new HalExecElement(this->devHandle->checkHalId(ID_SendJtagMailboxXv2));
		el->appendInputData16(LONG_MAILBOX_MODE);	// Mailbox Mode
		el->appendInputData16(STOP_DEVICE);			// Data 1 Mailbox
		el->appendInputData16(MASS_ERASE_MODE);		// Data 2 Mailbox
		el->appendInputData16(0x1);					// execute por
		cmd.elements.push_back(el);

		if (!this->devHandle->send(cmd))
		{
			return false;
		}
		ConfigManager* cm = this->devHandle->getFetHandle()->getConfigManager();

		/* assert hard RST/NMI and feed in magic pattern to stop device execution */
		/* thr RST/NMI will remove the register protection */ 
		if(!cm->reset(false, RST_PIN, devHandle->getJtagId()))
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
	else
	{
		if ((type != ERASE_SEGMENT) && (type != ERASE_MAIN))
		{
			return false;
		}
		/* if the  MPU is enabled, disable it to enable memory erase*/
		if(!mpu.readMpuSettings() || !mpu.disableMpu())
		{
			return false;
		}

		if (!uploadFunclet(FuncletCode::ERASE) )
		{
			return false;
		}
		shared_ptr<void> restoreRamOnExit(static_cast<void*>(0), 
										  bind(&FramMemoryAccessBase<MPU>::restoreRam, this));

		//Note the erase on an FRAM device is just a dummy write with 0xFFFF to the device FRAm 
		int32_t erase_address = start;

		const FuncletCode& funclet = devHandle->getFunclet(FuncletCode::ERASE);
	
		const uint32_t eraseType = 0;
		const uint32_t eraseLength = end - start + 1;
		const uint16_t flags = 0x0;
		const uint16_t programStartAddress = ram->getStart() + funclet.programStartOffset();

		HalExecCommand cmd;
		cmd.setTimeout(10000);	// overwrite 3 sec default with 10 sec
		HalExecElement* el = new HalExecElement(ID_SetDeviceChainInfo);
		el->appendInputData16(static_cast<uint16_t>(this->devHandle->getDevChainInfo()->getBusId()));
		cmd.elements.push_back(el);

		el = new HalExecElement(this->devHandle->checkHalId(ID_ExecuteFunclet));
		el->appendInputData16(static_cast<uint16_t>(ram->getStart() & 0xFFFF));
		el->appendInputData16(static_cast<uint16_t>(ram->getSize() & 0xFFFF));
		el->appendInputData16(programStartAddress);
		el->appendInputData32(static_cast<uint32_t>(erase_address));
		el->appendInputData32(eraseLength);
		el->appendInputData16(eraseType);
		el->appendInputData16(flags);
		el->appendInputData16(devHandle->getClockCalibration()->getCal0());
		el->appendInputData16(devHandle->getClockCalibration()->getCal1());

		//Dummy data to trigger execution of erase funclet
		el->appendInputData32(0xDEADBEEF);

		// set value for return length
		el->setOutputSize(2);
		cmd.elements.push_back(el);

		if (!this->devHandle->send(cmd))
		{
			return false;
		}
	}
	return true;
}

template<class MPU>
bool FramMemoryAccessBase<MPU>::doRead (uint32_t address, uint32_t* buffer, size_t count)
{
	MemoryArea* cpu = mm->getMemoryArea("CPU");
	if (!cpu)
		return false;

	uint32_t pc = 0;
	cpu->read(0, &pc, 1);

	bool omitFirst = (address & 0x1);
	if (omitFirst) 
	{
		--address;
		++count;
	}
	bool omitLast = (count & 1);
	if (omitLast) 
	{
		++count;
	}	

	const hal_id readMacro = devHandle->supportsQuickMemRead() ? 
								ID_ReadMemQuick : ID_ReadMemWords;

	HalExecElement* el = new HalExecElement(this->devHandle->checkHalId(readMacro));
	el->appendInputData32(this->getStart() + address);
	el->appendInputData32(static_cast<uint32_t>(count/2));
	el->appendInputData32(pc);

	el->setOutputSize(count);

	ReadElement r(buffer, count, omitFirst, omitLast, 0);
	this->readMap[this->elements.size()] = r;
	this->elements.push_back(el);
	return true;
}

template<class MPU>
bool FramMemoryAccessBase<MPU>::doWrite (uint32_t address, uint32_t* buffer, size_t count)
{
	if (count > this->getSize())
	{
		return false;
	}

	address += this->getStart();

	MemoryArea* ram = mm->getMemoryArea("system");
	if (ram == NULL)
	{
		return false;
	}

	const uint16_t flags = 0;

	//32bit alignment
	const uint32_t alignedAddress = address & 0xfffffffc;
	const int frontPadding = address - alignedAddress;
    const int stubble = (address + count) % 4;
	const int backPadding = (4 - stubble) % 4;

	HalExecElement* el = new HalExecElement(this->devHandle->checkHalId(ID_WriteFramQuickXv2));
	el->appendInputData32(alignedAddress);
	el->appendInputData32( (static_cast<uint32_t>(count) + frontPadding + backPadding)/2 );

	vector<uint32_t> frontBuffer(frontPadding);
	vector<uint32_t> backBuffer(backPadding);

	if(frontPadding != 0)
	{
		mm->read(alignedAddress, &frontBuffer[0], frontPadding);
		mm->sync();
	}

	if(backPadding != 0)
	{
		mm->read(address+count, &backBuffer[0], backPadding);
		mm->sync();
	}

	for (int i = 0; i < frontPadding; ++i) 
	{
		el->appendInputData8(frontBuffer[i]);
	}

	for (size_t i = 0; i < count; ++i)
	{
		if (buffer[i] > 0xFF) 
		{
			delete el;
			return false;
		}
		el->appendInputData8(static_cast<uint8_t>(buffer[i]));
	}
		
	for (int i = 0; i < backPadding; ++i)
	{
		el->appendInputData8(backBuffer[i]);
	}

	this->elements.push_back(el);

	return true;	
}

template<class MPU>
bool FramMemoryAccessBase<MPU>::doWrite (uint32_t address, uint32_t value)
{
	return this->doWrite(address, &value, 1);
}

template<class MPU>
bool FramMemoryAccessBase<MPU>::erase (uint32_t start, uint32_t end)
{
	return erase(start, end, this->getSegmentSize(), 0);
}

template<class MPU>
bool FramMemoryAccessBase<MPU>::erase ()
{
	const uint32_t bank_size = this->getSize() / this->getBanks();

	return erase(this->getStart(), this->getEnd(), bank_size, 1);
}

template<class MPU>
bool FramMemoryAccessBase<MPU>::uploadFunclet(FuncletCode::Type type)
{
	bool success = false;

	const FuncletCode& funclet = devHandle->getFunclet(type);

	funcletToUpload = FuncletCode::NONE;

	if ( !funclet.code() )
	{
		success = true;
		ramBackup.clear();
	}
	else if ( MemoryArea* ram = mm ? mm->getMemoryArea("system", 0) : 0 )
	{	
		if ( funclet.codeSize() <= ram->getSize() )
		{
			if ( mm && mm->getRamPreserveMode() )
			{
				const size_t backupSize = min((size_t)ram->getSize(), 
										   funclet.codeSize() + funclet.maxPayloadSize());
				
				ramBackup.resize( backupSize );
				if (!ram->read(0, &ramBackup[0], ramBackup.size()) || !ram->sync())
					return false;
			}
			else
			{
				ramBackup.clear();
			}

			const uint8_t* code = (uint8_t*)funclet.code();
			const size_t count = funclet.codeSize();

			vector<uint32_t> tmp(count);
			for (size_t i = 0; i < count; ++i)
			{
				tmp[i] = static_cast<uint32_t>(code[i]);
			}
			success = ram->write(0, &tmp[0], count) && ram->sync();
		}
	}
	return success;
}

template<class MPU>
void FramMemoryAccessBase<MPU>::restoreRam()
{
	if (!ramBackup.empty())
	{
		if ( MemoryArea* ram = mm->getMemoryArea("system", 0) )
		{
			size_t count = ramBackup.size();
			vector<uint32_t> tmp(count);
			for (size_t i = 0; i < count; ++i)
				tmp[i] = ramBackup[i];

			ram->write(0, &tmp[0], count) && ram->sync();
		}
	}
}

template<class MPU>
bool FramMemoryAccessBase<MPU>::preSync ()
{
	bool success = true;

	if (funcletToUpload != FuncletCode::NONE)
	{
		success = uploadFunclet(funcletToUpload);
		funcletToUpload = FuncletCode::NONE;
	}
	
	return success;
}

template<class MPU>
bool FramMemoryAccessBase<MPU>::postSync (const HalExecCommand& cmd)
{
	restoreRam();

	for (size_t i = 1; i < cmd.elements.size(); ++i) 
	{
		ReadElement_map::iterator it = this->readMap.find(i-1);
		if (it != this->readMap.end()) 
		{
			ReadElement r = it->second;
			size_t size = r.size;
			if (r.omitLast)
				--size;

			const HalExecElement& el = cmd.elements.at(i);
			for (size_t i = 0, k = (r.omitFirst? 1: 0); k < size; ++k, ++i) 
			{
				r.v_buffer[i] = el.getOutputAt8(k);
			}
			this->readMap.erase(it);
		}
	}

	return true;
}

template class FramMemoryAccessBase<MpuFr5969>;
template class FramMemoryAccessBase<MpuFr5739>;
