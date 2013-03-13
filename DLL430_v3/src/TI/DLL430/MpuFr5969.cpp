/*
 * MpuFr5969.cpp
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
#include "MpuFr5969.h"

using namespace TI::DLL430;

MpuFr5969::MpuFr5969(DeviceHandleV3* devHandle, MemoryManager* mm)
	: devHandle(devHandle), mm(mm), MPUCTL0(0), MPUIPC0(0)
{	
}

bool MpuFr5969::isMpuEnabled()
{	
	readMpuSettings();

	return (MPUCTL0 & 0x1) != 0;
}

bool MpuFr5969::calculateMpuBorders()
{
	return true;
}

bool MpuFr5969::readMpuSettings()
{
	/* read out  mpu settings*/
	MemoryArea* peripheral = mm->getMemoryArea("peripheral16bit", 0);
	if (!peripheral)
	{
		return false;
	}
	uint32_t tmp[8] = {0};
	// read control register of MPU protection
	if (!peripheral->read(MPUCTL0_Address, tmp, 8) || !peripheral->sync())
	{
		if (!peripheral->read(MPUCTL0_Address, tmp, 8) || !peripheral->sync())
		{
			return false; 
		}
	}

	// set MPUCTL0
	this->MPUCTL0 = tmp[0] | (tmp[1] << 8);

	return true;
}

bool MpuFr5969::disableMpu()
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
	if (!peripheral->write(MPUCTL0_Address, framCtlKey)|| !peripheral->sync())
	{
		return false; 
	}

	this->readMpuSettings();
	/* MPU is not enabled just return no error */
	return (MPUCTL0 & 0x1) == 0;
}






