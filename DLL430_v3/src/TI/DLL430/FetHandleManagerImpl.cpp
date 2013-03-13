/*
 * FetHandleManagerImpl.cpp
 *
 * Implementation of FetHandle interface.
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

#include "FetHandleManagerImpl.h"
#include "FetHandleV3.h"
#include "DeviceDbManagerExt.h"
#include "Logger.h"
#include "IoChannelFactory.h"
#include "UsbCdcIoChannel.h"

using namespace TI::DLL430;

static FetHandleManagerImpl* manager = 0;
FetHandleManager* FetHandleManager::instance ()
{
	if (manager == 0)
		manager = new FetHandleManagerImpl;

	return manager;
}

FetHandleManagerImpl::FetHandleManagerImpl ()
 : dbm(0)
{
}

FetHandleManagerImpl::~FetHandleManagerImpl ()
{
	manager = 0;
	if (this->dbm)
		delete this->dbm;
}

bool FetHandleManagerImpl::createPortList (char * type, bool update, bool open)
{
	if( !ids.empty() && update )
		clearPortList();

	const string port(type);
	if (port != "USB" && port != "TIUSB" && port != "CDC" && port.substr(0,3) != "LPT")
		ids[port] = new UsbCdcIoChannel(port, "");

	IoChannelFactory::enumeratePorts(ids, type, open);
	return true;
}

PortInfo * FetHandleManagerImpl::getPortElement(std::string name)
{
	PortMap::iterator it = ids.begin();

	//If any USB port was specified, use the first free UIF
	if (name == "USB" || name == "TIUSB" || name=="CDC")
	{
		createPortList("CDC", true, false);

		for (it = ids.begin(); it != ids.end(); ++it)
		{
			if (it->second->getStatus() == PortInfo::freeForUse)
				break;
		}
	}
	else
	{
		it = ids.find(name);
	}
	return (it != ids.end()) ? it->second : NULL;
}

void FetHandleManagerImpl::clearPortList()
{
	for (PortMap::iterator it = ids.begin(); it != ids.end(); ++it)
	{
		if ( PortInfo* currInfo = it->second )
		{
			currInfo->close();
			delete currInfo;
		}
	}
	ids.clear();
}

PortInfo * FetHandleManagerImpl::getPortElement(long idx)
{
	PortMap::iterator it = ids.begin();
	for(int i =0; i < idx; ++i)
	{
		++it;
	}
	return (ids.end() != it) ? it->second : NULL;
}


FetHandle* FetHandleManagerImpl::createFetHandle (PortInfo * portInfo, uint16_t vcc) const
{
	if(portInfo==NULL)
		return NULL;

	auto_ptr<FetHandle> handle(new FetHandleV3(portInfo));

	if (!handle->hasCommunication())
	{
		return 0;
	}

	ConfigManager* c = handle->getConfigManager();
	if (!c)
	{
		return 0;
	}
	if (vcc > 0)
	{
		if (!c->setDeviceVcc(vcc)) 
		{
			return 0;
		}
	}

	return handle.release();
}

void FetHandleManagerImpl::destroyFetHandle (FetHandle* handle) const
{
	delete handle;
}

DeviceDbManager* FetHandleManagerImpl::getDeviceDbManager ()
{
	if (!this->dbm)
		this->dbm = new TemplateDeviceDbManagerExt;
	return this->dbm;
}

