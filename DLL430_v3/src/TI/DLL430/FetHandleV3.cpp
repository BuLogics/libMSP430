/*
 * FetHandleV3.h
 *
 * Communication with FET.
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

#include "FetHandleV3.h"
#include "FetControl.h"
#include "DeviceInfo.h"
#include "../../../version.h"
#include "HalExecCommand.h"
#include "ConfigManagerV3.h"
#include "DeviceHandleManagerV3.h"


using namespace TI::DLL430;

const char* FetHandleV3::id = "FetHandleV3";
typedef boost::mutex mutex_type;


FetHandleV3::FetHandleV3 (PortInfo * info)
 : version(VERSION_MAJOR, VERSION_MINOR, VERSION_PATCH, VERSION_BUILD)
 , channel(0)
 , control(0)
 , configManager(0)
 , deviceHandleManager(NULL)
 , myMutex(0)
 , communication(false)
{
	this->channel = dynamic_cast<IoChannel*>(info);
	if (!this->channel)
		return;

	this->control = new FetControl(this->channel);

	if (this->control->hasCommunication())
	{
		communication=true;
		this->myMutex = this->control->getMutex();
		this->deviceHandleManager = new DeviceHandleManagerV3(this);
		this->configManager = new ConfigManagerV3(this);
		if (!this->configManager->init()) 
		{
			delete this->configManager;
			this->configManager = 0;
			delete this->deviceHandleManager;
			this->deviceHandleManager = 0;
		}
	}
}

FetHandleV3::~FetHandleV3 ()
{
	delete configManager;
	configManager = 0;

	delete control;
	control = 0;

	delete deviceHandleManager;
	deviceHandleManager = 0;
}

void FetHandleV3::shutdown()
{
	if (control)
		control->shutdown();
}

bool FetHandleV3::hasCommunication()
{
	return communication;
}

void FetHandleV3::configure (const DeviceInfo* info)
{
	this->control->setObjectDbEntry(info->getObjectId());
}

ConfigManager* FetHandleV3::getConfigManager ()
{
	return this->configManager;
}

FetControl* FetHandleV3::getControl ()
{
	return this->control;
}

DeviceHandleManager* FetHandleV3::getDeviceHandleManager ()
{
	return this->deviceHandleManager;	
} 

bool FetHandleV3::send (HalExecCommand &command)
{
	boost::mutex::scoped_lock lock(*(this->myMutex));
	return this->control->send(command,NULL);
}

bool FetHandleV3::kill(uint8_t respId)
{
	return control->kill(respId);
}

bool FetHandleV3::pauseLoopCmd(unsigned long respId)
{
	return (respId == 0) || control->pauseLoopCmd((uint8_t)respId);
}

bool FetHandleV3::resumeLoopCmd(unsigned long respId)
{
	return (respId == 0) || control->resumeLoopCmd((uint8_t)respId);
}

std::vector<uint8_t> * FetHandleV3::getHwVersion()
{
	return control ? control->getHwVersion() : NULL;
}

std::vector<uint8_t> * FetHandleV3::getSwVersion()
{
	return control ? control->getSwVersion() : NULL;
}

void FetHandleV3::addSystemNotifyCallback(const NotifyCallback& notifyCallback)
{
	if(this->control!=NULL)
		this->control->addSystemNotifyCallback(notifyCallback);
}

bool FetHandleV3::sendHilCommand(HIL_COMMAND command, uint32_t data)
{
	HalExecElement *el = new HalExecElement(ID_HilCommand);
	el->appendInputData32((uint32_t)command);
	el->appendInputData32(data);
	el->appendInputData32(0);
	el->appendInputData32(0);
	el->setOutputSize(0);

	HalExecCommand cmd;
	cmd.elements.push_back(el);

	return send(cmd);
}

uint64_t FetHandleV3::sendJtagShift(HIL_COMMAND shiftType, uint64_t data, long bitSize)
{
	HalExecElement *el = new HalExecElement(ID_HilCommand);
	el->appendInputData32((uint32_t)shiftType);
	el->appendInputData32((uint32_t)(data & 0xFFFFFFFF) );
	el->appendInputData32((uint32_t)(data >> 32) );
	el->appendInputData32(bitSize);
	el->setOutputSize(8);

	HalExecCommand cmd;
	cmd.elements.push_back(el);

	uint64_t result = (uint64_t)-1;
	if ( send(cmd) )
	{
		result  = cmd.elements.at(0).getOutputAt32(0);
		result |= (uint64_t)cmd.elements.at(0).getOutputAt32(4) << 32;
	}
	return result;
}

bool FetHandleV3::setJtagPin(JTAG_PIN pin, bool state)
{
	HalExecElement* el = new HalExecElement(ID_BitSequence);
	el->appendInputData8(1);
	el->appendInputData16(state << pin);
	el->appendInputData16(1 << pin);
	el->appendInputData16(0);

	HalExecCommand cmd;
	cmd.elements.push_back(el);
	return send(cmd);
}
