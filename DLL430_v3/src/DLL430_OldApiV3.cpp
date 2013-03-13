/*
 * DLL430_OldApi.cpp
 *
 * Old API interface for IAR - V3 implementation.
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

#ifdef _MSC_VER
#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS //disabling warnings to  use secure c-function versions (e.g. strcpy_s) as not compatible none MS development
#endif
#endif

#include <boost/foreach.hpp>

#include "DLL430_OldApiV3.h"
#include "DeviceDbManagerExt.h"
#include "DeviceInfo.h"
#include "EemMemoryAccess.h"

#include "EM/EmulationManager/IEmulationManager.h"
#include "EM/Trace/ITrace.h"
#include "EM/VariableWatch/IVariableWatch.h"
#include "EM/Sequencer/ISequencer.h"
#include "EM/Exceptions/Exceptions.h"

using namespace TI::DLL430;

#define OLD_API_USB_TYPE "CDC"

#define ERROR_DEF(errorEnum, errorString) errorString,
const char* const errorStrings[] = { ERROR_DEFINITIONS };
#undef ERROR_DEF



DLL430_OldApiV3::DLL430_OldApiV3 ()
	: manager(FetHandleManager::instance())
	, handle(0)
	, errNum(0)
	, singleDevice(0)
	, v2initCalled(false)
	, devInfoFilled(false)
	, devCode(0)
	, notifyCallback(0)
	, varWatch_state(VW_DISABLE)
{
	varWatch_state = VW_DISABLE;

	trace_storage.trAction=TR_FETCH;
	trace_storage.trControl=TR_DISABLE;
	trace_storage.trMode=TR_HISTORY;

	selectedJtagMode=ConfigManager::JTAG_MODE_UNDEF;

	this->debug.state = RUNNING;
	this->debug.jtagReleased = true;
	this->debug.cb.func = 0;
	Logger::instance()->registerLogTarget(this);

	clock_control.ccControl		= CC_DISABLE;
	clock_control.ccGeneralCLK	= DEFAULT_CLKCNTRL;
	clock_control.ccModule		= DEVICE_DEFAULT_MCLKCNTRL0;

	memset(&moduleNameBuffer, 0, sizeof(moduleNameBuffer));

	memset(&sequencer_control, 0, sizeof(sequencer_control));
}

DLL430_OldApiV3::~DLL430_OldApiV3 () 
{
	Close(0);

	if (singleDevice && handle)
	{
		DeviceHandleManager* dhm = handle->getDeviceHandleManager();	
		dhm->destroyDeviceHandle(singleDevice);
		singleDevice = NULL;
	}

	Logger::instance()->deregisterLogTarget(this);

	if (this->handle)
		this->manager->destroyFetHandle(this->handle);

	if (this->manager)
		delete this->manager;
}

void DLL430_OldApiV3::event(DebugEventTarget::EventType e,  uint32_t lParam, uint16_t wParam) 
{
	switch (e) 
	{
	case DebugEventTarget::Lpm5Sleep:
		execNotifyCallback(DEVICE_IN_LPM5_MODE);
		debug.state = LPMX5_MODE;
		break;

	case DebugEventTarget::Lpm5Wakeup:
		if ( DebugManager* db_man = singleDevice->getDebugManager() )
		{
			db_man->syncDeviceAfterLPMx5();
			long dummy = 0;
			State(&dummy,1,&dummy);
		}

		execNotifyCallback(DEVICE_WAKEUP_LPM5_MODE);
		debug.state = LPMX5_WAKEUP;
		break;

	case DebugEventTarget::BreakpointHit:
		if(debug.state == LPMX5_MODE)
		{
			break;
		}		
		if ( DebugManager* db_man = singleDevice->getDebugManager() )
		{
			//With trace/variable watch enabled, give storage events on same trigger time to be reported
			if (trace_storage.trControl == TR_ENABLE)
			{
				boost::this_thread::sleep(boost::get_system_time() + boost::posix_time::milliseconds(100));
			}
			db_man->pausePolling();
		}
		this->debug.state = BREAKPOINT_HIT;

		if (this->debug.cb.func) 
		{
			(*this->debug.cb.func)(

				this->debug.cb.ids.uiMsgIdBreakpoint,
				0,
				0,
				this->debug.cb.clientHandle
			);
		}
		break;

	case DebugEventTarget::Storage:
		if (this->debug.cb.func) 
		{
			(*this->debug.cb.func)(
				this->debug.cb.ids.uiMsgIdStorage,
				lParam,		// # new entries
				1,
				this->debug.cb.clientHandle
			);
		}
		break;

	case DebugEventTarget::VariableWatch:
		watchedVariablesMutex.lock();
		for (map<uint16_t, WatchedVariablePtr>::const_iterator it = watchedVariables.begin(); it != watchedVariables.end(); ++it)
		{
			if (it->second->isValid() && this->debug.cb.func) 
			{
				(*this->debug.cb.func)(
					this->debug.cb.ids.uiMsgIdStorage,
					it->first,
					it->second->value(),
					this->debug.cb.clientHandle
				);
			}
		}
		watchedVariablesMutex.unlock();
		break;
	}
}


void DLL430_OldApiV3::log(TI::DLL430::LogTarget::Severity sev, unsigned int id, const char* message)
{
	switch (sev) {
	case LogTarget::ERR:
	case LogTarget::FATAL:
		this->errNum = id;
		break;

	default:
		break;
	}
}

bool DLL430_OldApiV3::GetNumberOfUsbIfs(long* Number)
{
	assert(this->manager);

	//Make sure there is no active communication when we clear the port list
	this->Close(0);

	// API does not support different USB types here
	// using default
	this->manager->createPortList(OLD_API_USB_TYPE, true, false);
	if(NULL != Number)
	{
		*Number = static_cast<long>(this->manager->getPortNumber());
		return true;
	} 
	else 
	{
		log(LogTarget::ERR, PARAMETER_ERR, "");
		return false;
	}
}

bool DLL430_OldApiV3::GetNameOfUsbIf(long Idx, char** Name, long* Status)
{	
	PortInfo * portinfo = this->manager->getPortElement(Idx);
	if(NULL != portinfo)
	{
		// Copy string to persistent buffer that won't become invalid after init
		port_names.resize( max((long)port_names.size(), Idx+1) );
		strncpy(port_names[Idx].name, portinfo->getName(), sizeof(port_names[Idx].name)-1);

		*Name = port_names[Idx].name;
		*Status = portinfo->getStatus();
		return true;
	} 
	else 
	{
		log(LogTarget::ERR, USB_FET_NOT_FOUND_ERR, "");
		return false;
	}
}

bool DLL430_OldApiV3::Initialize(char* port, long* version)
{
	lastUsedPort = port;
	// if ports are still open, exec a cleanup here
	this->Close(0);
	
	PortInfo *portInfo = this->manager->getPortElement(std::string(port));

	if (!portInfo)
	{
		manager->createPortList(OLD_API_USB_TYPE, true, false);
		portInfo = this->manager->getPortElement(std::string(port));
	}

#ifdef UNIX
	if (!portInfo)
	{
		manager->createPortList(port, true, false);
		portInfo = this->manager->getPortElement(std::string(port));
	}
#endif

	if (NULL == portInfo)
	{
		log(LogTarget::ERR, USB_FET_NOT_FOUND_ERR, "");
		return false;
	}
	else if (portInfo->getStatus() == PortInfo::inUseByAnotherInstance)
	{
		log(LogTarget::ERR, USB_FET_BUSY_ERR, "");
		return false;
	}
	else if ( !portInfo->open() )
	{
		log(LogTarget::ERR, INITIALIZE_ERR, "");
		return false;
	}

	// if this point is reached, a port was found -> start communication now

	this->handle = this->manager->createFetHandle(portInfo);
	if (!this->handle)
	{
		log(LogTarget::ERR, COMM_ERR, "");
		this->Close(0);
		return false;
	}

	if(notifyCallback!=NULL)
		handle->addSystemNotifyCallback( boost::bind(&DLL430_OldApiV3::iNotifyCallback, this, _1, _2, _3, _4) );

	if (version)		// pointer not 0
	{
		long verFw = 0;
		this->FET_GetFwVersion(&verFw);
		if( verFw == handle->getDllVersion().get())
		{
			*version=verFw;
		}
		else
		{
			*version=(-1);
		}
		if (!this->handle->getConfigManager()->checkCoreVersion())
		{
			*version=(-1);
		}
	}

	errNum = NO_ERR;
	return true;
}

bool DLL430_OldApiV3::SetSystemNotfyCallback(SYSTEM_NOTIFY_CALLBACK parSystemNotifyCallback)
{
	notifyCallback=parSystemNotifyCallback;
	
	if(handle && singleDevice)
	{
		handle->addSystemNotifyCallback( boost::bind(&DLL430_OldApiV3::iNotifyCallback, this, _1, _2, _3, _4) );
		
		if ( DebugManager* db_man = singleDevice->getDebugManager() )
		{
			if (db_man->getLpmDebugging())
			{
				db_man->activateJStatePolling(this);
			}
		}
	}

	return true;
}

bool DLL430_OldApiV3::OpenDevice(char* Device, char* Password, long PwLength, long DeviceCode, long setId)
{
	string tmpName(Device);
	if (tmpName == "MSP430C091" || tmpName == "MSP430C092" ||
	    DeviceCode == 0xA55AA55A || DeviceCode == 0x5AA55AA5)
	{
		this->Configure(INTERFACE_MODE, JTAG_IF);
		if(DeviceCode != 0xA55AA55A && DeviceCode != 0x5AA55AA5)
		{
			DeviceCode = 0xDEADBABE;//c092 ROM device
			devCode = 0xDEADBABE; // just for reset function 
		}
		if(DeviceCode == 0xA55AA55A )//L092
		{
			devCode = 0xA55AA55A;
		}
		if(DeviceCode == 0x5AA55AA5 )//C092EMU
		{
			devCode = 0x5AA55AA5;
		}
	}
	if(this->Identify((char*)&devInfo,sizeof(DEVICE_T), setId, Password, PwLength, DeviceCode) == true)
	{
		devInfoFilled=true;
		debug.jtagReleased = false;
		return true;
	}
	return false;
}

bool DLL430_OldApiV3::GetFoundDevice(char* FoundDevice, long count)
{
	count = min((long)sizeof(DEVICE_T), count);
	memcpy(FoundDevice, &devInfo, count);
	return true;
}

bool DLL430_OldApiV3::Close(long vccOff)
{
	if (!this->handle)
		return true;

	bool success = true;

	if ( ConfigManager* cm = this->handle->getConfigManager() )
	{
		if (vccOff)
		{
			ConfigManager* cm = this->handle->getConfigManager();
			if (!cm->setDeviceVcc(0))
			{
				log(LogTarget::ERR, VCC_ERR, "");
				success = false;
			}
		}
		cm->stop();
	}

	this->handle->shutdown();

	if(singleDevice!=NULL)
	{
		DeviceHandleManager* dhm = handle->getDeviceHandleManager();	
		dhm->destroyDeviceHandle(singleDevice);
		singleDevice=NULL;
	}

	this->manager->destroyFetHandle(this->handle);
	this->handle = 0;

	selectedJtagMode=ConfigManager::JTAG_MODE_UNDEF;
	manager->clearPortList();

	config_settings.clear();

	return success;
}


bool DLL430_OldApiV3::Configure(enum CONFIG_MODE mode, long value)
{
	if(handle == NULL)
	{
		log(LogTarget::ERR, INTERFACE_SUPPORT_ERR, "");
		return false;
	}

	// Save the value for later use
	if ( mode != WRITE_EXTERNAL_MEMORY && mode != SET_MDB_BEFORE_RUN && mode != TOTAL_ERASE_DEVICE )
	{
		config_settings[mode] = value;
	}

	switch (mode) 
	{
	case INTERFACE_MODE:
		{
		ConfigManager* cm = this->handle->getConfigManager();
		if (!cm)
		{
			log(LogTarget::ERR, INTERNAL_ERR, "");
			return false;
		}
		enum INTERFACE_TYPE type = (enum INTERFACE_TYPE)value;
		switch (type) 
		{
			case JTAG_IF:
				selectedJtagMode=ConfigManager::JTAG_MODE_4WIRE;
				cm->setJtagMode(ConfigManager::JTAG_MODE_4WIRE);
				break;
			case SPYBIWIRE_IF:
				selectedJtagMode=ConfigManager::JTAG_MODE_SPYBIWIRE;
				cm->setJtagMode(ConfigManager::JTAG_MODE_SPYBIWIRE);
				break;
			case SPYBIWIREJTAG_IF:
				selectedJtagMode=ConfigManager::JTAG_MODE_4AFTER2;
				cm->setJtagMode(ConfigManager::JTAG_MODE_4AFTER2);
				break;
			case AUTOMATIC_IF:
				selectedJtagMode=ConfigManager::JTAG_MODE_AUTOMATIC;
				break;
		}
		}
		break;

	case EMULATION_MODE:
		break;

	case SET_MDB_BEFORE_RUN:
		{
		if(singleDevice == NULL)
		{
			log(LogTarget::ERR, INTERNAL_ERR, "");
			return false;
		}

		DebugManager* db_man = singleDevice->getDebugManager();
		db_man->setOpcode((uint16_t)value);
		}
		break;

	case WRITE_EXTERNAL_MEMORY:
		if (value != 0)
		{
			if(!writeToExternalMemory())
			{
				log(LogTarget::ERR, WRITE_MEMORY_ERR, "");
				return  false;
			}		
		}
		break;

	case TOTAL_ERASE_DEVICE:
		{
			ConfigManager* cm = this->handle->getConfigManager();
			if (!cm)
			{
				log(LogTarget::ERR, INTERNAL_ERR, "");
				return false;
			}

			// execute total erase in SBW2 Mode
			Configure(INTERFACE_MODE, ConfigManager::JTAG_MODE_SPYBIWIRE);	
			cm->totalErase();

			// execute total erase in SBW4 Mode
			Configure(INTERFACE_MODE, ConfigManager::JTAG_MODE_4AFTER2);	
			cm->totalErase();	
		}
		break;

	case UNLOCK_BSL_MODE: 
		lockMemory("boot", value != ENABLE);
		break;

	case LOCKED_FLASH_ACCESS: 
		lockMemory("information", value != ENABLE);
		break;		

	case RAM_PRESERVE_MODE:
		if ( singleDevice ) 
		{
			if ( MemoryManager* mm = singleDevice->getMemoryManager() ){
				mm->setRamPreserveMode( value == ENABLE );
			}
		}
		break;

	case DEBUG_LPM_X:
		if ( singleDevice )	
		{
			if ( DebugManager* dbgManager = singleDevice->getDebugManager() ) 
			{
				dbgManager->setLpmDebugging( value == ENABLE );
				if ( dbgManager->getLpmDebugging() && notifyCallback != 0) 
				{
					dbgManager->activateJStatePolling(this);
				}
			}
		}
		break;

	case JTAG_SPEED:
		{
			ConfigManager* cm = this->handle->getConfigManager();
			if (!cm)
			{
				log(LogTarget::ERR, INTERNAL_ERR, "");
				return false;
			}
			cm->setJtagSpeed((ConfigManager::JTAG_4WIRE_SPEED)value);	
			break;		
		}
	default:
		break;
	}
	return true;
}

bool DLL430_OldApiV3::lockMemory(const string& memoryName, bool lock)
{
	if (singleDevice != NULL)
	{
		MemoryManager* mm = singleDevice->getMemoryManager();
		if (mm != NULL)
		{
			const bool success = mm->lock(memoryName.c_str(), lock);
			if (!success)
			{
				log(LogTarget::ERR, UNLOCK_BSL_ERR, "");
			}

			return success;
		}
	}
	return false;
}

long DLL430_OldApiV3::Error_Number(void)
{
	long ret = this->errNum;

	this->errNum = 0;
	return ret;
}

const char* DLL430_OldApiV3::Error_String(long errorNumber)
{
	if ((errorNumber < 0) || (errorNumber >= INVALID_ERR))
	{
		errorNumber = INVALID_ERR;
	}

	return (errorStrings[errorNumber]);
}

bool DLL430_OldApiV3::GetJtagID(long* JtagId)
{
	ConfigManager* cm = this->handle->getConfigManager();
	if (!cm)
	{
		log(LogTarget::ERR, INTERNAL_ERR, "");
		return false;
	}
	cm->start();
	uint16_t id = cm->start();
	if (id == 0)
	{
		log(LogTarget::ERR, NO_DEVICE_ERR, "");
		return false;
	}
	if (JtagId)
		*JtagId = id;

	return true;
}

bool DLL430_OldApiV3::Identify(char* buffer, long count, long setId, char* Password, long PwLength, long code)
{
	errNum=0;
	if (setId < 0 || !this->handle || setId > (long)manager->getDeviceDbManager()->getMaxId())
	{
		log(LogTarget::ERR, PARAMETER_ERR, "");
		return false;
	}
	ConfigManager* cm = this->handle->getConfigManager();
	if (!cm)
	{
		log(LogTarget::ERR, INTERNAL_ERR, "");
		return false;
	}
	//---------------------find Interface-------------------
	ConfigManager::jtagMode ifMode = ConfigManager::JTAG_MODE_UNDEF;
	if((selectedJtagMode==ConfigManager::JTAG_MODE_UNDEF)||(selectedJtagMode==ConfigManager::JTAG_MODE_AUTOMATIC))
	{
		ifMode = cm->getInterfaceMode();	
		if(ifMode == ConfigManager::JTAG_MODE_UNDEF)
		{
			// Assume 4-wire with JTAG-bits locked. This will cause the MagicPattern to be sent later
			ifMode = ConfigManager::JTAG_MODE_4AFTER2;
		}
		Configure(INTERFACE_MODE, ifMode);
	}
	//------------------end find Interface-------------------
	// start JTAG 
	cm->setDeviceCode(code);
	//Leading "0x" is cut off here
	const int strLen = (Password && PwLength > 0) ? PwLength * 4 : 0;
	const string pwd = (strLen > 0) ? string(Password + 2, strLen) : "";
	cm->setPassword(pwd);

	uint16_t num = cm->start();
	if((short)num == -2)
	{
		log(LogTarget::ERR, WRONG_PASSWORD, "");
		return(false);
	}

	DeviceHandleManager* dhm = handle->getDeviceHandleManager();
	DeviceChainInfoList* dcil = dhm->getDeviceChainInfo();
	
	// test, whether target is attached
	if(dcil->size()<1)
	{
		// return error, if no target is attached
		cm->stop();
		log(LogTarget::ERR, NO_DEVICE_ERR, "");
		return(false);
	}
	// the first device is the only one
	DeviceChainInfoList::iterator itsdcil = dcil->begin();
	
	if(singleDevice!=NULL)
	{	
		dhm->destroyDeviceHandle(singleDevice);
		itsdcil->setInUse(false);
	}
	// save the device handle to work with
	singleDevice = dhm->createDeviceHandle(itsdcil,(uint32_t)code);
	// sanity check
	if(singleDevice==NULL)
	{
		cm->stop();
		log(LogTarget::ERR, NO_DEVICE_ERR, "");
		return(false);
	}
	//attach to running target
	if(setId != 0x0)
	{
		if(0 == singleDevice->getJtagId())// if no jtag id was found the device is locked or lpmx.5
		{
			setId = DEVICE_UNKNOWN;
		}
	}

	if (setId == DEVICE_UNKNOWN) 
	{
		setId = singleDevice->identifyDevice(code);	
		if(setId == -5555)
		{
			log(LogTarget::ERR, FUSE_BLOWN_ERR, "");
			return false;
		}
		if (setId < 0)
		{
			// call magic pattern to get device under control
			switch(singleDevice->magicPatternSend())
			{
			case 0: // JTAG Mode
				ifMode = ConfigManager::JTAG_MODE_4WIRE;
				break;
			case 1: // SBW2 Mode
				ifMode = ConfigManager::JTAG_MODE_SPYBIWIRE;
				break;
			case 2: // SBW4 Mode
				ifMode = ConfigManager::JTAG_MODE_4AFTER2;
				break;
			default:
				ifMode = ConfigManager::JTAG_MODE_UNDEF;
				break;
			}
			Configure(INTERFACE_MODE, ifMode);

			// If the device is locked by a password, unlock it
			if(!pwd.empty())
			{
				cm->start();
			}

			// Re-initialize the device handle
			itsdcil->setInUse(false);
			dhm->destroyDeviceHandle(singleDevice);
			singleDevice = dhm->createDeviceHandle(itsdcil,(uint32_t)code);

			// try to identify the device again
			setId = singleDevice->identifyDevice(code);
			if (setId <= 0)
			{
				log(LogTarget::ERR, DEVICE_UNKNOWN_ERR, "");
				return false;
			}
		}
		this->debug.state = STOPPED;
	} 
	else // this is Attach to running target
	{
		singleDevice->setDeviceId(setId);
	}
	
	clock_control.ccModule=singleDevice->getDebugManager()->getClockModuleSetting();
	clock_control.ccGeneralCLK=singleDevice->getDebugManager()->getClockControlSetting();

	// Apply all configuration parameters
	std::map<enum CONFIG_MODE, long>::const_iterator it = config_settings.begin();
	for( ; it != config_settings.end(); ++it)
	{
		this->Configure(it->first,it->second);
	}

	return this->Device(setId, buffer, count);
}	

bool DLL430_OldApiV3::Identify(char* buffer, long count, long setId)
{
	return this->Identify(buffer, count, setId, NULL, 0, 0);
}

bool DLL430_OldApiV3::Device(long localDeviceId, char* buffer, long count)
{
	if (buffer == 0)
	{
		log(LogTarget::ERR, PARAMETER_ERR, "");
		return false;
	}

	//use smaller value as limit for memcpy
	count = min((long)sizeof(DEVICE_T), count);

	DEVICE_T data;
	::memset(&data, 0, sizeof(data));
	data.endian = 0xaa55;
	data.id = (uint16_t)(localDeviceId & 0xFFFF);

	if ( static_cast<size_t>(localDeviceId) > TemplateDeviceDbManagerExt().getMaxId() )
	{
		log(LogTarget::ERR, PARAMETER_ERR, "");
		::memcpy(buffer, data.buffer, count);
		return false;
	}

	const DeviceInfoPtr info = TemplateDeviceDbManagerExt().queryDb(localDeviceId);

	::strncpy((char*)data.string, info->getDescription(), sizeof(data.string) - 1);
	
	const DeviceInfo::memoryInfo_list_type& memoryInfo = info->getMemoryInfo();
	
	int bits = 16;
	WORD usbRamStart = 0;
	WORD usbRamEnd = 0;
	int ramAreaCounter = 0;
	
	DeviceInfo::memoryInfo_list_type::const_iterator it = memoryInfo.begin();
	const DeviceInfo::memoryInfo_list_type::const_iterator listEnd = memoryInfo.end();
	for (; it != listEnd; ++it)
	{
		const unsigned start = it->offset;
		const unsigned end = it->offset + it->size - 1;

		if (it->name == "CPU")
		{
			bits = it->bits;
		}
		else if (it->name == "main")
		{
			data.mainStart = static_cast<WORD>(start);
			data.mainEnd = end;
		}
		else if (it->name == "information")
		{
			data.infoStart = static_cast<WORD>(start);
			data.infoEnd = static_cast<WORD>(end);
		}
		else if (it->name == "boot")
		{
			data.bslStart = static_cast<WORD>(start);
			data.bslEnd = static_cast<WORD>(end);
		}
		else if (it->name == "lcd")
		{
			data.lcdStart = static_cast<WORD>(start);
			data.lcdEnd = static_cast<WORD>(end);
		}
		else if (it->name == "system" && ramAreaCounter == 0)
		{
			data.ramStart = static_cast<WORD>(start);
			data.ramEnd = static_cast<WORD>(end);
			++ramAreaCounter;
		}
		else if (it->name == "system" && ramAreaCounter == 1)
		{
			data.ram2Start = static_cast<WORD>(start);
			data.ram2End = static_cast<WORD>(end);
			++ramAreaCounter;
		}
		else if (it->name == "usbram")
		{
			usbRamStart = static_cast<WORD>(start);
			usbRamEnd = static_cast<WORD>(end);
		}
	}
	if ( usbRamStart > 0 && usbRamStart < data.ramStart )
		data.ramStart = usbRamStart;

	if ( usbRamEnd > 0 && usbRamEnd > data.ramEnd )
		data.ramEnd = usbRamEnd;

	data.clockControl = info->getClockControl();
	data.nStateStorage = info->getStateStorage();
	data.nCycleCounter = info->getCycleCounter();
	data.nCycleCounterOperations = info->getCycleCounterOperations();
	data.emulation = info->getEmulationLevel();
	data.nRegTriggerOperations = info->getTriggerMask();
	data.nSequencer = info->getMaxSequencerStates();
	data.nBreakpoints = info->getPossibleTrigger(0);
	data.nRegTrigger = info->getPossibleTrigger(1);
	data.nCombinations = info->getPossibleTrigger(2);
	data.nBreakpointsOptions = info->getTriggerOptionsModes();
	data.nBreakpointsDma = info->getTriggerDmaModes();
	data.nBreakpointsReadWrite = info->getTriggerReadWriteModes();	
	data.nRegTriggerOperations = info->getRegTriggerOperations();
	data.TrigerMask = info->getTriggerMask();
	data.vccMinOp = info->minVcc();
	data.vccMaxOp = info->maxVcc();
	data.hasTestVpp = info->hasTestVpp();
	data.HasFramMemroy = info->hasFram();

	if (singleDevice)
	{
		data.jtagId=singleDevice->getJtagId();
		data.deviceIdPtr=singleDevice->getDeviceIdPtr();
		data.eemVersion=singleDevice->getEemVersion();
		
		switch(data.jtagId)
		{
		case 0x91:
			data.cpuArch = CPU_ARCH_XV2;
			break;

		case 0x95:	// L092 special handling
			data.cpuArch = CPU_ARCH_ORIGINAL;
			break;

		default:
			if ( bits == 20 )
				data.cpuArch = CPU_ARCH_X;
			else
				data.cpuArch = CPU_ARCH_ORIGINAL;			
			break;
		}
	}
	::memcpy(buffer, data.buffer, count);	
	return true;
}

bool DLL430_OldApiV3::VCC(long voltage)
{
	if (voltage > 0xFFFF)
	{
		log(LogTarget::ERR, PARAMETER_ERR, "");
		return false;
	}
	
	if (!this->handle)
	{
		log(LogTarget::ERR, INTERNAL_ERR, "");
		return false;
	}

	ConfigManager* cm = this->handle->getConfigManager();
	if (!cm)
	{
		log(LogTarget::ERR, INTERNAL_ERR, "");
		return false;
	}

	if (voltage == 0 || ( voltage >= 1800 && voltage <= 3600 ))
	{
		if (!cm->setDeviceVcc(static_cast<uint16_t>(voltage & 0xFFFF)))
		{
			log(LogTarget::ERR, VCC_ERR, "");
			return false;
		}
	}
	else
	{
		log(LogTarget::ERR, PARAMETER_ERR, "");
		return false;
	}	
	return true;
}

bool DLL430_OldApiV3::GetCurVCCT(long* voltage)
{
	if (!this->handle)
	{
		log(LogTarget::ERR, INTERNAL_ERR, "");
		return false;
	}

	ConfigManager* cm = this->handle->getConfigManager();
	if (!cm)
	{
		log(LogTarget::ERR, INTERNAL_ERR, "");
		return false;
	}
	if (voltage)
		*voltage = static_cast<long>(cm->getDeviceVcc());
	return true;	
}

bool DLL430_OldApiV3::GetExtVoltage(long* voltage, long* state)
{
	if (!this->handle)
	{
		log(LogTarget::ERR, INTERNAL_ERR, "");
		return false;
	}

	ConfigManager* cm = this->handle->getConfigManager();
	if (!cm)
	{
		log(LogTarget::ERR, VCC_ERR, "");
		return false;
	}
	
	const uint16_t extVoltage = cm->getExternalVcc();
	
	if (voltage)
		*voltage = static_cast<long>(cm->getExternalVcc());
	
	if (state) 	
	{
		if (extVoltage < 1000) 
		{ //1V
			*state = NO_EX_POWER;
		} 
		else if (extVoltage < 1700) 
		{ //1.7V
			*state = LOW_EX_POWER;
		} 
		else if (extVoltage < 5600) 
		{ //5.6V
			*state = EX_POWER_OK;
		} 
		else 
		{
			*state = HIGH_EX_POWER;
		}
	}
	return true;	
}

bool DLL430_OldApiV3::Erase(long type, long address, long length)
{
	if(singleDevice==NULL)
	{
		log(LogTarget::ERR, NO_DEVICE_ERR, "");
		return false;
	}

	MemoryManager* mm = singleDevice->getMemoryManager();
	switch (type) {
	case ERASE_SEGMENT:
		if (!mm->erase(address, address+length-1))
		{
			log(LogTarget::ERR, ERASE_ERR, "");
			return false;
		}
		break;

	case ERASE_ALL:
		if (!mm->erase())
		{
			log(LogTarget::ERR, ERASE_ERR, "");
			return false;
		}
		break;

	case ERASE_MAIN:
		{
			MemoryArea* area = mm->getMemoryArea("main");
			if (!area || !area->erase())
			{
				log(LogTarget::ERR, ERASE_ERR, "");
				return false;
			}
		}
		break;
	}
	return true;
}

bool DLL430_OldApiV3::Memory(long address, uint8_t* buf, long count, long rw)
{
	bool status = true;

	if(singleDevice==NULL)
	{
		log(LogTarget::ERR, NO_DEVICE_ERR, "");
		return false;
	}

	MemoryManager* mm = singleDevice->getMemoryManager();

	if (rw == WRITE) 
	{		
		vector<uint32_t> tmp(buf, buf + count);
		status = mm->write(address, &tmp[0], count) && mm->sync();
	} 
	else 
	{
		v_tmp.resize(count);

		//Prefill buffer with pattern for vacant memory
		bool oddAddress = ((address % 2) != 0);
		for (int byte = 0; byte < count; ++byte)
		{
			v_tmp[byte] = oddAddress ? 0x3f : 0xff;
			oddAddress = !oddAddress;
		}

		status = mm->read(address, &v_tmp[0], count) && mm->sync();

		for (long i = 0; i < count; ++i)
			buf[i] = static_cast<uint8_t>(v_tmp[i] & 0xFF);
	}

	if(status==false)
	{
		switch (mm->getLastError())
		{
		case MEMORY_READ_ERROR: log(LogTarget::ERR, READ_MEMORY_ERR, ""); break;
		case MEMORY_WRITE_ERROR: log(LogTarget::ERR, WRITE_MEMORY_ERR, ""); break;
		case MEMORY_LOCKED_ERROR: log(LogTarget::ERR, BSL_MEMORY_LOCKED_ERR, ""); break;
		case MEMORY_UNLOCK_ERROR: log(LogTarget::ERR, UNLOCK_BSL_ERR, ""); break;
		default:
			if(rw == WRITE)
				log(LogTarget::ERR, WRITE_MEMORY_ERR, "");
			else
				log(LogTarget::ERR, READ_MEMORY_ERR, "");
		}		
	}
	return status;
}

bool DLL430_OldApiV3::Secure(void)
{
	const bool success = singleDevice && singleDevice->secure();
	if (!success)
	{
		log(LogTarget::ERR, BLOW_FUSE_ERR, "");
	}
	return success;
}

bool DLL430_OldApiV3::ReadOutFile(long wStart, long wLength, char* lpszFileName, long iFileType)
{
	// parameter check
	if((wStart<0)||(wLength<1)||(lpszFileName==NULL))
	{
		log(LogTarget::ERR, PARAMETER_ERR, "");
		return false;
	}

	if(singleDevice==NULL)
	{
		log(LogTarget::ERR, NO_DEVICE_ERR, "");
		return false;
	}

	// first: read mem
	MemoryManager * m_man = singleDevice->getMemoryManager();
	if (!m_man)
	{
		log(LogTarget::ERR, INTERNAL_ERR, "");
		return false;
	}

	std::vector<uint32_t> buffer(wLength);

	if( !m_man->read(wStart,&buffer[0],wLength) || !m_man->sync())
	{
		log(LogTarget::ERR, READ_MEMORY_ERR, "");
		return false;
	}

	const fileType type = (iFileType == 2) ? INTEL_HEX : TI_TXT;
	
	FileFunc* file = singleDevice->getFileRef(); //Creates filemanager if necessary
	
	const bool success = file->printSeg(lpszFileName, &buffer[0], wStart, wLength, type);
	if (!success)
		log(LogTarget::ERR, FILE_IO_ERR, "");
	
	return success;
}

bool DLL430_OldApiV3::ProgramFile(char* File, long eraseType, long verifyMem)
{
	if(singleDevice==NULL)
	{
		log(LogTarget::ERR, NO_DEVICE_ERR, "");
		return false;
	}

	FileFunc * ff=singleDevice->getFileRef();

	int file_ret=ff->readOpen(File,UNKNOWN);
	switch(file_ret)
	{
	case 0:
		break;

	case -1:
		log(LogTarget::ERR, FILE_OPEN_ERR, "");
		return false;

	case -2:
		log(LogTarget::ERR, FILE_DETECT_ERR, "");
		return false;

	case -3:
		log(LogTarget::ERR, FILE_DATA_ERR, "");
		return false;

	case -4:
		log(LogTarget::ERR, FILE_END_ERR, "");
		return false;

	default:
		log(LogTarget::ERR, INVALID_ERR, "");
		return false;
	}

	if(ff->getFileType()==FILE_ERROR)
	{
		log(LogTarget::ERR, FILE_DETECT_ERR, "");
		return false;
	}

	MemoryManager* mm = singleDevice->getMemoryManager();
	if(mm==NULL)
	{
		log(LogTarget::ERR, NO_DEVICE_ERR, "");
		return false;
	}

	if((eraseType==ERASE_ALL)||(eraseType==ERASE_MAIN))
	{
		if (!this->Erase(eraseType,0,0))
		{
			log(LogTarget::ERR, ERASE_ERR, "");
			return false;
		}
	}

	if (!singleDevice->writeSegments())
	{
		ff->close();
		log(LogTarget::ERR, FILE_IO_ERR, "");
		return false;
	}
	// checksum verify
	if (verifyMem)
	{
		if(!singleDevice->verifySegments())
		{
			ff->close();
			log(LogTarget::ERR, VERIFY_ERR, "");
			return false;
		}
	}
	ff->close();

	if (!mm->flushAll())
	{
		log(LogTarget::ERR, WRITE_MEMORY_ERR, "");
		return false;
	}
	return(true);
}

bool DLL430_OldApiV3::VerifyFile(char* File)
{
	if(singleDevice==NULL)
	{
		log(LogTarget::ERR, NO_DEVICE_ERR, "");
		return false;
	}

	FileFunc * ff=singleDevice->getFileRef();

	// read to buffer
	int file_ret=ff->readOpen(File,UNKNOWN);
	switch(file_ret)
	{
	case 0:
		break;

	case -1:
		log(LogTarget::ERR, FILE_OPEN_ERR, "");
		return false;

	case -2:
		log(LogTarget::ERR, FILE_DETECT_ERR, "");
		return false;

	default:
		log(LogTarget::ERR, INVALID_ERR, "");
		return false;
	}

	// file ok, data loaded?
	if(ff->getFileType()==FILE_ERROR)
	{
		log(LogTarget::ERR, FILE_DETECT_ERR, "");
		return false;
	}
	// compare memory segments with file data
	if(singleDevice->verifySegments()==false)
	{
		ff->close();
		log(LogTarget::ERR, VERIFY_ERR, "");
		return false;
	}
	ff->close();
	return(true);
}

bool DLL430_OldApiV3::VerifyMem(long StartAddr, long Length, uint8_t* DataArray)
{
	bool status = true;
	uint32_t addr = static_cast<uint32_t>(StartAddr);
	size_t len = static_cast<size_t>(Length);
	vector<uint32_t> data(len);

	if(singleDevice==NULL)
	{
		log(LogTarget::ERR, NO_DEVICE_ERR, "");
		return false;
	}

	for (size_t i = 0; i < len; ++i) {
		data[i] = static_cast<uint32_t>(DataArray[i]);
	}
	if (!singleDevice->getMemoryManager()->verify(addr, &data[0], len))
	{
		log(LogTarget::ERR, VERIFY_ERR, "");
		status = false;
	}
	return status;
}

bool DLL430_OldApiV3::EraseCheck(long StartAddr, long Length)
{
	uint32_t addr = static_cast<uint32_t>(StartAddr);
	size_t len = static_cast<size_t>(Length);

	if(singleDevice==NULL)
	{
		log(LogTarget::ERR, NO_DEVICE_ERR, "");
		return false;
	}

	if (!singleDevice->getMemoryManager()->verify(addr, 0, len))
	{
		log(LogTarget::ERR, VERIFY_ERR, "");
		return false;
	}
	return true;
}

bool DLL430_OldApiV3::Reset(long method, long execute, long releaseJTAG)
{	
	if (!method)
	{
		log(LogTarget::ERR, PARAMETER_ERR, "");
		return(false);
	}

	if(singleDevice==NULL)
	{
		log(LogTarget::ERR, NO_DEVICE_ERR, "");
		return(false);
	}
	
	DebugManager* db_man = singleDevice->getDebugManager();
	if(!db_man)
	{
		log(LogTarget::ERR, RESET_ERR, "");
		return(false);
	}
	
	db_man->pausePolling();
	
	if (this->debug.jtagReleased)
	{
		this->debug.jtagReleased = !db_man->reconnectJTAG();
	}

	db_man->pausePolling();
	bool resetSuccessful = false;
	bool restartJtag = false;

	ConfigManager* cm = handle ? handle->getConfigManager() : 0;

	/*-----------------------------------------------------------------------------*/
	if (!(method & FORCE_RESET) || (method & PUC_RESET)) 
	{	
		resetSuccessful = singleDevice->reset();
	}
	/*-----------------------------------------------------------------------------*/	
	if (!resetSuccessful && (method & RST_RESET))
	{
		resetSuccessful = cm && cm->reset(false, 10, this->singleDevice->getJtagId());
		restartJtag = true;
	}

	/*-----------------------------------------------------------------------------*/	
	if (!resetSuccessful && (method & VCC_RESET))
	{
		resetSuccessful = cm && cm->reset(true, 0,this->singleDevice->getJtagId());
		restartJtag = true;
	}

	/*-----------------get control over the device after RST--------------------*/
	if (restartJtag)
	{
		if (!cm || cm->start()!= 0x1)
		{	
			log(LogTarget::ERR, RESET_ERR, "");
			return false;
		}

		if(!singleDevice->reset())
		{	
			log(LogTarget::ERR, RESET_ERR, "");
			return false;
		}
	}

	// if no reset was executed, raise error
	if(!resetSuccessful)
	{
		log(LogTarget::ERR, RESET_ERR, "");
		return false;
	}

	long state;
	long pCPUCycles;

	db_man->resumePolling();
	boost::this_thread::sleep(boost::get_system_time() + boost::posix_time::milliseconds(500));
	db_man->pausePolling();

	if(!State(&state, true, &pCPUCycles))
	{
		log(LogTarget::ERR, RESET_ERR, "");
		return false;
	}

	db_man->resetCycleCounterValue();

	if (devCode == 0x5AA55AA5 || devCode == 0xDEADBABE) // set Programm counter to the correct location for C092 EMU an C092
	{
		long program_start;
		unsigned char word[2];
		
		this->Memory(0xFFFE,word,2,READ);   // read address of program start
		
		program_start = 0x00000000 + (word[1] << 8)+ word[0];
        this->Register(&program_start, PC, WRITE);
	}
	if (devCode == 0xA55AA55A ) // set programm counter to the correct location for L092
	{
		long program_start;
		unsigned char word[2];
		
		this->Memory(0x1C7E,word,2,READ );   // read address of program start
		
		program_start = 0x00000000 + (word[1] << 8)+ word[0];
        this->Register(&program_start, PC, WRITE);
	}

	if (execute) 
	{
		if (!db_man->run(FreeRun))
		{
			log(LogTarget::ERR, RUN_ERR, "");
			return false;
		}
	}
	
	return resetSuccessful;
}


bool DLL430_OldApiV3::ExtRegisters(long address, uint8_t * buffer,long count, long rw)
{
	if(singleDevice==NULL)
	{
		log(LogTarget::ERR, NO_DEVICE_ERR, "");
		return false;
	}

	MemoryArea* extended = singleDevice->getMemoryManager()->getMemoryArea("extended");

	if(extended==NULL)
	{
		log(LogTarget::ERR, INTERNAL_ERR, "");
		return false;
	}

	if (rw == WRITE) 
	{
		if (!extended->write(address,(uint32_t*) buffer ,count) || !extended->sync())
		{
			log(LogTarget::ERR, WRITE_REGISTER_ERR, "");
			return false;
		}
	} 
	else 
	{
		std::vector<uint32_t> v_buffer(count);
		if (!extended->read(address, &v_buffer[0], count) || !extended->sync())
		{
			log(LogTarget::ERR, READ_REGISTER_ERR, "");
			return false;
		}
		if(v_buffer.size()!=count)
		{
			log(LogTarget::ERR, READ_REGISTER_ERR, "");
			return false;
		}
		for(int i=0;i<count;i++)
			buffer[i]=v_buffer[i];
	}

	return true;
}

bool DLL430_OldApiV3::Registers(long* registers, long mask, long rw)
{
	if(singleDevice==NULL)
	{
		log(LogTarget::ERR, NO_DEVICE_ERR, "");
		return false;
	}

	MemoryManager * mm=singleDevice->getMemoryManager();
	if(mm==NULL)
	{
		log(LogTarget::ERR, INTERNAL_ERR, "");
		return false;
	}

	MemoryArea* cpu = mm->getMemoryArea("CPU");
	if (!cpu)
	{
		log(LogTarget::ERR, INTERNAL_ERR, "");
		return false;
	}

	for (uint32_t i = 0; i < cpu->getSize(); ++i)
	{
		if ((mask & (1 << i)) != 0)
		{
			DLL430_OldApiV3::Register(&registers[i] , i, rw);
		}
	}

	return true;
}

bool DLL430_OldApiV3::Register(long* reg, long regNb, long rw)
{
	if(singleDevice==NULL)
	{
		log(LogTarget::ERR, NO_DEVICE_ERR, "");
		return(false);
	}

	MemoryArea* cpu = singleDevice->getMemoryManager()->getMemoryArea("CPU");
	if (!cpu)
	{
		log(LogTarget::ERR, INTERNAL_ERR, "");
		return false;
	}

	if (rw == WRITE) 
	{
		uint32_t tmp = (uint32_t)(*reg);
		if (!cpu->write(regNb, tmp))
		{
			log(LogTarget::ERR, WRITE_REGISTER_ERR, "");
			return false;
		}
	} 
	else 
	{
		uint32_t value;
		if (!cpu->read(regNb, &value, 1))
		{
			log(LogTarget::ERR, READ_REGISTER_ERR, "");
			return false;
		}
		*reg = (long)value;
	}

	return true;
}


bool DLL430_OldApiV3::Run(long mode, long releaseJTAG)
{
	if(this->debug.state == RUNNING)
	{
		log(LogTarget::ERR, THREAD_ACTIVE_ERR, "");
		return false;
	}
	if(singleDevice==NULL)
	{
		log(LogTarget::ERR, NO_DEVICE_ERR, "");
		return false;
	}

	DebugManager* db_man = singleDevice->getDebugManager();
	if (!db_man)
	{
		log(LogTarget::ERR, INTERNAL_ERR, "");
		return false;
	}
	MemoryManager * m_man=singleDevice->getMemoryManager();
	if (!m_man)
	{
		log(LogTarget::ERR, INTERNAL_ERR, "");
		return false;
	}

	uint16_t controlType = FreeRun;
	uint8_t modes=0x00;

	if(mode==RUN_TO_BREAKPOINT)
		controlType|=Stopped;

	switch (mode) 
	{
	case FREE_RUN:
	case RUN_TO_BREAKPOINT:

		this->debug.state = RUNNING;

		if ( !db_man->run(controlType, this, releaseJTAG != 0) )
		{
			log(LogTarget::ERR, RUN_ERR, "");
			return false;
		}

		if ( releaseJTAG != 0 )
		{
			//If we get here, run was successfully releasing JTAG
			debug.jtagReleased = true;
		}
	
		break;

	case SINGLE_STEP:
		{
		uint32_t cycles;
		bool inLpmX5 = db_man->queryLpm5State();
		
		// Query LPMx.5 state: if the processor is in LPMx.5 don't even attempt to singleStep
		if(!inLpmX5)
		{
			if (!db_man->singleStep(&cycles))
			{
				log(LogTarget::ERR, STEP_ERR, "");
				return false;
			}
		}
		else
		{
			return true;
		}
		
		// Query LPMx.5 state again: if the processor stepped into LPMx.5, do not send the callback
		inLpmX5 = db_man->queryLpm5State();
		if(inLpmX5)
		{
			db_man->resumePolling();
		}
		else
		{
			this->debug.state = SINGLE_STEP_COMPLETE;
			if (this->debug.cb.func) 
			{
				(*this->debug.cb.func)(
					this->debug.cb.ids.uiMsgIdSingleStep,
					0,
					cycles,
					this->debug.cb.clientHandle
				);
			}
		}
		}
		break;
	}
	return true;
}

bool DLL430_OldApiV3::State(long* state, long stop, long* pCPUCycles)
{
	if (stop == FALSE)
	{
		log(LogTarget::ERR, PARAMETER_ERR, "");
		return false;
	}

	bool jtagWasReleased = false;

	if (this->debug.jtagReleased)
	{
		if ( ConfigManager* cm = handle->getConfigManager() )
			this->debug.jtagReleased = (cm->start() == 0);

		jtagWasReleased = true;
	}

	if (state) 
	{
		if(this->debug.state >= LPMX5_MODE)
		{
			*state = STOPPED;
		}
		else
		{
			*state = this->debug.state;
		}
	}

	if(singleDevice==NULL)
	{
		log(LogTarget::ERR, NO_DEVICE_ERR, "");
		return(false);
	}


	DebugManager* db_man = singleDevice->getDebugManager();
	if (!db_man)
	{
		log(LogTarget::ERR, INTERNAL_ERR, "");
		return false;
	}

	if (stop == TRUE) 
	{
		const uint8_t actResponse = db_man->getRunEemResponseId();
		if(actResponse > 0)
		{
			if (!handle->kill(actResponse))
			{
				log(LogTarget::ERR, INTERNAL_ERR, "");
				return false;
			}
		}

		if ((this->debug.state==RUNNING) || (this->debug.state==LPMX5_MODE) || (this->debug.state==LPMX5_WAKEUP))
		{	// If device is in LPMx.5, we must also perform the stop to re-gain control of the device
			if (!db_man->stop(jtagWasReleased))
			{
				log(LogTarget::ERR, INTERNAL_ERR, "");
				return false;
			}
		}
		this->debug.state=STOPPED;
	}

	if (pCPUCycles)
		*pCPUCycles = (long)db_man->getCycleCounterValue();

	// find condition to avoid stop while stopped
	if (state) 
	{
		if(this->debug.state >= LPMX5_MODE)
		{
			*state = RUNNING;
		}
		else
		{
			*state = this->debug.state;
		}
	}
	return true;
}


bool DLL430_OldApiV3::CcGetClockNames(long localDeviceId, EemGclkCtrl_t** CcClockNames)
{
	if(singleDevice==NULL)
	{
		log(LogTarget::ERR, NO_DEVICE_ERR, "");
		return false;
	}

	DebugManager * dm=singleDevice->getDebugManager();

	uint32_t size;
	char ** list = dm->getClockStrings(&size);

	if( (list == NULL) || (size != 16) )
	{
		log(LogTarget::ERR, INTERNAL_ERR, "");
		return false;
	}

	*CcClockNames=(EemGclkCtrl_t*)list;

	return true;
}

bool DLL430_OldApiV3::CcGetModuleNames(long localDeviceId, EemMclkCtrl_t** CcModuleNames)
{
	if(singleDevice==NULL)
	{
		log(LogTarget::ERR, NO_DEVICE_ERR, "");
		return false;
	}

	DebugManager * dm=singleDevice->getDebugManager();

	uint32_t size;
	char ** list = dm->getModuleStrings(&size);

	if( (list == NULL) || (size != 32) )
	{
		log(LogTarget::ERR, INTERNAL_ERR, "");
		return false;
	}

	*CcModuleNames=(EemMclkCtrl_t*)list;

	return true;
}


bool DLL430_OldApiV3::EEM_Init(
	MSP430_EVENTNOTIFY_FUNC callback,
	long clientHandle,
	MessageID_t* pMsgIdBuffer
)
{
	if(singleDevice==NULL)
	{
		log(LogTarget::ERR, NO_DEVICE_ERR, "");
		return false;
	}

	DebugManager * dm=singleDevice->getDebugManager();

	this->debug.cb.func = callback;
	this->debug.cb.clientHandle = clientHandle;
	this->debug.cb.ids.uiMsgIdBreakpoint = pMsgIdBuffer->uiMsgIdBreakpoint;
	this->debug.cb.ids.uiMsgIdSingleStep = pMsgIdBuffer->uiMsgIdSingleStep;
	this->debug.cb.ids.uiMsgIdStorage = pMsgIdBuffer->uiMsgIdStorage;
	this->debug.cb.ids.uiMsgIdState = pMsgIdBuffer->uiMsgIdState;
	this->debug.cb.ids.uiMsgIdWarning = pMsgIdBuffer->uiMsgIdWarning;
	this->debug.cb.ids.uiMsgIdCPUStopped = pMsgIdBuffer->uiMsgIdCPUStopped;

	try
	{
		singleDevice->getEmulationManager()->writeConfiguration();
	}
	catch(const EM_Exception&)
	{
		//error writing to eem register
	}

	dm->initEemRegister();

	v2initCalled = true;

	return true;
}


TriggerConditionPtr DLL430_OldApiV3::triggerConditionFromBpParameter(EmulationManagerPtr emuManager, BpParameter_t* bpBuffer)
{
	TriggerConditionManagerPtr tcManager = emuManager->getTriggerConditionManager();

	TriggerConditionPtr triggerCondition;

	switch (bpBuffer->bpMode)
	{
	case BP_CODE:
		triggerCondition = tcManager->createInstructionAddressCondition(bpBuffer->lAddrVal);
		break;

	case BP_RANGE:
		if (bpBuffer->lAddrVal > bpBuffer->lRangeEndAdVa || bpBuffer->bpType == BP_REGISTER)
			throw EM_TriggerParameterException();
		
		if (bpBuffer->bpType == BP_MDB)
		{
			triggerCondition = tcManager->createDataRangeCondition(bpBuffer->lAddrVal, bpBuffer->lRangeEndAdVa,
																	0xffffffff, 0xffffffff,
																	(AccessType)bpBuffer->bpAccess,
																	bpBuffer->bpRangeAction == BP_OUTSIDE);
		}
		else
		{
			triggerCondition = tcManager->createAddressRangeCondition(bpBuffer->lAddrVal, bpBuffer->lRangeEndAdVa,
																		0xffffffff, 0xffffffff,
																		(AccessType)bpBuffer->bpAccess,
																		bpBuffer->bpRangeAction == BP_OUTSIDE);
		}
		break;

	case BP_COMPLEX:	
		if (bpBuffer->bpType == BP_REGISTER)
		{
			triggerCondition = tcManager->createRegisterCondition((uint8_t)bpBuffer->lReg, bpBuffer->lAddrVal, 
																	bpBuffer->lMask, (ComparisonOperation)bpBuffer->bpOperat);
		}
		else
		{
			if (bpBuffer->bpType == BP_MDB)
			{
				triggerCondition = tcManager->createDataValueCondition(bpBuffer->lAddrVal, bpBuffer->lMask,
																		(AccessType)bpBuffer->bpAccess,
																		(ComparisonOperation)bpBuffer->bpOperat);
			}
			else
			{
				triggerCondition = tcManager->createDataAddressCondition(bpBuffer->lAddrVal, bpBuffer->lMask,
																		(AccessType)bpBuffer->bpAccess,
																		(ComparisonOperation)bpBuffer->bpOperat);
			}

			if (bpBuffer->bpCondition == BP_COND)
			{
				DataValueConditionPtr dataCond = tcManager->createDataValueCondition(bpBuffer->lCondMdbVal, bpBuffer->lCondMask,
																					(AccessType)bpBuffer->bpCondAccess,
																					(ComparisonOperation)bpBuffer->bpCondOperat);
				triggerCondition->combine(dataCond);
			}
		}
		break;

	default:
		break;
	}

	return triggerCondition;
}


void DLL430_OldApiV3::addBreakpointsAndStorage(EmulationManagerPtr emuManager, TriggerConditionPtr triggerCondition, 
								BpAction_t reactions, uint16_t handle)
{
	if (triggerCondition)
	{
		//Collect all triggers (including those without configured reactions)
		triggers[handle].set(triggerCondition);

		if (reactions & BP_BRK)
		{
			breakpoints[handle].set( emuManager->getBreakpointManager()->createBreakpoint(triggerCondition) );
		}

		if (reactions & BP_STO)
		{
			traceTriggers[handle].set(triggerCondition);
		}
	}
}


void DLL430_OldApiV3::updateStorageConfiguration(EmulationManagerPtr emuManager)
{
	if (emuManager->hasTrace())
	{
		typedef pair<long, TableEntry<TriggerConditionPtr> > MapEntry;

		TracePtr trace = emuManager->getTrace();
	
		trace->clearTriggerConditions();
		BOOST_FOREACH(const MapEntry& entry, traceTriggers)
		{
			if (entry.second.inUse())
			{
				trace->addTriggerCondition(entry.second.value());
			}
		}
	}
}


bool DLL430_OldApiV3::EEM_SetBreakpoint(uint16_t* bpHandle, BpParameter_t* bpBuffer)
{
	if (singleDevice==NULL)
	{
		log(LogTarget::ERR, NO_DEVICE_ERR, "");
		return false;
	}

	if ( bpHandle == NULL || bpBuffer == NULL || (bpBuffer->bpMode == BP_CLEAR && *bpHandle == 0))
	{
		log(LogTarget::ERR, PARAMETER_ERR, "");
		return false;
	}

	bool success = false;

	try
	{
		EmulationManagerPtr emuManager = singleDevice->getEmulationManager();

		if (*bpHandle != 0)
		{
			//Special case handling if trigger is part of a combination
			map<uint16_t, vector<uint16_t> >::iterator it;
			for (it = triggerCombinations.begin(); it != triggerCombinations.end(); ++it)
			{
				vector<uint16_t> combinedHandles = it->second;
				if (find(it->second.begin(), it->second.end(), *bpHandle) != it->second.end())
				{
					uint16_t combinationHandle = it->first;
					
					//If part of a combination is removed, simply break up the combination
					if (bpBuffer->bpMode == BP_CLEAR)
					{
						EEM_SetCombineBreakpoint(CB_CLEAR, combinedHandles.size(), &combinationHandle, &combinedHandles[0]);
					}
					//If the breakpoint is changed, trigger a reconstruction of the combination with new parameters
					else
					{
						it->second.clear();
						bp_storage[*bpHandle] = *bpBuffer;
						return EEM_SetCombineBreakpoint(CB_SET, combinedHandles.size(), &combinationHandle, &combinedHandles[0]);						
					}
				}
			}


			triggers[*bpHandle].clear();
			breakpoints[*bpHandle].clear();
			traceTriggers[*bpHandle].clear();
		}
		else
		{
			*bpHandle = 1;
			while (triggers[*bpHandle].inUse())
				++(*bpHandle);
		}


		if (bpBuffer->bpMode == BP_CLEAR)
		{
			bp_storage.erase(*bpHandle);
		}

		else
		{
			TriggerConditionPtr triggerCondition = triggerConditionFromBpParameter(emuManager, bpBuffer);

			addBreakpointsAndStorage(emuManager, triggerCondition, bpBuffer->bpAction, *bpHandle);
	
			bp_storage[*bpHandle] = *bpBuffer;
		}

		//Reconfigure all trace triggers
		updateStorageConfiguration(emuManager);

		emuManager->writeConfiguration();

		success = true;
	}
	catch (const EM_Exception& e)
	{
		log(LogTarget::ERR, e.errorCode(), e.what());
	}

	return success;
}


bool DLL430_OldApiV3::EEM_GetBreakpoint(uint16_t wBpHandle, BpParameter_t* pBpDestBuffer)
{
	if(pBpDestBuffer==NULL)
	{
		log(LogTarget::ERR, PARAMETER_ERR, "");
		return false;
	}

	map<uint16_t,BpParameter_t>::iterator it = bp_storage.find(wBpHandle);

	if( it == bp_storage.end() )
	{
		log(LogTarget::ERR, PARAMETER_ERR, "");
		return false;
	}

	*pBpDestBuffer = it->second;

	return true;
}



bool DLL430_OldApiV3::EEM_SetCombineBreakpoint(CbControl_t control, uint16_t count, uint16_t* cbHandle, uint16_t* bpHandle)
{
	if (singleDevice == NULL)
	{
		log(LogTarget::ERR, NO_DEVICE_ERR, "");
		return false;
	}

	if (bpHandle == NULL || cbHandle == NULL ||	(control == CB_SET && count < 2))
    {
		log(LogTarget::ERR, PARAMETER_ERR, "");
		return false;
    }

	bool success = false;

	try
	{
		EmulationManagerPtr emuManager = singleDevice->getEmulationManager();
		
		if (control == CB_SET)
		{
			//Combination handle is handle of first combined trigger
			*cbHandle = bpHandle[0];
			if (!triggerCombinations[*cbHandle].empty())
				return false;

			//Combine: replace existing breakpoints/watched variables with placeholders, create new one based on bp_storage
			for (int i = 0; i < count; ++i)
			{
				triggers[ bpHandle[i] ].reserveSlot();
				breakpoints[ bpHandle[i] ].reserveSlot();
				traceTriggers[ bpHandle[i] ].reserveSlot();
			}

			TriggerConditionPtr triggerCondition = triggerConditionFromBpParameter(emuManager, &bp_storage[bpHandle[0]]);

			for (int i = 1; i < count; ++i)
			{
				triggerCondition->combine( triggerConditionFromBpParameter(emuManager, &bp_storage[bpHandle[i]]) );
			}

			addBreakpointsAndStorage(emuManager, triggerCondition, bp_storage[bpHandle[0]].bpAction, bpHandle[0]);

			triggerCombinations[*cbHandle] = vector<uint16_t>(bpHandle, bpHandle + count);
		}

		//Destroy combination, recreate separate breakpoints/watched variables
		if (control == CB_CLEAR)
		{
			vector<uint16_t> combinedHandles = triggerCombinations[*cbHandle];
			triggerCombinations[*cbHandle].clear();

			BOOST_FOREACH(uint16_t handle, combinedHandles)
			{
				EEM_SetBreakpoint(&handle, &bp_storage[handle]);
			}
		}

		//Reconfigure all storage triggers
		updateStorageConfiguration(emuManager);

		emuManager->writeConfiguration();

		success = true;
	}
	catch(const EM_Exception& e)
	{
		log(LogTarget::ERR, e.errorCode(), e.what());
	}

	return success;
}

bool DLL430_OldApiV3::EEM_GetCombineBreakpoint(uint16_t cbHandle, uint16_t* count, uint16_t* bpHandle)
{
	if (singleDevice == NULL)
	{
		log(LogTarget::ERR, NO_DEVICE_ERR, "");
		return false;
	}

	if ((count == NULL) || (bpHandle == NULL))
	{
		log(LogTarget::ERR, PARAMETER_ERR, "");
		return false;
	}

	*count = 0;

	BOOST_FOREACH(long id, triggerCombinations[cbHandle])
	{
		bpHandle[(*count)++] = (uint16_t)id;
	}

	return true;
}

bool DLL430_OldApiV3::EEM_SetTrace(TrParameter_t* trBuffer)
{
	if(singleDevice==NULL)
	{
		log(LogTarget::ERR, NO_DEVICE_ERR, "");
		return false;
	}

	bool success = false;

	try
	{
		EmulationManagerPtr emuManager = singleDevice->getEmulationManager();
		TracePtr trace = emuManager->getTrace();

		trace_storage.trAction = trBuffer->trAction;
		trace_storage.trControl = trBuffer->trControl;
		trace_storage.trMode = trBuffer->trMode;

		switch(trBuffer->trAction)
		{
		case TR_FETCH: 
			trace->setStoreOnInstructionFetch();
			break;

		case TR_ALL_CYCLE:
			trace->setStoreOnClock();
			break;

		default: break;
		}


		switch(trBuffer->trMode)
		{
		case TR_HISTORY:
			trace->setStartOnTrigger(false);
			trace->setStopOnTrigger(true);
			trace->setStoreContinuously();
			break;

		case TR_FUTURE:		
			trace->setStartOnTrigger(true);
			trace->setStopOnTrigger(false);
			trace->setStoreUntilFull();
			break;

		case TR_SHOT:
			trace->setStartOnTrigger(false);
			trace->setStopOnTrigger(false);
			trace->setStoreUntilFull();
			break;

		case TR_COLLECT:
			trace->setStartOnTrigger(false);
			trace->setStopOnTrigger(false);
			trace->setStoreOnTrigger();
			trace->setStoreUntilFull();
			break;

		default: break;
		}


		switch(trBuffer->trControl)
		{
		case TR_ENABLE:	
			trace->enable();
			singleDevice->getDebugManager()->startStoragePolling();
			break;

		case TR_DISABLE: 
			trace->disable();
			singleDevice->getDebugManager()->stopStoragePolling();
			break;

		default: break;
		}

		//Changed settings always require a reset
		trace->reset();

		emuManager->writeConfiguration();

		success = true;
	}
	catch(const EM_Exception& e)
	{
		log(LogTarget::ERR, e.errorCode(), e.what());
	}

	return success;
}

bool DLL430_OldApiV3::EEM_GetTrace(TrParameter_t* trDestBuffer)
{
	trDestBuffer->trAction = trace_storage.trAction;
	trDestBuffer->trControl = trace_storage.trControl;
	trDestBuffer->trMode = trace_storage.trMode;
	return true;
}

bool DLL430_OldApiV3::EEM_ReadTraceBuffer(TraceBuffer_t* destTraceBuffer)
{
	if (destTraceBuffer == NULL)
	{
		log(LogTarget::ERR, PARAMETER_ERR, "");
		return false;
	}

	if(singleDevice==NULL)
	{
		log(LogTarget::ERR, NO_DEVICE_ERR, "");
		return false;
	}

	try
	{
		const TraceBuffer& traceBuffer = singleDevice->getEmulationManager()->getTrace()->getTraceData();

		const size_t count = min((size_t)8, traceBuffer.size());
		for (size_t i = 0; i < count; ++i)
		{
			destTraceBuffer[i].lTrBufMAB = traceBuffer[i].mab;
			destTraceBuffer[i].lTrBufMDB = traceBuffer[i].mdb;
			destTraceBuffer[i].wTrBufCNTRL = traceBuffer[i].ctl;
		}
	}
	catch (const EM_Exception& e)
	{
		log(LogTarget::ERR, e.errorCode(), e.what());
		return false;
	}

	return true;
}

bool DLL430_OldApiV3::EEM_ReadTraceData(TraceBuffer_t* destTraceBuffer, unsigned long *count)
{
	if (destTraceBuffer == NULL)
	{
		log(LogTarget::ERR, PARAMETER_ERR, "");
		return false;
	}

	if(singleDevice==NULL)
	{
		log(LogTarget::ERR, NO_DEVICE_ERR, "");
		return false;
	}

	bool success = false;

	try
	{
		const TraceBuffer& traceBuffer = singleDevice->getEmulationManager()->getTrace()->getTraceData();

		*count = min(*count, (unsigned long)traceBuffer.size());
		for (size_t i = 0; i < *count; ++i)
		{
			destTraceBuffer[i].lTrBufMAB = traceBuffer[i].mab;
			destTraceBuffer[i].lTrBufMDB = traceBuffer[i].mdb;
			destTraceBuffer[i].wTrBufCNTRL = traceBuffer[i].ctl;
		}
		success = true;
	}
	catch (const EM_Exception& e)
	{
		log(LogTarget::ERR, e.errorCode(), e.what());
	}

	return success;
}


bool DLL430_OldApiV3::EEM_RefreshTraceBuffer(void)
{
	if(singleDevice==NULL)
	{
		log(LogTarget::ERR, NO_DEVICE_ERR, "");
		return false;
	}

	bool success = false;

	try
	{
		EmulationManagerPtr emuManager = singleDevice->getEmulationManager();
		emuManager->getTrace()->reset();
	
		emuManager->writeConfiguration();

		success = true;
	}
	catch(const EM_Exception& e)
	{
		log(LogTarget::ERR, e.errorCode(), e.what());
	}

	return success;
}

bool DLL430_OldApiV3::EEM_SetVariableWatch(VwEnable_t vwEnable)
{
	if(singleDevice==NULL)
	{
		log(LogTarget::ERR, NO_DEVICE_ERR, "");
		return false;
	}

	bool success = false;

	try
	{
		EmulationManagerPtr emuManager = singleDevice->getEmulationManager();
		VariableWatchPtr varWatch = emuManager->getVariableWatch();

		if (vwEnable == VW_ENABLE)
		{
			varWatch->enable();
			singleDevice->getDebugManager()->startStoragePolling();
		}
		else
		{
			varWatch->disable();
			singleDevice->getDebugManager()->stopStoragePolling();
		}
		varWatch_state = vwEnable;

		emuManager->writeConfiguration();

		success = true;
	}
	catch (const EM_Exception& e)
	{
		log(LogTarget::ERR, e.errorCode(), e.what());
	}

	return success;
}


bool DLL430_OldApiV3::EEM_GetVariableWatch(VwEnable_t* vwEnable, VwResources_t* vwDestBuffer)
{
	*vwEnable = varWatch_state;

	for(int i = 0; i < 8; ++i)
	{
		vwDestBuffer[i].vwHandle = vw_storage[i].vwHandle;
		vwDestBuffer[i].lAddr = vw_storage[i].lAddr;
		vwDestBuffer[i].vwDataType = vw_storage[i].vwDataType;
	}
	
	return true;
}


bool DLL430_OldApiV3::EEM_SetVariable(uint16_t* vwHandle, VwParameter_t* vwBuffer)
{
	// Check provided parameter
	if (vwHandle == NULL || vwBuffer == NULL)
	{
		log(LogTarget::ERR, PARAMETER_ERR, "");
		return false;
	}

	if(singleDevice==NULL)
	{
		log(LogTarget::ERR, NO_DEVICE_ERR, "");
		return false;
	}

	bool success = false;

	try
	{
		EmulationManagerPtr emuManager = singleDevice->getEmulationManager();
		VariableWatchPtr varWatch = emuManager->getVariableWatch();

		if (vwBuffer->vwControl == VW_CLEAR)
		{
			boost::mutex::scoped_lock lock(watchedVariablesMutex);
			watchedVariables.erase(*vwHandle);
			vw_storage.erase(*vwHandle);
		}

		if (vwBuffer->vwControl == VW_SET)
		{
			boost::mutex::scoped_lock lock(watchedVariablesMutex);

			*vwHandle = 0;
			while (watchedVariables[*vwHandle])
				++(*vwHandle);

			uint32_t bits = 8;
			if (vwBuffer->vwDataType == VW_16)
				bits = 16;

			if (vwBuffer->vwDataType == VW_32)
				bits = 32;

			TriggerConditionManagerPtr tcManager = emuManager->getTriggerConditionManager();
			watchedVariables[*vwHandle] = varWatch->createWatchedVariable(vwBuffer->lAddr, bits, tcManager);

			VwResources_t resource = {*vwHandle, vwBuffer->lAddr, vwBuffer->vwDataType};
			vw_storage[*vwHandle] = resource;
		}

		emuManager->writeConfiguration();

		success = true;
	}
	catch (const EM_Exception& e)
	{
		log(LogTarget::ERR, e.errorCode(), e.what());
	}

	return success;
}

bool DLL430_OldApiV3::EEM_SetClockControl(CcParameter_t* pCcBuffer)
{
	if(pCcBuffer==NULL)
	{
		log(LogTarget::ERR, PARAMETER_ERR, "");
		return false;
	}
	if(singleDevice==NULL)
	{
		log(LogTarget::ERR, NO_DEVICE_ERR, "");
		return false;
	}
	// Check clock control emulation on device
	if ( singleDevice->getDebugManager()->getClockControl() < 1 )
	{
		log(LogTarget::ERR, INTERNAL_ERR, "");
		return false;
	}

	uint16_t cmp_general = DEFAULT_CLKCNTRL;
	uint16_t cmp_module = singleDevice->getDebugManager()->getClockModuleDefaultSetting();

	if(pCcBuffer->ccControl == CC_ENABLE)
	{
		cmp_general = pCcBuffer->ccGeneralCLK;
		cmp_module = pCcBuffer->ccModule;
	}

	const uint8_t clockControlType = singleDevice->getDebugManager()->getClockControl();
	bool needReset = false;

	//Only applies to Cpu/CpuX devices
	if ( (clock_control.ccGeneralCLK != cmp_general) && (singleDevice->getJtagId() == 0x89))
	{
		singleDevice->getDebugManager()->eemWriteRegister(GENCLKCTRL, cmp_general);
		needReset = true;
	}

	if ( (clock_control.ccModule != cmp_module) && (clockControlType == GccExtended))
	{
		singleDevice->getDebugManager()->eemWriteRegister(MODCLKCTRL0, cmp_module);
		needReset = true;
	}

	// PUC reset
	if (needReset && !singleDevice->reset())
	{
		log(LogTarget::ERR, RESET_ERR, "");
		return false;
	}
	
	clock_control.ccControl = pCcBuffer->ccControl;
	clock_control.ccGeneralCLK = cmp_general;
	clock_control.ccModule = cmp_module;

	return true;
}

bool DLL430_OldApiV3::EEM_GetClockControl(CcParameter_t* pCcDestBuffer)
{
	if(pCcDestBuffer==NULL)
	{
		log(LogTarget::ERR, PARAMETER_ERR, "");
		return false;
	}

	memcpy(pCcDestBuffer,&clock_control,sizeof(CcParameter_t));

	return true;
}


bool DLL430_OldApiV3::EEM_SetSequencer(SeqParameter_t* seqBuffer)
{
	if(singleDevice==NULL)
	{
		log(LogTarget::ERR, NO_DEVICE_ERR, "");
		return false;
	}

	bool success = false;

	try
	{
		EmulationManagerPtr emuManager = singleDevice->getEmulationManager();

		SequencerPtr sequencer = emuManager->getSequencer();

		if (seqBuffer->seqControl == SEQ_DISABLE)
		{
			sequencer->disable();
		}
		else
		{
			sequencer->enable();
		}

		sequencer_control = *seqBuffer;

		sequencer->clearAllTransitions();
		sequencer->clearResetTrigger();
		sequencer->clearReactions();

		for (uint8_t i = 0; i < 4; ++i)
		{
			if (seqBuffer->wHandleStateX[i] != 0)
			{
				sequencer->setTransition(i, 0, seqBuffer->seqNextStateX[i], triggers[ seqBuffer->wHandleStateX[i] ].value());
			}

			if (seqBuffer->wHandleStateY[i] != 0)
			{
				sequencer->setTransition(i, 1, seqBuffer->seqNextStateY[i], triggers[ seqBuffer->wHandleStateY[i] ].value());
			}
		}

		//Configure sequencer to reset on specified trigger
		if (seqBuffer->wHandleRstTrig != 0)
		{
			sequencer->setResetTrigger(triggers[seqBuffer->wHandleRstTrig].value());
		}
		
		//Set sequencer actions on reaching final state
		if (seqBuffer->bpAction & BP_BRK)
		{
			sequencer->addReaction(TR_BREAK);
		}
		if (seqBuffer->bpAction & BP_STO)
		{
			sequencer->addReaction(TR_STATE_STORAGE);
		}


		emuManager->writeConfiguration();

		success = true;
	}
	catch (const EM_Exception& e)
	{
		log(LogTarget::ERR, e.errorCode(), e.what());
	}

	return success;
}

bool DLL430_OldApiV3::EEM_GetSequencer(SeqParameter_t* seqDestBuffer)
{
	if (seqDestBuffer == NULL)
	{
		log(LogTarget::ERR, PARAMETER_ERR, "");
		return false;
	}

	*seqDestBuffer = sequencer_control;
	return true;
}

bool DLL430_OldApiV3::EEM_ReadSequencerState(SeqState_t* pSeqState)
{
	if (singleDevice == NULL)
	{
		log(LogTarget::ERR, NO_DEVICE_ERR, "");
		return false;
	}

	if (sequencer_control.seqControl == SEQ_DISABLE)
	{
		log(LogTarget::ERR, SEQ_ENABLE_ERR, "");
		return false;
	}

	bool success = false;

	try
	{
		*pSeqState = (SeqState)singleDevice->getEmulationManager()->getSequencer()->readCurrentState();

		success = true;
	}
	catch (const EM_Exception& e)
	{
		log(LogTarget::ERR, e.errorCode(), e.what());
	}

	return success;
}

bool DLL430_OldApiV3::FET_SelfTest(long count, uint8_t* buffer)
{
	return true;
}

bool DLL430_OldApiV3::FET_SetSignals(long SigMask, long SigState)
{
	log(LogTarget::ERR, INTERFACE_SUPPORT_ERR, "");	
	return false;
}

bool DLL430_OldApiV3::FET_Reset(void)
{
	log(LogTarget::ERR, INTERFACE_SUPPORT_ERR, "");
	return false;
}

bool DLL430_OldApiV3::FET_I2C(long address, uint8_t* buffer, long count, long rw)
{
	log(LogTarget::ERR, INTERFACE_SUPPORT_ERR, "");
	return false;
}

bool DLL430_OldApiV3::FET_EnterBootloader(void)
{
	log(LogTarget::ERR, INTERFACE_SUPPORT_ERR, "");
	return false;
}

bool DLL430_OldApiV3::FET_ExitBootloader(void)
{
	log(LogTarget::ERR, INTERFACE_SUPPORT_ERR, "");
	return false;
}

bool DLL430_OldApiV3::FET_GetFwVersion(long* version)
{
	if(handle==NULL)
	{
		log(LogTarget::ERR, INTERNAL_ERR, "");
		return false;
	}

	if (version)
		*version = this->handle->getConfigManager()->getBiosVersion().get();

	return true;
}


bool DLL430_OldApiV3::FET_GetHwVersion(uint8_t** version, long* count)
{
	if(handle==NULL)
	{
		log(LogTarget::ERR, INTERNAL_ERR, "");
		return false;
	}

	std::vector<uint8_t> * hwId=this->handle->getHwVersion();

	static uint8_t v[4] = {0,0,0,0};

	if((version==NULL)||(count==NULL))
	{
		log(LogTarget::ERR, PARAMETER_ERR, "");
		return false;
	}

	if( hwId->size() < 4 )
	{
		*version = (uint8_t*)v;
		*count = 4;
		log(LogTarget::ERR, INTERNAL_ERR, "");
		return false;
	}
	v[0]=hwId->at(0);
	v[1]=hwId->at(1);
	v[2]=hwId->at(2);
	v[3]=hwId->at(3);

	*version = (uint8_t*)v;
	*count = 4;
	return true;
}

bool DLL430_OldApiV3::FET_FwUpdate(
	char* lpszFileName,
	DLL430_FET_NOTIFY_FUNC callback,
	long clientHandle)
{
	this->errNum = 0;

	if (handle==NULL)
	{
		log(LogTarget::ERR, INTERNAL_ERR, "");
		return false;
	}

	ConfigManager *config = handle->getConfigManager();
	bool success = config->firmWareUpdate(lpszFileName, boost::bind(callback, _1, _2, _3, clientHandle));
	
	//Only perform retries when not using user specified file
	int retries = 2;
	while (!success && retries--)
	{
		long version = 0;
		vector<char> nonConstPortName(lastUsedPort.c_str(), lastUsedPort.c_str() + lastUsedPort.length() + 1);
		if (Initialize(&nonConstPortName[0], &version))
		{
			//Initialize will invalidate the old config manager
			config = handle->getConfigManager();
			success = config->firmWareUpdate(lpszFileName, boost::bind(callback, _1, _2, _3, clientHandle));
		}
	}

	if (!success)
	{
		log(LogTarget::ERR, USB_FET_BSL_ACTIVE_ERR, "");
	}
	return success;
}

void DLL430_OldApiV3::HIL_ResetJtagTap()
{
	if (handle)
		handle->sendHilCommand(HIL_CMD_RESET_JTAG_TAP);
}

bool DLL430_OldApiV3::HIL_Open()
{
	if (singleDevice)
	{
		if (DebugManager* db_man = singleDevice->getDebugManager())
		{
			// Stop polling, since we don't want it to interfere with low-level access
			db_man->pausePolling();
		}
	}

	const bool success = handle && handle->sendHilCommand(HIL_CMD_OPEN);
	if (!success)
		log(LogTarget::ERR, INTERNAL_ERR, "");

	return success;
}

bool DLL430_OldApiV3::HIL_Connect()
{
	const bool success = handle && handle->sendHilCommand(HIL_CMD_CONNECT);
	if (!success)
		log(LogTarget::ERR, INTERNAL_ERR, "");

	return success;
}

bool DLL430_OldApiV3::HIL_Close(long vccOff)
{
	const bool success = handle && handle->sendHilCommand(HIL_CMD_CLOSE, vccOff);
	if (!success)
		log(LogTarget::ERR, INTERNAL_ERR, "");
	
	return success;
}

bool DLL430_OldApiV3::HIL_TCK(long state)
{
	const bool success = handle && handle->setJtagPin(JTAG_PIN_TCK, state != 0);
	if (!success)
		log(LogTarget::ERR, INTERNAL_ERR, "");

	return success;
}

bool DLL430_OldApiV3::HIL_TMS(long state)
{
	const bool success = handle && handle->setJtagPin(JTAG_PIN_TMS, state != 0);
	if (!success)
		log(LogTarget::ERR, INTERNAL_ERR, "");

	return success;
}

bool DLL430_OldApiV3::HIL_TDI(long state)
{
	const bool success = handle && handle->setJtagPin(JTAG_PIN_TDI, state != 0);
	if (!success)
		log(LogTarget::ERR, INTERNAL_ERR, "");

	return success;
}

bool DLL430_OldApiV3::HIL_RST(long state)
{
	const bool success = handle && handle->setJtagPin(JTAG_PIN_RST, state != 0);
	if (!success)
		log(LogTarget::ERR, INTERNAL_ERR, "");

	return success;
}

bool DLL430_OldApiV3::HIL_TST(long state)
{
	const bool success = handle && handle->setJtagPin(JTAG_PIN_TST, state != 0);
	if (!success)
		log(LogTarget::ERR, INTERNAL_ERR, "");

	return success;
}

uint64_t DLL430_OldApiV3::HIL_JTAG_IR(long instruction)
{
	uint64_t retVal = (uint64_t)-1;
	if (handle)
		retVal = handle->sendJtagShift(HIL_CMD_JTAG_IR, instruction);

	return retVal;
}

uint64_t DLL430_OldApiV3::HIL_JTAG_DR(int64_t data, long bitSize)
{
	uint64_t retVal = (uint64_t)-1;
	if (handle)
		retVal = handle->sendJtagShift(HIL_CMD_JTAG_DR, data, bitSize);

	return retVal;
}

void DLL430_OldApiV3::execNotifyCallback(SYSTEM_EVENT_MSP event)
{
	// just executes the function, given by the IDE
	if(notifyCallback!=0)
	{
		this->notifyCallback(event);		// event could be DEVICE_CONNECTION_LOST etc
	}
}

void DLL430_OldApiV3::iNotifyCallback(uint32_t msgId, uint32_t wParam, uint32_t lParam, uint32_t clientHandle)
{
	switch(wParam)
	{
	case 0:
		this->execNotifyCallback(FET_CONNECTION_LOST);
		break;

	default:
		break;
	}
}

bool DLL430_OldApiV3::writeToExternalMemory()
{
	// PC  = @0xF8A2      Set to start of Generate Image routine
	// R11 = @0x1C7E      Set to addr in RAM Reset vector
	// R12 = RAM_START
	// R13 = 0x0000       Upper addr for start of SPI memory
	// R14 = 0x0002       write image starting at addr 2
	// R15 = RAM_LENGTH   (RAM_END-RAM_START)
	// SP  = 0x23FE       make space for return addr on stack
	// @23FE = @1C7E      make function return to program start 
	MemoryManager* mm = singleDevice ? singleDevice->getMemoryManager() : 0;
		
	MemoryArea* registers = mm ? mm->getMemoryArea("CPU") : 0;
	
	if (!registers)
	{
		log(LogTarget::ERR, INTERNAL_ERR, "");
		return false;
	}

	const uint32_t WRITE_ERROR = 0xFD4C;

	uint32_t saveRegister[16] = {0x0};
	const uint32_t ramStart = 0x1C60;
	const uint32_t ramEnd = 0x2400;
	const uint32_t stackPtr = ramEnd - 2;
	const uint32_t stackReserve = 50;

	registers->read(0, saveRegister, 16);

	uint32_t tmp[2] = {0};

	if ( !mm->read(0xF8A2, tmp, 2) || !mm->sync() )
	{
		log(LogTarget::ERR, READ_MEMORY_ERR, "");
		return false;
	}

	const uint32_t functionAddress = tmp[0] | (tmp[1] << 8);

	if ( !mm->read(0x1C7E, tmp, 2) || !mm->sync() )
	{
		log(LogTarget::ERR, READ_MEMORY_ERR, "");
		return false;
	}

	const uint32_t programStart = tmp[0] | (tmp[1] << 8);	

	uint32_t loaderAddress[2] = {0xFE, 0xBC};
	if ( !mm->write(stackPtr, loaderAddress, 2) || !mm->sync() )
	{
		log(LogTarget::ERR, WRITE_MEMORY_ERR, "");
		return false;
	}

	registers->write(0, functionAddress );	//Set PC to start of Generate Image routine
	registers->write(1, stackPtr);
	registers->write(11, programStart );
	registers->write(12, ramStart);
	registers->write(13, 0);
	registers->write(14, 2);
	registers->write(15, ramEnd - ramStart - stackReserve);

	this->handle->getConfigManager()->start();

	Run(FREE_RUN, true);					// start execution of loader code

	boost::this_thread::sleep(boost::get_system_time() + boost::posix_time::seconds(15));
	
	long state;
	long pCPUCycles;
	State(&state, true, &pCPUCycles);

	uint32_t value;
	registers->read(0, &value, 1);

	registers->write(0, saveRegister, 16);
	registers->getCacheCtrl()->flush(0, 0);	 // write save register values into the msp

	return (value != WRITE_ERROR);
}
