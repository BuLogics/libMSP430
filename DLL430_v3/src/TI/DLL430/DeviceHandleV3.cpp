/*
 * DeviceHandleV3.cpp
 *
 * Communication with target device.
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

#include <DLL430/VersionInfo.h>

#include "DeviceHandleV3.h"
#include "DeviceDbManagerExt.h"
#include "HalExecCommand.h"
#include "FetControl.h"
#include "WatchdogControl.h"
#include "FetHandleV3.h"
#include "MemoryManagerV3.h"
#include "DebugManagerV3.h"
#include "ClockCalibration.h"
#include "EM/EmulationManager/EmulationManager430.h"
#include "EM/EemRegisters/EemRegisterAccess.h"
#include "EM/Exceptions/Exceptions.h"
#include "EemMemoryAccess.h"

#include "../../Bios/include/ConfigureParameters.h"

#include <iostream>

using namespace TI::DLL430;

DeviceHandleV3::DeviceHandleV3 (FetHandleV3* parent, DeviceChainInfo* deviceChainInfo)
 : parent(parent)
 , deviceChainInfo(deviceChainInfo)
 , memoryManager(NULL)
 , debugManager(NULL)
 , fileManager(NULL)
 , clockCalibration(NULL)
 , minFlashVcc(2700)
 , hasTestVpp(false)
 , quickMemRead(true)
 , deviceHasFram(false)
 , clockSystem(BC_1xx)
 , jtagId(0)
 , deviceIdPtr(0)
 , CoreIpId(0)
 , IdDataAddr(0)
 , eemVersion(0)
 , mode(JTAG_MODE_4WIRE)
 , myMutex(0)
{ 
	this->myMutex = parent->getControl()->getMutex();

	assert(DeviceInfo::nrUsedClockModules == etwCodes.size());
	for (int i=0;i<DeviceInfo::nrUsedClockModules;i++)
	{
		etwCodes[i]=0;
	}
	//// set watchdog and realtime clock as default
	etwCodes[0] = 1;
	etwCodes[10] = 40;
}

DeviceHandleV3::~DeviceHandleV3 ()
{ 
	if (NULL != memoryManager)
	{
		setEemRegisterAccess(0);
		delete memoryManager;
		memoryManager = NULL;
	}

	if (NULL != debugManager)
	{
		delete debugManager;
		debugManager = NULL;
	}

	if (fileManager!=NULL)
	{
		delete fileManager;
		fileManager = NULL;
	}

	if(clockCalibration != NULL)
	{
		delete clockCalibration;
		clockCalibration = NULL;
	}
}


EmulationManagerPtr DeviceHandleV3::getEmulationManager()
{
	if (!emulationManager)
		throw EM_NoEmulationManagerException();

	return this->emulationManager;
}

MemoryManagerV3* DeviceHandleV3::getMemoryManager ()
{
	return this->memoryManager;
}

DebugManagerV3* DeviceHandleV3::getDebugManager ()
{
	return this->debugManager;
}

FileFunc* DeviceHandleV3::getFileRef()
{
	if (fileManager!=NULL)
		delete fileManager;

	fileManager=new FileFuncImpl();

	return (FileFunc*)fileManager;
}

bool DeviceHandleV3::writeSegments()
{
	if (fileManager==NULL)
		return false;

	return fileManager->writeSegs(this);
}

bool DeviceHandleV3::runBootCode(uint32_t command)
{
	HalExecCommand cmd;	
	HalExecElement* el = new HalExecElement(ID_StartJtagActivationCode);

	el->appendInputData8(0);		// Protocol: 0 - JTAG
	el->appendInputData32(command);	// Code
	el->appendInputData32(100);		// Delay ms
		
	cmd.elements.push_back(el);

	if (!this->send(cmd))
	{
		return false;
	}
	return (this->identifyDevice(0) >= 0);
}

bool DeviceHandleV3::verifySegments()
{
	if (fileManager==NULL)
		return false;

	return fileManager->verifySegs(this,false);
}

long DeviceHandleV3::magicPatternSend()
{
	HalExecCommand cmd;	
    HalExecElement* el = new HalExecElement(ID_MagicPattern);
	cmd.elements.push_back(el);

	if (!this->send(cmd))
	{
		return -1;
	}	

	//GET Chain Length
	uint8_t chainLen= cmd.elements.at(0).getOutputAt8(0);
	//Get JTAG ID 
	uint8_t iJtagID= cmd.elements.at(0).getOutputAt8(1);

	if ((chainLen > 0) && 
	    (iJtagID == 0x91) || // FR57xx
	    (iJtagID == 0x99))
	{
		// Return the protocol
		return cmd.elements.at(0).getOutputAt8(2);
	}
	return -1;
}

long DeviceHandleV3::identifyDevice (uint32_t activationKey)
{	
	sendDeviceConfiguration(CONFIG_PARAM_CLK_CONTROL_TYPE, GccNone);
	sendDeviceConfiguration(CONFIG_PARAM_DEFAULT_CLK_CONTROL, 0x040f);
	sendDeviceConfiguration(CONFIG_PARAM_ENHANCED_PSA, DeviceInfo::PSATYPE_REGULAR);
	sendDeviceConfiguration(CONFIG_PARAM_PSA_TCKL_HIGH, false);
	sendDeviceConfiguration(CONFIG_PARAM_POWER_TESTREG_MASK, 0x0000);
	sendDeviceConfiguration(CONFIG_PARAM_POWER_TESTREG3V_MASK, 0x0000);
	sendDeviceConfiguration(CONFIG_NO_BSL, 0x0000);
	sendDeviceConfiguration(CONFIG_ALT_ROM_ADDR_FOR_CPU_READ, 0);

	if (this->isJtagFuseBlown())
	{		
		return -5555;
	}
	
	long devId = -1;
	uint32_t pc = 0;
	uint32_t sr = 0;

	switch (this->getJtagId())
	{
		case 0x89:  // msp430 cpu
		case 0x91:  // 5xx
		case 0x95:	// L092 
		case 0x99:
			devId = this->getDeviceIdentity(activationKey, &pc, &sr);
			break;
	}
	if (devId < 0)
	{
		return devId;
	}
	this->setDeviceId(devId);
	MemoryManager* mm = this->getMemoryManager();
	if (mm) 
	{
		MemoryArea* cpu = mm->getMemoryArea("CPU");
		if (cpu) 
		{
			cpu->write(0, pc);
			cpu->write(2, sr);
			cpu->getCacheCtrl()->fill(0, 16);
		}
	}
	return devId;
}

const std::string & DeviceHandleV3::getDescription()
{
	return description;
}

// reset moved from ConfigManager to Device
bool DeviceHandleV3::reset ()
{
	boost::shared_ptr<WatchdogControl> wdt = this->getWatchdogControl();

	HalExecCommand cmd;

	HalExecElement* el = new HalExecElement(this->checkHalId(ID_SyncJtag_AssertPor_SaveContext));

	wdt->addParamsTo(el, true);

	el->appendInputData8(this->getJtagId());

	for(int i=0;i<16;i++)
	{
		el->appendInputData8(etwCodes[i]);
	}

	el->setOutputSize(8);
	cmd.elements.push_back(el);
	if (!this->parent->getControl()->send(cmd))
		return false;

	uint16_t wdtCtrl = el->getOutputAt16(0);
	if (!WatchdogControl::checkRead(wdtCtrl))
		return false;

	wdt->set(wdtCtrl);

	MemoryManagerV3* mm = this->getMemoryManager();
	if (mm==NULL)
		return false;

	if ( MemoryArea* cpu = mm->getMemoryArea("CPU") )
	{
		cpu->write(0, el->getOutputAt32(2));
		cpu->write(2, el->getOutputAt16(6));
		cpu->getCacheCtrl()->fill(0, 16);
	}
	return true;
}

void DeviceHandleV3::setDeviceId (long id)
{
	TemplateDeviceDbManagerExt dbm;
	boost::shared_ptr<DeviceInfo> info = dbm.queryDb(id);
	this->configure(info.get());
}


void DeviceHandleV3::configure (const DeviceInfo* info)
{
	boost::mutex::scoped_lock lock(*(this->myMutex));

	setEemRegisterAccess(0);

	if (this->debugManager)
		delete this->debugManager;

	if (this->memoryManager)
		delete this->memoryManager;

	this->parent->getControl()->setObjectDbEntry(info->getObjectId());
	this->map = info->getMap();

	this->funcletTable = info->getFuncletMap();

	this->minFlashVcc = info->minFlashVcc();
	this->hasTestVpp = info->hasTestVpp();
	this->quickMemRead = info->quickMemRead();
	this->deviceHasFram = info->hasFram();
	this->clockSystem = info->clockSystem(),

	this->description=info->getDescription();

	this->memoryManager = new MemoryManagerV3(this, info);
	this->debugManager  = new DebugManagerV3(this, info);
	this->emulationManager = EmulationManager430::create(info->getEmulationLevel());
	
	setEemRegisterAccess( dynamic_cast<EemMemoryAccess*>(memoryManager->getMemoryArea("EEM")) );
	
	const ConfigManager* configManager = parent->getConfigManager();
	this->clockCalibration = ClockCalibration::create(this, memoryManager, configManager, *info);

	// If frequency calibration is disabled, remap funclets to FLL versions (those won't change any settings)
	if (configManager && !configManager->freqCalibrationEnabled())
	{
		const FuncletCode& erase = funcletTable[FuncletCode::ERASE];
		if ( erase == FuncletCode(eraseFuncletCodeDCO, sizeof(eraseFuncletCodeDCO), 4) )
		{
			funcletTable[FuncletCode::ERASE] = FuncletCode( eraseFuncletCodeFLL, sizeof(eraseFuncletCodeFLL), 4 );
			funcletTable[FuncletCode::WRITE] = FuncletCode( writeFuncletCodeFLL, sizeof(writeFuncletCodeFLL), 128 );
		}
		else if ( erase == FuncletCode( eraseFuncletCodeXDCO, sizeof(eraseFuncletCodeXDCO), 4 ) )
		{
			funcletTable[FuncletCode::ERASE] = FuncletCode( eraseFuncletCodeXFLL, sizeof(eraseFuncletCodeXFLL), 4 );
			funcletTable[FuncletCode::WRITE] = FuncletCode( writeFuncletCodeXFLL, sizeof(writeFuncletCodeXFLL), 256 );
		}
	}

	const DeviceInfo::ClockMapping& clockMapping = info->getClockMapping();
	assert(DeviceInfo::nrUsedClockModules <= clockMapping.size());
	
	for (uint32_t i=0;i<DeviceInfo::nrUsedClockModules;++i)
	{
		etwCodes[DeviceInfo::nrUsedClockModules-(i+1)] = clockMapping[i].second;
	}

	lock.unlock();

	sendDeviceConfiguration(CONFIG_PARAM_CLK_CONTROL_TYPE, info->getClockControl());
	sendDeviceConfiguration(CONFIG_PARAM_SFLLDEH,info->getSFll());
	sendDeviceConfiguration(CONFIG_PARAM_DEFAULT_CLK_CONTROL, info->getClockModDefault());
	sendDeviceConfiguration(CONFIG_PARAM_ENHANCED_PSA, info->getPsaType());
	sendDeviceConfiguration(CONFIG_PARAM_PSA_TCKL_HIGH, info->psach());
	
	// Set power settings
	sendDeviceConfiguration(CONFIG_PARAM_POWER_TESTREG_MASK, info->powerTestRegMask());
	sendDeviceConfiguration(CONFIG_PARAM_TESTREG_ENABLE_LPMX5, info->testRegEnableLpmx5());
	sendDeviceConfiguration(CONFIG_PARAM_TESTREG_DISABLE_LPMX5, info->testRegDisableLpmx5());

	sendDeviceConfiguration(CONFIG_PARAM_POWER_TESTREG3V_MASK, info->powerTestReg3VMask());
	sendDeviceConfiguration(CONFIG_PARAM_TESTREG3V_ENABLE_LPMX5, info->testReg3VEnableLpmx5());
	sendDeviceConfiguration(CONFIG_PARAM_TESTREG3V_DISABLE_LPMX5, info->testReg3VDisableLpmx5());

	sendDeviceConfiguration(CONFIG_NO_BSL, info->noBsl());
	sendDeviceConfiguration(CONFIG_ALT_ROM_ADDR_FOR_CPU_READ, info->b1377() ? 1 : 0);
}

bool DeviceHandleV3::sendDeviceConfiguration(uint32_t parameter, uint32_t value)
{
	HalExecElement* el = new HalExecElement(ID_Configure);
	el->appendInputData32(parameter);
	el->appendInputData32(value);
	el->setOutputSize(0);

	HalExecCommand configCmd;		
	configCmd.elements.push_back(el);

	return this->send(configCmd);
}

bool DeviceHandleV3::sendSyncCommand()
{
	boost::shared_ptr<WatchdogControl> wdt = this->getWatchdogControl();

	HalExecCommand devId;
		
	HalExecElement* el = new HalExecElement(this->checkHalId(ID_SyncJtag_AssertPor_SaveContext));
	wdt->addParamsTo(el, true);
	el->appendInputData8(this->getJtagId());
	for(int i=0;i<16;i++)
		el->appendInputData8(etwCodes[i]);

	el->setOutputSize(8);
	devId.elements.push_back(el);

	return this->send(devId);
}

long DeviceHandleV3::getDeviceIdentity(uint32_t activationKey, uint32_t* pc, uint32_t* sr)
{
	MSP430_device_idCode idCode;
	boost::shared_ptr<WatchdogControl> wdt = this->getWatchdogControl();

	uint32_t sync_jtag_id = 0;
	uint32_t read_mem_id = 0;
	bool is_xv2_device = false;
	int read_size = 0;

	uint32_t JtagId = this->getJtagId();

	// CPUxv2 devices 
	if (0x99 == JtagId || 0x95 == JtagId || 0x91 == JtagId)
	{
		sync_jtag_id = ID_SyncJtag_AssertPor_SaveContextXv2;
		read_mem_id = ID_ReadMemQuickXv2;
		is_xv2_device = true;
		read_size = 4 ;
	} 
	else // JTAG ID is 89 
	{
		sync_jtag_id = ID_SyncJtag_AssertPor_SaveContext;
		read_mem_id =  ID_ReadMemWords;
		is_xv2_device = false;
		read_size = 8;
	}
	HalExecCommand devId;	
	// Set device Chain info
    HalExecElement* el = new HalExecElement(ID_SetDeviceChainInfo);
	el->appendInputData16(static_cast<uint16_t>(this->deviceChainInfo->getBusId()));
	devId.elements.push_back(el);

	el = new HalExecElement(sync_jtag_id);		// ID_SyncJtag_AssertPor_SaveContext/...Xv2 this will return PC & SR

	wdt->addParamsTo(el, true);

	el->appendInputData8(this->getJtagId());

	for (int i=0;i<16;i++)
	{
		el->appendInputData8(etwCodes[i]);
	}
	el->setOutputSize(8);
	devId.elements.push_back(el);

	if(!is_xv2_device)
	{
		el = new HalExecElement(read_mem_id);		// ID_ReadMemWords/...Xv2	

		el->appendInputData32(this->getIdDataAddr());
		el->appendInputData32(read_size);

		el->setOutputSize(read_size*2);
		devId.elements.push_back(el);

		el = new HalExecElement(ID_GetFuses);
		el->setOutputSize(1);

		devId.elements.push_back(el);

		if (!this->send(devId))
		{
			return -1;
		}
		uint16_t wdtCtrl = devId.elements.at(1).getOutputAt16(0);
		if (!WatchdogControl::checkRead(wdtCtrl))
		{
			return -1;
		}
		wdt->set(wdtCtrl);
		*pc = devId.elements.at(1).getOutputAt32(2);
		*sr = devId.elements.at(1).getOutputAt16(6);

		idCode.verId = devId.elements.at(2).getOutputAt16(0);
		idCode.verSubId = 0x0000;
		idCode.revisison = devId.elements.at(2).getOutputAt8(2);
		idCode.fab = devId.elements.at(2).getOutputAt8(3);
		idCode.self = devId.elements.at(2).getOutputAt16(4);
		idCode.config = (char)(devId.elements.at(2).getOutputAt8(13) & 0x7F);
		idCode.fuses = devId.elements.at(3).getOutputAt8(0);
		idCode.activationKey = 0;
	}
	else // must be xv2 CPU device 99,95,91
	{
		HalExecCommand devDesc;

		if (!this->send(devId))
		{
			return -1;
		}
		uint16_t wdtCtrl = devId.elements.at(1).getOutputAt16(0);

		// check read watchdog password 
 		if (!WatchdogControl::checkRead(wdtCtrl))
		{
			return -1;
		}
		wdt->set(wdtCtrl);

		*pc = devId.elements.at(1).getOutputAt32(2);
		*sr = devId.elements.at(1).getOutputAt16(6);

		el = new HalExecElement(read_mem_id);

		el->appendInputData32(this->getDeviceIdPtr());
		el->appendInputData32(4);
			
		devDesc.elements.push_back(el);
		if (!this->send(devDesc))
		{
			return -1;
		}
		uint8_t info_len = el->getOutputAt8(0);
		uint8_t crc_len = el->getOutputAt8(1);

		// get device ID was read out with comand before
		idCode.verId = (el->getOutputAt8(5) << 8) + el->getOutputAt8(4);
#ifndef NDEBUG
		printf("version ID (0x91): %4x\n",idCode.verId);
#endif
		idCode.verSubId = 0;
		idCode.config = el->getOutputAt8(6);    // SW Revision
		idCode.revisison = el->getOutputAt8(7); // HW Revision
		
		idCode.verSubId = 0x0000; // init with zero = no sub id 
		
		//Sub ID should be everything but not -1  
		uint16_t SubId= getSubID(info_len, deviceIdPtr, *pc);
		if((short)SubId != -1)
		{
			idCode.verSubId = SubId;
		}
		else
		{
			// in case we have an error during sub id detectin 
			return -1;
		}
		HalExecCommand eemv;
		HalExecElement* el = new HalExecElement(ID_EemDataExchangeXv2);
		el->appendInputData8(1);
		el->appendInputData8(0x87);
		eemv.elements.push_back(el);
		if (!this->send(eemv))
		{
			return -1;
		}
		eemVersion=el->getOutputAt32(0);

		idCode.fab = 0x55;
		idCode.self = 0x5555;
		
		idCode.fuses = 0x55;
		idCode.activationKey = activationKey;
	}
   	TemplateDeviceDbManagerExt db;
	return (long)db.queryDb(idCode);
};

uint16_t DeviceHandleV3::getSubID(uint32_t info_len, uint32_t deviceIdPtr , uint32_t pc)
{
	const int UNDEFINED_00 = 0x00;
	if ((info_len > 1)&&(info_len < 11))
	{
		int tlv_size=4*((int)pow(2.0,(double)info_len))-2;
		HalExecCommand cmd;
		HalExecElement* el = new HalExecElement(ID_ReadMemQuickXv2);
		el->appendInputData32(deviceIdPtr);
		el->appendInputData32(tlv_size/2);
		el->appendInputData32(pc);
		el->setOutputSize(tlv_size);

		cmd.elements.push_back(el);
		if (!this->send(cmd))
		{
			return -1;
		}

		const vector<uint8_t>& tlv = el->getOutput();
		int pos = 8;
		//Must have at least 2 byte data left
		while (pos + 3 < tlv_size)
		{
			const uint8_t tag = tlv[pos++];
			const uint8_t len = tlv[pos++];
			const uint8_t *value = &tlv[pos];
			pos += len;
			          
			const int SUBVERSION_TAG = 0x14;
			const int UNDEFINED_FF = 0xFF;
					  
			if (tag == SUBVERSION_TAG)
			{
				return ((value[1]<<8) + value[0]);
			}
			if ((tag == UNDEFINED_FF) || (tag == UNDEFINED_00) || (tag == SUBVERSION_TAG))
			{
				return UNDEFINED_00;
			}
		}
	}
	return UNDEFINED_00;
}


uint8_t DeviceHandleV3::getJtagId()
{
	return this->jtagId;
}
uint16_t DeviceHandleV3::getCoreIpId()
{
	return this->CoreIpId;
}
uint32_t DeviceHandleV3::getIdDataAddr()
{
	return this->IdDataAddr;
}

uint32_t DeviceHandleV3::getDeviceIdPtr()
{
	return this->deviceIdPtr;
}

uint32_t DeviceHandleV3::getEemVersion()
{
	return this->eemVersion;
}

uint8_t DeviceHandleV3::getDeviceJtagId()
{
	if (this->jtagId != 0)
	{
		return this->jtagId;
	}
	HalExecCommand cmd;
    HalExecElement* el = new HalExecElement(ID_SetDeviceChainInfo);
	el->appendInputData16(static_cast<uint16_t>(this->deviceChainInfo->getBusId()));
	cmd.elements.push_back(el);

	el = new HalExecElement(ID_GetJtagId);

	el->setOutputSize(12);
	cmd.elements.push_back(el);
	if (!this->send(cmd))
	{
		return 0;
	}

	uint16_t Id	= cmd.elements.at(1).getOutputAt16(0);
	
#ifndef NDEBUG
	printf("JtagID: %2x\n",Id);
#endif
	switch (Id) 
	{
		case 0x89:
			this->setWatchdogControl(
				boost::shared_ptr<WatchdogControl>(new WatchdogControl(0x0120))
			);
			break;
		case 0x91:
		case 0x95:
		case 0x99:
			this->setWatchdogControl(
				boost::shared_ptr<WatchdogControl>(new WatchdogControl(0x015C))
			);
			break;
	}
	if((Id==0x89)||(Id==0x91)||(Id==0x95)|| (Id==0x99))
	{
		this->jtagId = (uint8_t)Id;
		this->CoreIpId		= cmd.elements.at(1).getOutputAt16(2);
		this->deviceIdPtr	= cmd.elements.at(1).getOutputAt32(4);
		this->IdDataAddr	= cmd.elements.at(1).getOutputAt32(8);
	}
	else
	{
		this->jtagId		= 0x00;
		this->CoreIpId		= 0x00;
		this->deviceIdPtr	= 0x00;
		this->IdDataAddr	= 0x00;
	}
	return static_cast<uint8_t>(Id);
};
bool DeviceHandleV3::isJtagFuseBlown()
{	
	HalExecCommand cmd;
	HalExecElement* el = new HalExecElement(ID_IsJtagFuseBlown);
	el->setOutputSize(2);
	cmd.elements.push_back(el);

	if(!send(cmd))
	{
		return false;
	}
	if(el->getOutputAt16(0) == 0x5555)			        // If the fuse is blown, the device will be in JTAG bypass mode, and data
	{											// input to the device will be echoed with a one-bit delay.
		return true; // Fuse is blown.
	}
	else
	{
		return false; // Fuse blow error.
	}
}

bool DeviceHandleV3::secure ()
{
	HalExecCommand cmd;
	HalExecElement* el = new HalExecElement(ID_SetDeviceChainInfo);
	el->appendInputData16(static_cast<uint16_t>(this->deviceChainInfo->getBusId()));
    cmd.elements.push_back(el);
	// Device with silicon fuse
	if(this->checkHalId(ID_BlowFuse) == ID_BlowFuse)
	{
		el = new HalExecElement(this->checkHalId(ID_BlowFuse));
		el->appendInputData8(this->hasTestVpp ? 1: 0);
		cmd.elements.push_back(el);
	}
	if(this->checkHalId(ID_BlowFuse) == ID_BlowFuseFram)
	{
		el = new HalExecElement(this->checkHalId(ID_BlowFuseFram));
		cmd.elements.push_back(el);
	}
	if(this->checkHalId(ID_BlowFuse) == ID_BlowFuseXv2)
	{
		if(!(memoryManager->uploadFunclet(FuncletCode::WRITE)))		
		{
			return false; 
		}
		MemoryManager* mm = this->getMemoryManager();
		MemoryArea* ram = mm->getMemoryArea("system", 0);
		const FuncletCode& funclet = this->getFunclet(FuncletCode::WRITE);

		const uint32_t type = 0;
		const uint32_t lenght =2; 	// 2 words
		const uint16_t flags = 0xA508;
		const uint16_t programStartAddress = ram->getStart() + funclet.programStartOffset();

		el = new HalExecElement(this->checkHalId(ID_BlowFuse));
		cmd.setTimeout(10000);	// overwrite 3 sec default with 10 sec

		el->appendInputData16(static_cast<uint16_t>(ram->getStart() & 0xFFFF));
		el->appendInputData16(static_cast<uint16_t>(ram->getSize() & 0xFFFF));
		el->appendInputData16(programStartAddress);
		el->appendInputData32(static_cast<uint32_t>(0x17FC));
		el->appendInputData32(lenght);
		el->appendInputData16(type);
		el->appendInputData16(flags);
		el->appendInputData16(0);
		el->appendInputData16(0);
		el->setOutputSize(2);
		cmd.elements.push_back(el);
	}
	if(!send(cmd))
	{
		return false;
	}

	ConfigManager* cm = parent->getConfigManager();
	cm->setDeviceCode(0);
	cm->setPassword("");
	cm->start();
	return this->isJtagFuseBlown();
}



FetControl* DeviceHandleV3::getControl ()
{
	return parent->getControl();
}

DeviceChainInfo* DeviceHandleV3::getDevChainInfo ()
{
	return this->deviceChainInfo;
}

bool DeviceHandleV3::send (HalExecCommand &command)
{
	this->myMutex = parent->getControl()->getMutex();
	boost::mutex::scoped_lock lock(*(this->myMutex));

	return this->parent->getControl()->send(command,&this->map);
}

void DeviceHandleV3::setWatchdogControl (boost::shared_ptr<WatchdogControl> ctrl)
{
	this->wdt = ctrl;
}

boost::shared_ptr<WatchdogControl> DeviceHandleV3::getWatchdogControl ()
{
	return this->wdt;
}

uint32_t DeviceHandleV3::checkHalId(uint32_t base_id) const
{
	function_map_type::const_iterator it = map.find(base_id);
	return (it != map.end()) ? it->second : base_id;
}

uint32_t DeviceHandleV3::getBaseHalId(uint32_t mapped_id) const
{
	for (function_map_type::const_iterator it = map.begin(); it != map.end(); ++it)
	{
		if ( it->second == mapped_id )
			return it->first;
	}
	return mapped_id;
}

const FuncletCode& DeviceHandleV3::getFunclet(FuncletCode::Type funclet)
{
	return funcletTable[funclet];
}

bool DeviceHandleV3::supportsQuickMemRead() const
{
	return quickMemRead;
}

uint16_t DeviceHandleV3::getMinFlashVcc() const
{
	return minFlashVcc;
}

bool DeviceHandleV3::hasFram() const
{
	return deviceHasFram;
}

ClockSystem DeviceHandleV3::getClockSystem() const
{
	return clockSystem;
}
