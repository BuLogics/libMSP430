/*
 * DebugManagerV3.cpp
 *
 * Functionality for debugging target device.
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
#define _CRT_SECURE_NO_WARNINGS //disabling warnings to use secure c-function versions (e.g. strcpy_s) as this is not compatible with none MS development
#endif
#endif

#include "EemMemoryAccess.h"

#include "MemoryCacheGeneric.h"

#include "DebugManagerV3.h"
#include "DeviceHandleV3.h"
#include "FetHandleV3.h"
#include "HalExecCommand.h"
#include "HalResponse.h"
#include "HalResponseHandler.h"
#include "FetControl.h"
#include "TemplateDeviceDb/MSP430Defaults.h"
#include "DeviceInfo.h"
#include "EM/EmulationManager/IEmulationManager.h"
#include "EM/Trace/ITrace.h"
#include "MessageData.h"

#include <algorithm>

using namespace TI::DLL430;
using namespace TI::DLL430::TemplateDeviceDb;

/// State modes.
enum INTERNAL_STATE_MODES {
	D_STOPPED = 0, /**< The device is stopped */
	D_RUNNING = 1, /**< The device is running or is being single stepped */
	D_SINGLE_STEP_COMPLETE = 2, /**< The device is stopped after the single step operation is complete */
	D_BREAKPOINT_HIT = 3, /**< The device is stopped as a result of hitting an enabled breakpoint */
	D_LPMX5_MODE = 4, /**< The device is in LPMx.5 low power mode  */
	D_LPMX5_WAKEUP = 5, /**<	The device woke up from LPMx.5 low power mode */
	D_LPMX5_WAKEUP_DETECTED = 6,
};

DebugManagerV3::DebugManagerV3 (DeviceHandleV3* parent, const DeviceInfo* devInfo)
 : parent(parent)
 , clockControl(devInfo->getClockControl())
 , genclkcntrl(DefaultClkCntrl)
 , mclkcntrl0(devInfo->getClockModDefault())
 , defaultMclkcntrl0(devInfo->getClockModDefault())
 , emulationLevel(devInfo->getEmulationLevel())
 , moduleStrings(0)
 , nModuleStrings(0)
 , clockStrings(0)
 , nClockStrings(0)
 , opcode(0)
 , lpmDebuggingEnabled(false)
 , deviceInLpm5(false)
 , internalDebugState(D_STOPPED)
 , cbx(0)
 , cycleCounter_(parent, devInfo)
 , resetCycleCounterBeforeNextStep(true)
 , storagePollingActive(false)
{
	this->waitForEem.setCallBack( boost::bind(&DebugManagerV3::localCallback, this, _1, _2), 0 );
	this->waitForEem.setAsyncMode(false);

	this->waitForJState.setCallBack( boost::bind(&DebugManagerV3::localCallback, this, _1, _2), 0 );
	this->waitForJState.setAsyncMode(true);

	this->waitForStorage.setCallBack( boost::bind(&DebugManagerV3::localCallback, this, _1, _2), 0 );
	this->waitForStorage.setAsyncMode(true);

	createModuleStrings(devInfo->getClockMapping());
	createClockStrings(devInfo->getClockNames());

	eventNotifier.setEventHandler( boost::bind(&DebugManagerV3::runEvent, this, _1) );
	eventNotifier.startProcessingEvents();
}

DebugManagerV3::~DebugManagerV3 ()
{
	if ( FetHandle* fetHandle = parent->getFetHandle() )
	{
		parent->getFetHandle()->kill( waitForJState.getResponseId() );
		parent->getFetHandle()->kill( waitForEem.getResponseId() );
		parent->getFetHandle()->kill( waitForStorage.getResponseId() );
	}

	if(NULL != moduleStrings)
	{
		for(uint32_t i = 0; i < nModuleStrings; ++i)
		{
			delete[] moduleStrings[i];
			moduleStrings[i] = NULL;		
		}
		delete[] moduleStrings;
		moduleStrings = NULL;
	}

	if(NULL != clockStrings)
	{
		for(uint32_t i = 0; i < nClockStrings; ++i)
		{
			delete[] clockStrings[i];
			clockStrings[i] = NULL;		
		}
		delete[] clockStrings;
		clockStrings = NULL;
	}
}

void DebugManagerV3::createModuleStrings(const DeviceInfo::ClockMapping& clockMapping)
{
	nModuleStrings = static_cast<uint32_t>(clockMapping.size());
	moduleStrings = new char*[nModuleStrings];
	for(uint32_t i = 0; i < nModuleStrings; ++i)
	{
		size_t size = clockMapping[i].first.size() + 1;
		moduleStrings[i] = new char[size];
		memset(moduleStrings[i], 0, size);
		strncpy(moduleStrings[i], clockMapping[i].first.c_str(), size - 1);
	
	}
}

void DebugManagerV3::createClockStrings(const DeviceInfo::ClockNames& clockNames)
{
	nClockStrings = static_cast<uint32_t>(clockNames.size());
	clockStrings = new char*[nClockStrings];
	for(uint32_t i = 0; i < nClockStrings; ++i)
	{
		size_t size = clockNames[i].size() + 1;
		clockStrings[i] = new char[size];
		memset(clockStrings[i], 0, size);
		strncpy(clockStrings[i], clockNames[i].c_str(), size - 1);	
	}
}

bool DebugManagerV3::activatePolling(uint16_t mask)
{
	if(parent->checkHalId(ID_WaitForEem) == ID_WaitForEem)
	{
		HalExecElement* el = new HalExecElement(parent->checkHalId(ID_WaitForEem));
		el->appendInputData16(mask);
		el->setOutputSize(2);
		this->waitForEem.elements.clear();
		this->waitForEem.elements.push_back(el);	
		return this->parent->send(this->waitForEem);
	}
	else 
	{	// case ID_PollJStateReg
		if ( waitForJState.getResponseId() != 0 )
		{
			return true;
		}
		uint64_t mask2 = 0x8000000100000000LL; // Only interested in bit 63

		HalExecElement* el = new HalExecElement(parent->checkHalId(ID_PollJStateReg));
		el->appendInputData32((uint32_t)(mask2 & 0xFFFFFFFF));
		el->appendInputData32((uint32_t)(mask2 >> 32));
		el->appendInputData16((uint16_t)false);
		el->setOutputSize(2);

		this->waitForJState.elements.clear();
		this->waitForJState.elements.push_back(el);
		
		if (!this->parent->send(this->waitForJState))
		{
			return false;
		}
		//this->resumePolling();
		return true;
	}
}


bool DebugManagerV3::startStoragePolling()
{
	if (!storagePollingActive)
	{
		HalExecElement* el = new HalExecElement(parent->checkHalId(ID_WaitForStorage));
		this->waitForStorage.elements.clear();
		this->waitForStorage.elements.push_back(el);
	
		storagePollingActive = this->parent->send(this->waitForStorage);
	}
	return storagePollingActive;
}


bool DebugManagerV3::stopStoragePolling()
{
	storagePollingActive = false;
	return parent->getFetHandle()->kill( waitForStorage.getResponseId() );
}


bool DebugManagerV3::reconnectJTAG()
{
	bool success = false;

	if ( FetHandle* fetHandle = parent->getFetHandle() )
	{
		if ( ConfigManager* cm = parent->getFetHandle()->getConfigManager() )
			success = (cm->start() > 0);

		fetHandle->resumeLoopCmd( waitForJState.getResponseId() );
		fetHandle->resumeLoopCmd( waitForEem.getResponseId() );
		fetHandle->resumeLoopCmd( waitForStorage.getResponseId() );
	}	
	return success;
}

bool DebugManagerV3::run (uint16_t controlMask, DebugEventTarget * cb, bool releaseJtag)
{
	MemoryManager* mm = this->parent->getMemoryManager();
	MemoryArea* cpu = mm->getMemoryArea("CPU");
	if (!cpu)
		return false;

	if(cb!=0)
	{
		cbx=cb;	
	}

	uint32_t pc, sr;
	cpu->read(0, &pc, 1);
	cpu->read(2, &sr, 1);

	if(mm->flushAll()==false)
	{
		return false;
	}

	queryLpm5State();

	cycleCounter_.reset();

	HalExecElement* el = new HalExecElement(this->parent->checkHalId(ID_RestoreContext_ReleaseJtag));
	this->parent->getWatchdogControl()->addParamsTo(el);
	el->appendInputData32(pc);
	el->appendInputData16(sr);
	el->appendInputData16(controlMask!=0? 0x0007: 0x0000);	// eem control bits 
	el->appendInputData16(opcode);		// mdb 
	el->appendInputData16(releaseJtag ? 1 : 0);
	opcode=0;
	
	HalExecCommand cmd;
	cmd.elements.push_back(el);

	if (!this->parent->send(cmd))
	{
		return false;
	}
	
	// handle lpmx5 polling
	if (releaseJtag)
	{
		pausePolling();
	}
	else
	{
		this->resumePolling();
	}	

	if (controlMask!=0 && !releaseJtag)
	{
		if (!activatePolling(controlMask))
		{
			return false;
		}
	}

	resetCycleCounterBeforeNextStep = true;

	return true;
}



void DebugManagerV3::localCallback (MessageDataPtr messageData, uint32_t clientHandle)
{
	eventNotifier.queueEvent(messageData);
}


void DebugManagerV3::runEvent(MessageDataPtr messageData)
{
	if (EmulationManagerPtr emManager = this->parent->getEmulationManager())
	{
		emManager->onEvent(messageData);
	}


	messageData->reset();

	uint16_t eventMask = 0;
	(*messageData) >> eventMask;

	if (eventMask & JSTATE_CAPTURE_FLAG)
	{
		uint32_t maskLow = 0;
		uint32_t maskHigh = 0;
		(*messageData) >> maskLow >> maskHigh;

		if ( maskHigh & (1 << 31) )
		{
			deviceInLpm5 = true;
			if(cbx)
			{
				cbx->event(DebugEventTarget::Lpm5Sleep);
			}
		}
		else
		{
			deviceInLpm5 = false;
			if(cbx)
			{
				cbx->event(DebugEventTarget::Lpm5Wakeup);
			}
		}
	}

	if (eventMask & BP_HIT_FLAG)
	{
		saveContext();
		if (cbx)
			cbx->event(DebugEventTarget::BreakpointHit);
	}
	
	if (eventMask & STATE_STORAGE_FLAG)
	{
		uint16_t numEntries = 0;
		(*messageData) >> numEntries;

		if (cbx)
			cbx->event(DebugEventTarget::Storage, numEntries, 0);
	}

	if (eventMask & VARIABLE_WATCH_FLAG)
	{
		if (cbx)
			cbx->event(DebugEventTarget::VariableWatch);
	}
}

struct PinState
{
	PinState(JTAG_PIN pin, uint16_t state, uint16_t delay = 0) 
		: mask(1<<pin), states(0), delay(delay) 
	{
		if (state != 0)
			states |= (1 << pin);
	}

	PinState& operator()(JTAG_PIN pin, uint16_t state)
	{
		mask |= (1 << pin);
		if (state != 0)
			states |= (1 << pin);
		else
			states &= ~(1 << pin);

		return *this;
	}
	
	PinState& setDelay(uint16_t ms) { delay = ms; return *this; }

	uint16_t mask;
	uint16_t states;
	uint16_t delay;
};

bool sendPinSequence(const list<PinState>& pinStates, DeviceHandleV3* handle)
{
	HalExecElement* el = new HalExecElement(ID_BitSequence);
	el->appendInputData8( (uint8_t)pinStates.size() );

	list<PinState>::const_iterator it = pinStates.begin();
	for (; it != pinStates.end(); ++it)
	{
		el->appendInputData16( it->states );
		el->appendInputData16( it->mask );
		el->appendInputData16( it->delay );
	}

	HalExecCommand cmd;
	cmd.elements.push_back(el);
	return handle->send(cmd);
}

bool DebugManagerV3::wakeupDevice()
{
    int i =0;
    
	list<PinState> stateChanges;

	bool isSleeping = true;

	while(isSleeping && i++ < 5)
    {
		stateChanges.push_back( PinState(JTAG_PIN_TST, true, 5) );
		stateChanges.push_back( PinState(JTAG_PIN_TST, false, 20) );		

		stateChanges.push_back( PinState(JTAG_PIN_RST, false, 10) );
		stateChanges.push_back( PinState(JTAG_PIN_RST, true, 5) );

		stateChanges.push_back( PinState(JTAG_PIN_TST, true, 5) );

		sendPinSequence(stateChanges, parent);
		stateChanges.clear();

		ConfigManager *cm = parent->getFetHandle()->getConfigManager();
		cm->start();

		isSleeping = queryLpm5State();
		if (!isSleeping)
		{
			break;
		}
    }

    return !isSleeping;
}

/* sync Jtag */
bool DebugManagerV3::stop(bool jtagWasReleased)
{
	bool success = false;
	int attemptsLeft = 3;
	bool isSleeping = false;
	bool wasInLpmx5 = false;

	// Pause the polling loop while we try to wake up the device and sync again
	this->pausePolling();
	
	do
	{
		success = false;
		
		if (queryLpm5State())
		{
			isSleeping = !wakeupDevice();
			wasInLpmx5 = true;
			this->resumePolling();
			boost::this_thread::sleep(boost::get_system_time() + boost::posix_time::milliseconds(500));	
			this->pausePolling();
		}

		// just save contex when no wakup was detected before 
		if(internalDebugState == D_LPMX5_WAKEUP_DETECTED)
		{
			wasInLpmx5 = true;	
		}

		success = saveContext();
	} 
	while ( isSleeping && --attemptsLeft );

	success = success && !isSleeping;

	//If the device was in LPMx5, read reset vector and set PC
	if ( success && wasInLpmx5)
	{
		MemoryManager* mm = parent->getMemoryManager();
		if ( MemoryArea* cpu = mm->getMemoryArea("CPU") )
		{
			uint32_t buffer[2];
			if ( mm->read(0xFFFE, buffer, 2) && mm->sync() )
			{
				cpu->write(0, buffer[0] | (buffer[1] << 8));
				cpu->write(2, 0); // BTT2001 status register bugfix
			}
		}				
	}

	// If the JTAG was released, we could not know that the device went into LPMx.5,
	// thus manually generate the event in this case
	if(jtagWasReleased && wasInLpmx5)
	{
		MessageDataPtr data(new MessageData);
		(*data) << (uint16_t)JSTATE_CAPTURE_FLAG;
		this->localCallback(data, 0);
	}

	return success;
}

bool DebugManagerV3::syncDeviceAfterLPMx5()
{
	internalDebugState = D_LPMX5_WAKEUP_DETECTED;
    return true;
}

bool DebugManagerV3::singleStep (uint32_t* cycles)
{
	MemoryManager* mm = this->parent->getMemoryManager();
	MemoryArea* cpu = mm->getMemoryArea("CPU");
	if (!cpu)
		return false;

	uint32_t pc, sr;
	cpu->read(0, &pc, 1);
	cpu->read(2, &sr, 1);
	
	/* flush and clear all caches */
	if (!mm->flushAll())
	{
		return false;
	}

	if (resetCycleCounterBeforeNextStep)
	{
		cycleCounter_.reset();
	}

	HalExecElement* el = new HalExecElement(ID_SetDeviceChainInfo);
	el->appendInputData16(static_cast<uint16_t>(this->parent->getDevChainInfo()->getBusId()));
	
	HalExecCommand cmd;	
	cmd.elements.push_back(el);

	HalExecElement* instructionRead = 0;
	if (cycles && emulationLevel < EMEX_EXTRA_SMALL_5XX)
	{
		//we run that inline instead of through the MemoryManager 
		instructionRead = new HalExecElement(this->parent->checkHalId(ID_ReadMemWords));
		instructionRead->appendInputData32(pc);
		instructionRead->appendInputData32(2);
		instructionRead->setOutputSize(4);
		cmd.elements.push_back(instructionRead);
	}

	boost::shared_ptr<WatchdogControl> wdt = this->parent->getWatchdogControl();
	el = new HalExecElement(this->parent->checkHalId(ID_SingleStep));
	// append stored watchdog value with hold bit
	wdt->addParamsTo(el);
	el->appendInputData32(pc);
	el->appendInputData16(static_cast<uint16_t>(sr));
	el->appendInputData16(7);		// mask for RestoreContext_ReleaseJtag (inside SingleStep macro)
	el->appendInputData16(opcode);
	el->appendInputData16(0);//releaseJtag
	opcode=0;
	el->setOutputSize(8);
	cmd.elements.push_back(el);
	
	parent->getFetHandle()->resumeLoopCmd( waitForStorage.getResponseId() );
	if (!this->parent->send(cmd))
	{
		return false;
	}
	parent->getFetHandle()->pauseLoopCmd( waitForStorage.getResponseId() );
	
	// Exit if device stepped into LPMx.5 mode
	if(queryLpm5State())
	{
		return true;
	}

	// get watchdog from single step macro
	uint16_t wdtCtrl = el->getOutputAt16(0);
	if (!WatchdogControl::checkRead(wdtCtrl))
	{
		return false;
	}
	// store watchdog values
	wdt->set(wdtCtrl);

	pc = el->getOutputAt32(2);
	sr = el->getOutputAt16(6);

	/* write PC and SR to MemoryManager to signal that _NO_ additional
	 * SyncJTAG is necessary
	 */
	cpu->write(0, pc);
	cpu->write(2, sr);
	cpu->getCacheCtrl()->fill(0, 16);

	if (cycles)
	{
		if (emulationLevel < EMEX_EXTRA_SMALL_5XX)
		{
			const uint16_t steppedIntoInterrupt = el->getOutputAt16(8);
			uint32_t instruction = instructionRead->getOutputAt32(0);
			cycleCounter_.countInstruction(instruction, steppedIntoInterrupt != 0);
		}
		*cycles = static_cast<uint32_t>(cycleCounter_.read());
	}

	resetCycleCounterBeforeNextStep = false;
	
	return true;
}


bool DebugManagerV3::initEemRegister()
{
	bool success = false;

	MemoryManager *mm = this->parent->getMemoryManager();
	EemMemoryAccess *ema = mm ? dynamic_cast<EemMemoryAccess *>(mm->getMemoryArea("EEM")) : 0;

	if (ema)
	{
		ema->writeEemRegister(GENCTRL,0x0);
		ema->writeEemRegister(MODCLKCTRL0, defaultMclkcntrl0);
		ema->writeEemRegister(GENCLKCTRL, DefaultClkCntrl);
		success = ema->sync();
	}

	cycleCounter_.configure();

	if (EmulationManagerPtr emuManager = parent->getEmulationManager())
		emuManager->writeConfiguration();

	return success;
}

bool DebugManagerV3::eemWriteRegister(uint32_t addr, uint32_t value)
{
	MemoryManager *mm = this->parent->getMemoryManager();
	EemMemoryAccess *ema = mm ? dynamic_cast<EemMemoryAccess *>(mm->getMemoryArea("EEM")) : 0;

	return ema && ema->writeEemRegister((EemRegister)addr, value) && ema->sync();
}


bool DebugManagerV3::eemReadRegister(uint32_t addr, uint32_t* buffer)
{
	MemoryManager *mm = this->parent->getMemoryManager();
	EemMemoryAccess *ema = mm ? dynamic_cast<EemMemoryAccess *>(mm->getMemoryArea("EEM")) : 0;

	return ema && ema->readEemRegister((EemRegister)(addr&0xfffffffe), buffer) && ema->sync();
}


uint8_t DebugManagerV3::getClockControl() const
{
	return clockControl;
}

uint16_t DebugManagerV3::getClockControlSetting() const
{
	return genclkcntrl;
}

void DebugManagerV3::setClockControlSetting(uint16_t clkcntrl)
{
	genclkcntrl = clkcntrl;
}

uint16_t DebugManagerV3::getClockModuleDefaultSetting() const
{
	return defaultMclkcntrl0;
}

uint16_t DebugManagerV3::getClockModuleSetting() const
{
	return mclkcntrl0;
}

void DebugManagerV3::setClockModuleSetting(uint16_t modules)
{
	mclkcntrl0 = modules;
}

char ** DebugManagerV3::getModuleStrings(uint32_t * n) const
{
	*n=nModuleStrings;
	return moduleStrings;
}

char ** DebugManagerV3::getClockStrings(uint32_t * n) const
{
	*n=nClockStrings;
	return clockStrings;
}

void DebugManagerV3::setOpcode(uint16_t value)
{
	opcode=(uint16_t)value;
}


bool DebugManagerV3::saveContext()
{
	MemoryManager* mm = this->parent->getMemoryManager();

	MemoryArea* cpu = mm->getMemoryArea("CPU");
	if (!cpu)
		return false;

	HalExecElement* el0 = new HalExecElement(this->parent->checkHalId(ID_SyncJtag_Conditional_SaveContext));
	this->parent->getWatchdogControl()->addParamsTo(el0, true);
	el0->setOutputSize(8);
	
	HalExecCommand cmd;
	cmd.elements.push_back(el0);

	if (!this->parent->send(cmd))
	{
		return false;
	}

	uint16_t wdtCtrl = el0->getOutputAt16(0);

	if (!WatchdogControl::checkRead(wdtCtrl))
		return false;
	// sync watchdog after stopping CPU
	this->parent->getWatchdogControl()->set(wdtCtrl);

	// get register values
	uint32_t pc = el0->getOutputAt32(2);
	uint32_t sr = el0->getOutputAt16(6);

	cpu->write(0, pc);
	cpu->write(2, sr);
	cpu->getCacheCtrl()->fill(0, 16);

	cycleCounter_.read();

	return true;
}

uint8_t DebugManagerV3::getRunEemResponseId()
{
	return waitForEem.getResponseId();
}

void DebugManagerV3::setLpmDebugging(bool enable)
{
	lpmDebuggingEnabled = enable;
}

bool DebugManagerV3::getLpmDebugging()
{
	return lpmDebuggingEnabled;
}

bool DebugManagerV3::activateJStatePolling(DebugEventTarget * cb)
{
	//If loop is already running, just return
	if ( waitForJState.getResponseId() != 0 )
	{
		return true;
	}

	uint64_t mask = 0x8000000100000000LL; // Only interested in bit 63

	if(cb!=0)
	{
		cbx = cb;
	}

	HalExecElement* el = new HalExecElement(parent->checkHalId(ID_PollJStateReg));
	el->appendInputData32((uint32_t)(mask & 0xFFFFFFFF));
	el->appendInputData32((uint32_t)(mask >> 32));
	el->appendInputData16((uint16_t)false);
	el->setOutputSize(2);

	this->waitForJState.elements.clear();
	this->waitForJState.elements.push_back(el);
		
	if (!this->parent->send(this->waitForJState))
	{
		return false;
	}

	this->pausePolling();

	return true;
}

bool DebugManagerV3::queryLpm5State()
{
	if (!getLpmDebugging())
		return false;

	const uint64_t bit = 1;
	uint64_t mask = (bit << 63) | (bit << 62);

	HalExecElement* el = new HalExecElement(parent->checkHalId(ID_PollJStateReg));
	el->appendInputData32((uint32_t)(mask & 0xFFFFFFFF));
	el->appendInputData32((uint32_t)(mask >> 32));
	el->appendInputData16((uint16_t)1);
	el->setOutputSize(10);

	HalExecCommand cmd;
	cmd.elements.push_back(el);

	uint64_t jstate = 0;
	
	if(this->parent->send(cmd))
	{
		jstate  = el->getOutputAt32(2);
		jstate |= (uint64_t)el->getOutputAt32(6) << 32;
	}
	return (( jstate & (bit << 63) ) != 0);
}

bool DebugManagerV3::isDeviceInLpm5()
{
	return deviceInLpm5;
}

void DebugManagerV3::pausePolling()
{
	if(parent->getFetHandle())
	{
		parent->getFetHandle()->pauseLoopCmd( waitForJState.getResponseId() );
		parent->getFetHandle()->pauseLoopCmd( waitForEem.getResponseId() );
		parent->getFetHandle()->pauseLoopCmd( waitForStorage.getResponseId() );
	}
}

void DebugManagerV3::resumePolling()
{
	if(parent->getFetHandle())
	{
		parent->getFetHandle()->resumeLoopCmd( waitForJState.getResponseId() );
		parent->getFetHandle()->resumeLoopCmd( waitForEem.getResponseId() );
		parent->getFetHandle()->resumeLoopCmd( waitForStorage.getResponseId() );
	}
}
