/*
 * StateStorage430.h
 *
 * Shared trace and variable watch implementation for 430
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


#ifndef DLL430_STATE_STORAGE_430_H
#define DLL430_STATE_STORAGE_430_H

#include <stdint.h>
#include <vector>
#include <boost/weak_ptr.hpp>
#include <boost/thread.hpp>

#include "../Trace/Trace430.h"
#include "../VariableWatch/VariableWatch430.h"
#include "../VariableWatch/WatchedVariable430.h"



namespace TI { namespace DLL430 {


class MessageData;
typedef boost::shared_ptr<MessageData> MessageDataPtr;


enum StateStorageControlBits
{
	STOR_EN = 0x1,
	STOR_MODE_TRIGGER = 0x0,
	STOR_MODE_INSTR_FETCH = 0x2,
	STOR_MODE_VAR_WATCH = 0x4,
	STOR_MODE_ALL_CYCLES = 0x6,
	STOR_MODE_CLEAR = 0x6,
	STOR_ONE_SHOT = 0x8,
	STOR_START_TRIG = 0x10,
	STOR_STOP_TRIG = 0x20,
	STOR_RESET = 0x40
};



class StateStorage430 : public Trace430InterfaceSplitter, public VariableWatch430InterfaceSplitter
{
public:
	StateStorage430();
	~StateStorage430();

	virtual void initialize();
	virtual void cleanup();
	
	virtual void writeConfiguration();


	//Trace Interface implementation
	virtual void onEventTrace(MessageDataPtr messageData);

	virtual void enableTrace();
	virtual void disableTrace();

	virtual void addTriggerCondition(TriggerConditionPtr triggerCondition);
	virtual void clearTriggerConditions();

	virtual void reset();

	virtual void setStartOnTrigger(bool startOnTrigger);
	virtual void setStopOnTrigger(bool stopOnTrigger);

	virtual void setStoreOnTrigger();
	virtual void setStoreOnInstructionFetch();
	virtual void setStoreOnClock();

	virtual void setStoreContinuously();
	virtual void setStoreUntilFull();

	virtual const TraceBuffer getTraceData();



	//VariableWatch Interface implementation
	virtual void onEventVWatch(MessageDataPtr messageData);

	virtual void enableVWatch();
	virtual void disableVWatch();

	virtual WatchedVariablePtr createWatchedVariable(uint32_t address, uint32_t bitSize, TriggerConditionManagerPtr tcManager);

private:
	void updateWatchedVariable(uint32_t triggerId, uint16_t value);


	uint16_t controlRegister_;

	std::vector<TriggerConditionPtr> triggerConditions_;
	
	TraceBuffer traceBuffer_;

	boost::mutex traceBufferMutex_;

	std::vector<boost::weak_ptr<WatchedVariable430> > watchedVariables_;
};






}}

#endif
