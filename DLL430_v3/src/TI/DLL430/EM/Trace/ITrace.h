/*
 * ITrace.h
 *
 * Interface for trace module
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


#ifndef DLL430_ITRACE_H
#define DLL430_ITRACE_H

#include <stdint.h>
#include <vector>
#include <boost/shared_ptr.hpp>


namespace TI { namespace DLL430 {


class ITriggerCondition;
class MessageData;

typedef boost::shared_ptr<ITriggerCondition> TriggerConditionPtr;
typedef boost::shared_ptr<MessageData> MessageDataPtr;

//Trace data contains the values of the address bus, data bus and control register
struct TraceData
{
	TraceData() : mab(0), mdb(0), ctl(0) {}

	uint32_t mab;
	uint16_t mdb;
	uint16_t ctl;
};

typedef std::vector<TraceData> TraceBuffer;


//Trace will store the most recent device states in a ring buffer and notify on trace storage events
class ITrace
{
public:
	virtual ~ITrace() {}

	//Add a trigger condition to act as trace trigger (there is no distinction between different triggers)
	virtual void addTriggerCondition(TriggerConditionPtr triggerCondition) = 0;

	//Remove all trace triggers
	virtual void clearTriggerConditions() = 0;
	
	//Enable/disable trace feature (on 430 trace is mutually exclusive with variable watch)
	virtual void enable() = 0;
	virtual void disable() = 0;
	
	//Reset/restart trace module (must always be called after changing sequencer configuration)
	virtual void reset() = 0;
	
	//Trace will start when a trace trigger is hit
	virtual void setStartOnTrigger(bool startOnTrigger) = 0;

	//Trace will stop when a trace trigger is hit
	virtual void setStopOnTrigger(bool stopOnTrigger) = 0;
	
	//Store state on instruction fetch, when trace trigger is hit or on every clock cycle
	virtual void setStoreOnInstructionFetch() = 0;
	virtual void setStoreOnTrigger() = 0;
	virtual void setStoreOnClock() = 0;
	
	//Store states indefinitely in ring buffer or stop trace when buffer is full
	virtual void setStoreContinuously() = 0;
	virtual void setStoreUntilFull() = 0;

	//Return content of trace buffer at time of last trace event
	virtual const TraceBuffer getTraceData() = 0;

	//Only used by EmulationManager
	virtual void initialize() = 0;
	virtual void cleanup() = 0;	
	virtual void writeConfiguration() = 0;
	virtual void onEvent(MessageDataPtr msgData) = 0;
};

}}

#endif
