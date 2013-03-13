/*
 * CycleCounter430.h
 *
 * Cycle counter functionality for MSP430.
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


#ifndef DLL430_CYCLE_COUNTER_430_H
#define DLL430_CYCLE_COUNTER_430_H

#include <stdint.h>
#include <vector>
#include "ICycleCounter.h"

namespace TI { namespace DLL430 {

class CycleCounter430 : public ICycleCounter
{
public:
	CycleCounter430(int numCounters);
	virtual ~CycleCounter430();

	virtual void initialize() {}
	virtual void cleanup() {}	
	virtual void writeConfiguration();

	virtual void setCounterCondition(int counter, TriggerConditionPtr condition);
	virtual void clearCounterCondition(int counter);

	virtual uint64_t getCounterValue(int counter);
	virtual void setCounterValue(int counter, uint64_t value);
	virtual void setCounterMode(int counter, CounterMode mode);
	
	virtual void readCounter(int counter);
	virtual void writeCounter(int counter);
	virtual void resetCounter(int counter);

private:
	struct Counter
	{
		Counter() : cycleCounterCntrl(0), value(0) {}

		uint16_t cycleCounterCntrl;
		uint64_t value;
	};

	std::vector<Counter> mCounters;
	TriggerConditionPtr mCondition;
};

}}

#endif
