/*
 * ICycleCounter.h
 *
 * Interface for cycle counter module
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



#ifndef DLL430_ICYCLE_COUNTER_H
#define DLL430_ICYCLE_COUNTER_H

#include <stdint.h>
#include "../TriggerCondition/ITriggerConditionManager.h"

namespace TI { namespace DLL430 {

enum CounterMode {};


class ICycleCounter
{
public:
	virtual ~ICycleCounter() {}

	//Limit to AddressRangeCondition ??
	virtual void setCounterCondition(int counter, TriggerConditionPtr condition) = 0;
	virtual void clearCounterCondition(int counter) = 0;

	virtual uint64_t getCounterValue(int counter) = 0;
	virtual void setCounterValue(int counter, uint64_t value) = 0;
	
	virtual void readCounter(int counter) = 0;
	virtual void writeCounter(int counter) = 0;
	virtual void resetCounter(int counter) = 0;

	//Only used by EmulationManager
	virtual void initialize() = 0;
	virtual void cleanup() = 0;	
	virtual void writeConfiguration() = 0;
};

}}

#endif
