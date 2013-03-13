/*
 * CycleCounter430.cpp
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


#include "CycleCounter430.h"
#include "../TriggerCondition/ITriggerCondition.h"
#include "../EemRegisters/EemRegisterAccess.h"

using namespace TI::DLL430;

namespace {
	enum ClockControlRegister
	{
		CCNT0CTL   = 0xb0,
		CCNT0L	   = 0xb2,
		CCNT0H	   = 0xb4,
		CCNT1CTL   = 0xb8,
		CCNT1L	   = 0xba,
		CCNT1H	   = 0xbc
	};

	const ClockControlRegister cycleCounterRegisters[2][3] = { {CCNT0CTL, CCNT0L, CCNT0H}, 
															   {CCNT1CTL, CCNT1L, CCNT1H} };

	struct LfsrRegister
	{
		LfsrRegister(uint64_t value = 0) : value(value) {}

		union
		{
			struct { uint32_t low, high; };
			uint64_t value;
		};
	};

	uint64_t toLFSR(uint64_t value)
	{
		const uint64_t hex2lfsr[16] = {0x0, 0x1, 0x2, 0x5, 0xa, 0x4, 0x9, 0x3, 0x6, 0xd, 0xb, 0x7, 0xe, 0xc, 0x8, 0xf};
	
		uint64_t lfsr = 0;

		for (int i = 0; i < 10; ++i)
		{
			uint8_t fraction = (value % 15);
			lfsr |= hex2lfsr[fraction] << (i*4);
			value /= 15;
		}
		return lfsr;
	}

	uint64_t fromLFSR(uint64_t lfsr)
	{
		const uint32_t lfsr2hex[16] = {0x0, 0x1, 0x2, 0x7, 0x5, 0x3, 0x8, 0xb, 0xe, 0x6, 0x4, 0xa, 0xd, 0x9, 0xc, 0};

		uint64_t value = 0;
		uint32_t factor = 1;
	
		for (int i = 0; i < 10; ++i)
		{
			value += factor * lfsr2hex[(lfsr & 0xf)];
			lfsr >>= 4;
			factor *= 15;
		}
		return value;
	}

}




CycleCounter430::CycleCounter430(int numCounters)
	: mCounters(numCounters)
{
	for (size_t i = 0; i < mCounters.size(); ++i)
	{
		//Start/stop with Jtag control, count CPU bus cycles
		mCounters[i].cycleCounterCntrl = 0x6;
	}
}


CycleCounter430::~CycleCounter430()
{
}
	

void CycleCounter430::writeConfiguration()
{
	for (size_t i = 0; i < mCounters.size(); ++i)
	{
		writeCounter(i);
	}
}


void CycleCounter430::setCounterCondition(int counter, TriggerConditionPtr condition)
{
	//Only counter 1 has a reaction register
	if (counter == 1)
	{
		mCondition = condition;
		mCondition->addReaction(TR_CYCLE_COUNTER);		
	}
}


void CycleCounter430::clearCounterCondition(int counter)
{
	if (counter == 1)
	{
		mCondition->removeReaction(TR_CYCLE_COUNTER);
		mCondition.reset();
	}
}


uint64_t CycleCounter430::getCounterValue(int counter)
{
	return mCounters[counter].value;
}


void CycleCounter430::setCounterValue(int counter, uint64_t value)
{
	mCounters[counter].value = value;
}


void CycleCounter430::setCounterMode(int counter, CounterMode mode)
{
	//@TODO consider all modes (start, stop, clear conditions, counting modes)
	mCounters[counter].cycleCounterCntrl &= ~0xf;
	mCounters[counter].cycleCounterCntrl |= 0x6;
}


void CycleCounter430::readCounter(int counter)
{
	LfsrRegister lfsrRegister;
	//Read
	mCounters[counter].cycleCounterCntrl = readEemRegister(cycleCounterRegisters[counter][0]);
	lfsrRegister.low = readEemRegister(cycleCounterRegisters[counter][1]);
	lfsrRegister.high = readEemRegister(cycleCounterRegisters[counter][2]);

	mCounters[counter].value = fromLFSR(lfsrRegister.value);
}


void CycleCounter430::writeCounter(int counter)
{
	LfsrRegister lfsrRegister( toLFSR(mCounters[counter].value) );
	
	writeEemRegister(cycleCounterRegisters[counter][0], mCounters[counter].cycleCounterCntrl);
	writeEemRegister(cycleCounterRegisters[counter][1], lfsrRegister.low);
	writeEemRegister(cycleCounterRegisters[counter][2], lfsrRegister.high);
}


void CycleCounter430::resetCounter(int counter)
{
	mCounters[counter].cycleCounterCntrl |= (1<<6);
}
