/*
 * WatchedVariable430.h
 *
 * Watched variable implementation for 430
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


#ifndef DLL430_WATCHED_VARIABLE_430_H
#define DLL430_WATCHED_VARIABLE_430_H

#include <stdint.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include "IWatchedVariable.h"

namespace TI { namespace DLL430 {


class WatchedVariable430 : public IWatchedVariable
{
public:
	WatchedVariable430(uint32_t address, uint32_t bitSize, 
						TriggerConditionPtr lowWordCondition, 
						TriggerConditionPtr highWordCondition = TriggerConditionPtr());
	
	virtual ~WatchedVariable430();

	virtual uint32_t address() const;
	
	virtual uint32_t bitSize() const;

	virtual uint32_t value() const;

	virtual bool isValid() const;

	virtual void enable();

	virtual void disable();

private:
	friend class StateStorage430;
	
	void setValue(uint16_t value);

	void setValueHighWord(uint16_t value);

	TriggerConditionPtr conditionLow_;
	TriggerConditionPtr conditionHigh_;

	uint32_t address_;
	uint32_t bits_;
	uint32_t value_;

	bool validLowWord_;
	bool validHighWord_;
	bool enabled_;

	mutable boost::mutex accessMutex_;
};


}}

#endif
