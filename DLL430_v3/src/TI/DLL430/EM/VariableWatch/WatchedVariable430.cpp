/*
 * WatchedVariable430.cpp
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


#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>

#include "WatchedVariable430.h"
#include "../TriggerCondition/ITriggerCondition.h"


using namespace TI::DLL430;


WatchedVariable430::WatchedVariable430(uint32_t address, uint32_t bitSize, 
										TriggerConditionPtr lowWordCondition,
										TriggerConditionPtr highWordCondition)
	: conditionLow_(lowWordCondition)
	, conditionHigh_(highWordCondition)
	, address_(address)
	, bits_(bitSize)
	, value_(0)
	, validLowWord_(false)
	, validHighWord_(bits_ <= 16)
	, enabled_(true)
{
	conditionLow_->addReaction(TR_VARIABLE_WATCH);

	if (conditionHigh_)
	{
		conditionHigh_->addReaction(TR_VARIABLE_WATCH);
	}
}


WatchedVariable430::~WatchedVariable430()
{
	disable();
}



void WatchedVariable430::setValue(uint16_t value) 
{
	boost::mutex::scoped_lock lock(accessMutex_);

	validLowWord_ = true;

	const uint16_t valueMask = (bits_ == 8) ? 0xFF : 0xFFFF;

	value_ = (value_ & 0xFFFF0000) | (value & valueMask);
}



void WatchedVariable430::setValueHighWord(uint16_t value) 
{
	boost::mutex::scoped_lock lock(accessMutex_);

	validHighWord_ = true;

	value_ = ((uint32_t)value << 16) | (value_ & 0xFFFF);
}



uint32_t WatchedVariable430::address() const
{
	return address_; 
}



uint32_t WatchedVariable430::bitSize() const
{
	return bits_; 
}



uint32_t WatchedVariable430::value() const
{
	boost::mutex::scoped_lock lock(accessMutex_);
	return value_; 
}



bool WatchedVariable430::isValid() const
{
	boost::mutex::scoped_lock lock(accessMutex_);
	return validLowWord_ && validHighWord_;
}



void WatchedVariable430::enable()
{
	enabled_ = true;
	conditionLow_->addReaction(TR_VARIABLE_WATCH);

	if (conditionHigh_)
	{
		conditionHigh_->addReaction(TR_VARIABLE_WATCH);
	}
}


void WatchedVariable430::disable()
{
	enabled_ = false;
	conditionLow_->removeReaction(TR_VARIABLE_WATCH);

	if (conditionHigh_)
	{
		conditionHigh_->removeReaction(TR_VARIABLE_WATCH);
	}
}
