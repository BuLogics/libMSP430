/*
 * EemTriggerAccess.cpp
 *
 * Memory class for EEM trigger access.
 *
 * Copyright (C) 2008 - 2011 Texas Instruments Incorporated - http://www.ti.com/ 
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

#include "EemTriggerAccess.h"

using namespace TI::DLL430;

EemTriggerAccess::EemTriggerAccess ()
 : mask(0xFFFFFFFF)
 , value(0)
 , control(0)
 , combination(0)
 , base (0)
{
}

void EemTriggerAccess::init(uint8_t id)
{
	if (id>16)
		return;

	base=id;
	this->reset();
}

void EemTriggerAccess::reset()
{
	type=MEMBUS_ADDRESS;	// default
	mask=0;
	control=0;
	value=0;
	combination=0;			// default to 0 > not active
}


EemTriggerAccess::~EemTriggerAccess ()
{
}

bool EemTriggerAccess::setType (TI::DLL430::TriggerType t_type)
{
	this->control=0;		// reset control register anyway

	if (t_type==MEMBUS_DATA)
		this->control |= 0x1;

	type=t_type;

	return true;
}

void EemTriggerAccess::enableHold (bool enable)
{
	if (type!=REGISTER)
		return;

	if (enable)
		this->control |= (1 << 6);
	else
		this->control &= ~(1 << 6);
}

void EemTriggerAccess::setRegister(TI::DLL430::Register reg)
{
	if(type!=REGISTER)
		return;

	this->control &= ~0xF00;
	this->control |= ((reg & 0xF) << 8);
}

bool EemTriggerAccess::setMask (uint32_t mask)
{
	// some models do only allow some special masks
	// -> DebugManager
	this->mask = mask;
	return true;
}

void EemTriggerAccess::setValue (uint32_t value)
{
	this->value = value;
}

void EemTriggerAccess::setControl (uint32_t value)
{
	this->control = value;
}

void EemTriggerAccess::setCondition(TriggerCondition c)
{
	if((type!=MEMBUS_ADDRESS)&&(type!=MEMBUS_DATA))
		return;

	this->control &= ~0x66;
	this->control |= ((c & 0xC) << 3) | ((c & 0x3) << 1);
}

void EemTriggerAccess::setCompare (CompareMode mode)
{
	this->control &= ~0x18;
	this->control |= (mode << 3) & 0x18;
}

void EemTriggerAccess::setCombination(uint32_t value)
{
	this->combination|=value;
}

void EemTriggerAccess::setDefaultCombination()
{
	this->combination=(0x1<<base);
}

uint32_t EemTriggerAccess::getValue ()
{
	return this->value;
}

uint32_t EemTriggerAccess::getControl ()
{
	return this->control;
}

uint32_t EemTriggerAccess::getMask ()
{
	return this->mask;
}

uint32_t EemTriggerAccess::getCombination ()
{
	return this->combination;
}

TriggerType EemTriggerAccess::getTriggerType()
{
	return this->type;
}

uint8_t EemTriggerAccess::getId()
{
	return base;
}

uint32_t EemTriggerAccess::getValueAdr() const
{
	return (uint32_t)base<<3;
}

uint32_t EemTriggerAccess::getControlAdr()
{
	return ((uint32_t)base<<3)+2;
}

uint32_t EemTriggerAccess::getMaskAdr()
{
	return ((uint32_t)base<<3)+4;
}

uint32_t EemTriggerAccess::getCombinationAdr()
{
	return ((uint32_t)base<<3)+6;
}
