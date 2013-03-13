/*
 * WatchdogControl.cpp
 *
 * Handles control over watchdog.
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

#include "WatchdogControl.h"

using namespace TI::DLL430;

WatchdogControl::WatchdogControl (uint16_t addr)
 : addr(addr)
 , value(0)
{
}

WatchdogControl::~WatchdogControl ()
{
}

bool WatchdogControl::checkRead (uint16_t wdtCtrl)
{
	// watchdog password value: 0x69 for reading
	return (((wdtCtrl >> 8) & 0xFF) == 0x69);
}

bool WatchdogControl::checkWrite (uint16_t wdtCtrl)
{
	// watchdog password value: 0x5A for writing
	return (((wdtCtrl >> 8) & 0xFF) == 0x5A);
}

void WatchdogControl::set (uint16_t wdtCtrl)
{
	this->value = (wdtCtrl & 0xFF);
}

uint16_t WatchdogControl::get ()
{
	return (0x69 << 8 | this->value);
}

void WatchdogControl::addParamsTo (HalExecElement* el, bool force_hold) const
{

	el->appendInputData16(this->addr);
	uint16_t wdtCtrl = 0x5A << 8 | this->value;
	// if true set the watchdog hold bit
	if (force_hold)
		wdtCtrl |= (1 << 7);

	el->appendInputData16(wdtCtrl);
}

uint16_t WatchdogControl::getAddress () const
{
	return this->addr;
}
