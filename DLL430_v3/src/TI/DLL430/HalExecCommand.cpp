/*
 * HalExecCommand.cpp
 *
 * Forwards calls to HalExecBuffered.
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

#include "HalExecCommand.h"
#include "HalResponse.h"
#include "FetControl.h"
#include "IoChannel.h"

using namespace TI::DLL430;

HalExecCommand::HalExecCommand ()
	: fetControl(0)
{
	exec = HalExecBufferedPtr(new HalExecBuffered());
	exec->responseHandlerPtr = exec;
}

HalExecCommand::~HalExecCommand ()
{
	if (fetControl)
		fetControl->unregisterResponseHandler(exec);
	
	//Remove self-reference to allow deletion
	exec->responseHandlerPtr.reset();
}

bool HalExecCommand::send (FetControl& fetCtrl, IoChannel& chan)
{
	fetControl = &fetCtrl;
	return exec->send(elements, fetCtrl, chan);
}

void HalExecCommand::setCallBack(const EventCallback& callback, uint32_t clientHandle)
{
	exec->setCallBack(callback,clientHandle);
}

uint8_t HalExecCommand::getResponseId()
{
	return exec->getResponseId();
}

void HalExecCommand::setAsyncMode(bool continued)
{
	exec->setAsyncMode(continued);
}

void HalExecCommand::setTimeout(uint32_t msec)
{
	exec->setTimeout(msec);
}
