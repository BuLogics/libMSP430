/*
 * EventNotifier.h
 *
 * Copyright (C) 2009 - 2011 Texas Instruments Incorporated - http://www.ti.com/ 
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

#if _MSC_VER > 1000
#pragma once
#endif
#ifndef DLL430_EVENT_NOTIFIER_H
#define DLL430_EVENT_NOTIFIER_H


#include <boost/function.hpp>

#include "MessageQueue.h"


//The event notifier will run a separate thread to process queued events by calling a 
//specified event handler callback function on each of them.
template<class EventType>
class EventNotifier
{	
public:
	typedef	boost::function1<void, EventType> Callback;

	EventNotifier() : mRunning(false), mCallback(0) {}

	explicit EventNotifier(Callback callback) : mRunning(false), mCallback(callback) {}

	~EventNotifier() { stopProcessingEvents(); }

	void startProcessingEvents() 
	{
		if (!mRunning)
		{
			mRunning = true;
			mThread = boost::thread(&EventNotifier<EventType>::execute, this);
		}
	}

	void stopProcessingEvents() 
	{
		if (mRunning)
		{
			mRunning = false;
			mMessageQueue.cancelWait();
			mThread.join();
		}
	}

	void queueEvent(EventType evt) { mMessageQueue.queueMessage(evt); }

	//Do not call this while events are being processed (mCallback is not thread safe)
	void setEventHandler(const Callback& callback) { mCallback = callback; }

private:
	void execute()
	{
		while (mRunning)
		{
			EventType evt = mMessageQueue.retrieveMessage();
			if (evt && mCallback)
			{
				mCallback(evt);
			}
		}
	}

	bool mRunning;
	Callback mCallback;
	boost::thread mThread;

	//Since it is critical to cancel waiting for messages before the queue is destroyed,
	//the queue has been made a member instead of being passed from the outside
	MessageQueue<EventType> mMessageQueue;
};


#endif
