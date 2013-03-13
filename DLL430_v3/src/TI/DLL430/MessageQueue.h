/*
 * MessageQueue.h
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
#ifndef DLL430_MESSAGE_QUEUE_H
#define DLL430_MESSAGE_QUEUE_H


#include <queue>
#include <boost/thread/condition.hpp>


//MessageQueue is meant to be used with either primitive types or pointers. While it
//technically works with any Object that can be default constructed, the preferred
//usage is in combination with shared_ptr (the queue itself will NOT delete any objects)

//While only intended and tested to be used with a single reader, it should be possible to
//have several threads waiting for messages, which will only be processed by the first thread
//to receive them.

template <class MessageType>
class MessageQueue
{
public:
	//While a queue will automatically remove all content on destruction
	//it is important to not do so while another thread is adding or removing elements
	~MessageQueue()
	{
		boost::mutex::scoped_lock l(mMutex);
		while (!mMessages.empty())
		{
			mMessages.pop();
		}
		mMessageAvailable.notify_all();
	}

	//Will wake up a single receiver to process the queued message
	void queueMessage(MessageType msg)
	{
		boost::mutex::scoped_lock l(mMutex);
		mMessages.push(msg);
		mMessageAvailable.notify_one();
	}

	//In case of cancelling, this will return a default constructed message
	//(ie. 0, NULL, empty string or depending on the type of MessageType)
	MessageType retrieveMessage()
	{
		boost::mutex::scoped_lock l(mMutex);

		if (mMessages.empty())		
		{
			mMessageAvailable.wait(l);
		}

		MessageType msg = MessageType();

		if (!mMessages.empty())
		{
			msg = mMessages.front();
			mMessages.pop();
		}

		return msg;
	}

	//Cancel will wake up all threads waiting for a message, so they can exit the retrieveMessage function
	void cancelWait()
	{
		boost::mutex::scoped_lock l(mMutex);
		mMessageAvailable.notify_all();
	}

private:
	boost::condition_variable mMessageAvailable;
	boost::mutex mMutex;
	std::queue<MessageType> mMessages;
};

#endif
