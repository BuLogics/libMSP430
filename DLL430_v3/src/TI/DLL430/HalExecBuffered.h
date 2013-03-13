/*
 * HalExecBuffered.h
 *
 * Sending and receiving on hal level.
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
#ifndef DLL430_HALEXECBUFFERED_H
#define DLL430_HALEXECBUFFERED_H

#include "HalExecElement.h"
#include "HalResponseHandler.h"
#include "IoChannel.h"
#include <inttypes.h>
#include <vector>
#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/function.hpp>

#define FOLLOWER 0x80
#define LOOP_FLAG 0x40



namespace TI
{
	namespace DLL430
	{
		class MessageData;

		typedef boost::function2<void, boost::shared_ptr<MessageData>, uint32_t> EventCallback;

		typedef boost::shared_ptr<HalExecBuffered> HalExecBufferedPtr;

		class HalExecBuffered : public HalResponseHandler
		{
		public:
			typedef boost::ptr_vector<HalExecElement> list_type;

			HalExecBuffered ();
			~HalExecBuffered ();
			
			bool send (list_type &, FetControl&, IoChannel&);
			
			void recv (FetControl& , HalResponse& resp);
			bool isAsync();
			bool isContinuous();

			uint8_t getResponseId();

			void setAsyncMode(bool continued);

			void setTimeout(uint32_t msec);

			void setCallBack(const EventCallback& callback, uint32_t clientHandle);

			HalResponseHandlerPtr responseHandlerPtr;

		private:
			void setDelay(int time);
			
			void setEventType(uint8_t type);

			bool waitForSingleEvent(int timeout, int sleep, HalExecElement& el, uint8_t id);

			void createMessage(std::vector<uint8_t>& tdata,
									uint8_t type,
									uint8_t response,
									uint16_t addr,
									bool hasaddr,
									uint8_t * tbuf);

			void sendAck(uint8_t response, IoChannel& chan, std::vector<uint8_t>& tdata);
			bool sendElement(HalExecElement& el, FetControl& fetCtrl, IoChannel& chan);
			bool sendAsync(HalExecElement& el, FetControl& fetCtrl, IoChannel& chan, bool continued);
			bool checkException(HalResponse& resp);
			
			bool recv_data (HalResponse& resp);


			boost::condition_variable dataCondition;
			boost::mutex dataMutex;

			HalExecElement * trans;

			uint8_t buf[256];
			int cmd_timeout;
			bool hal_error;
			list_type * elem;
			bool async;
			bool cont;
			uint8_t tout; 
			int syncDelay;
			
			IoChannel * tmp_channel;

			EventCallback info_callback;

			uint32_t extClientHandle;

			uint8_t eventType;
			uint8_t loopCmdId;
		};

	};
};

#endif /* DLL430_HALEXEBUFFERED_H */
