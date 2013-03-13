/*
 * FetControl.h
 *
 * Flash Emulation Tool Control.
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

#if _MSC_VER > 1000
#pragma once
#endif
#ifndef DLL430_FETCONTROL_H
#define DLL430_FETCONTROL_H

#include <map>
#include <vector>
#include <inttypes.h>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/function.hpp>
#include "WatchdogControl.h"
#include "IoChannel.h"
#include "FetControlThread.h"
#include "HalResponseHandler.h"

namespace TI
{
	namespace DLL430
	{

		class HalCommand;

		typedef boost::function4<void, uint32_t, uint32_t, uint32_t, uint32_t> NotifyCallback;

		class FetControl
		{
		public:
			FetControl (IoChannel* channel);
			~FetControl ();
			typedef boost::mutex mutex_type;
			void setObjectDbEntry (size_t index) const ;
			void setFunctionMap (IoChannel::function_map_type*) const;
			
			typedef std::map<unsigned long, uint16_t> function_map_type;

			bool send (HalExecCommand& command, function_map_type* fctMap = NULL);

			bool sendData(const std::vector<uint8_t>& data);

			void shutdown();

			bool resetCommunication();
			bool kill (uint8_t id);
			bool pauseLoopCmd (uint8_t id);
			bool resumeLoopCmd (uint8_t id);

			uint8_t createResponseId (bool reserveId = false);
			bool registerResponseHandler (uint8_t id, HalResponseHandlerPtr h);
			void unregisterResponseHandler (uint8_t id, HalResponseHandlerPtr h);
			void unregisterResponseHandler (HalResponseHandlerPtr h);

			HalResponseHandlerPtr findResponseHandler (uint8_t id);
			HalResponseHandlerPtr findResponseHandler (HalResponseHandlerPtr h);
			void clearResponse();

			uint8_t getNextId(uint8_t ref);

			std::string getSerial() const;
			void provideSystemErrorMsg(HalResponse& resp);
			void provideSystemConnectMsg(bool connect);

			void addSystemNotifyCallback(const NotifyCallback& notifyCallback);

			bool hasCommunication();

			boost::mutex *getMutex ();

			std::vector<uint8_t> * getHwVersion();
			std::vector<uint8_t> * getSwVersion();

			uint16_t getFetCoreVersion();
			uint16_t getFetSafeCoreVersion();

			bool communication;

		private:
			IoChannel* channel;
			std::vector<uint8_t> fetSwVersion;
			std::vector<uint8_t> fetHwVersion;			
	
			uint16_t fetCoreVersion;
			uint16_t fetSafeCoreVersion;		

			uint8_t currentId;

			friend class FetControlThread;
			FetControlThread* reader;

			typedef std::map<uint32_t, HalResponseHandlerPtr> ResponseHandlerTable;
			ResponseHandlerTable responseHandlers;

			/* this mutex protects the reponseHandlers list, so iterators keep stable */
			mutex_type rhMutex;
			mutex_type sendMutex;

			NotifyCallback lNotifyCallback;

			std::map<uint8_t, bool> reservedIds;
		};

	};
};

#endif /* DLL430_FETCONTROL_H */
