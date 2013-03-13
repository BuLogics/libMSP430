/*
 * FetHandleV3.cpp
 *
 * Communication with FET.
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
#ifndef DLL430_FETHANDLEV3_H
#define DLL430_FETHANDLEV3_H

#include <DLL430/FetHandle.h>
#include <DLL430/VersionInfo.h>
#include <boost/thread/mutex.hpp>

namespace TI
{
	namespace DLL430
	{
		class PortInfo;
		class FetControl;
		class IoChannel;
		class DeviceInfo;
		class HalExecCommand;
		class ConfigManagerV3;
		class MemoryManagerV3;
		class DebugManagerV3;
		class DeviceHandleManagerV3;

		class FetHandleV3 : public FetHandle
		{
		public:
			static const char* id;
		
			FetHandleV3 (PortInfo *);
			~FetHandleV3 ();

			/* public API */			
			const VersionInfo& getDllVersion () const { return this->version; };
			ConfigManager* getConfigManager ();
			DeviceHandleManager* getDeviceHandleManager (); 

			/* internal API */
			FetControl* getControl ();
			void configure (const DeviceInfo*);

			bool send (HalExecCommand &command);
	
			bool kill(uint8_t respId);
			bool pauseLoopCmd(unsigned long respId);
			bool resumeLoopCmd(unsigned long respId);

			bool hasCommunication();

			void addSystemNotifyCallback(const NotifyCallback& notifyCallback);

			void shutdown();

			bool sendHilCommand(HIL_COMMAND command, uint32_t data = 0);
			uint64_t sendJtagShift(HIL_COMMAND shiftType, uint64_t data, long bitSize = 16);
			bool setJtagPin(JTAG_PIN pin, bool state);

			std::vector<uint8_t> * getHwVersion();
			std::vector<uint8_t> * getSwVersion();

		private:
			VersionInfo version;
			IoChannel* channel;
			FetControl* control;

			ConfigManagerV3* configManager;
			DeviceHandleManagerV3* deviceHandleManager;

			typedef boost::mutex mutex_type;
			mutex_type* myMutex;

			bool communication;
		};
	};
};

#endif /* DLL430_FETHANDLEV3_H */
