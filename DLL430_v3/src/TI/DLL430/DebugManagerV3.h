/*
 * DebugManagerV3.h
 *
 * Functionality for debugging target device.
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
#ifndef DLL430_DEBUGMANAGERV3_H
#define DLL430_DEBUGMANAGERV3_H

#include <DLL430/DebugManager.h>
#include <boost/ptr_container/ptr_vector.hpp>
#include <boost/thread/thread.hpp>
#include <map>
#include "HalExecCommand.h"
#include "DeviceInfo.h"

#include "EEM/CycleCounter.h"

#include "MessageData.h"
#include "EventNotifier.h"


namespace TI
{
	namespace DLL430
	{
		class DeviceHandleV3;
		namespace EEM {
			class Trigger;
			class TriggerGroupImpl;
			class SequencerImpl;
		};


		class DebugManagerV3 : public DebugManager
		{
		public:
			DebugManagerV3 (DeviceHandleV3*, const DeviceInfo*);
			~DebugManagerV3 ();

			bool reconnectJTAG();

			/** free run */
			bool run (uint16_t controlMask, DebugEventTarget* = 0, bool releaseJTAG = false);

			bool clearEemResponse();

			/* sync Jtag */
			bool stop (bool jtagWasReleased = false);
			
			/* do a single step */
			bool singleStep (uint32_t* cycles = 0);

			uint8_t getClockControl() const;

			uint16_t getClockControlSetting() const;

			void setClockControlSetting(uint16_t clkCntrl);

			uint16_t getClockModuleDefaultSetting() const;

			uint16_t getClockModuleSetting() const;

			void setClockModuleSetting(uint16_t modules);

			char ** getModuleStrings(uint32_t * n) const;

			char ** getClockStrings(uint32_t * n) const;

			bool initEemRegister();

			bool eemWriteRegister(uint32_t, uint32_t);

			bool eemReadRegister(uint32_t, uint32_t* buffer);

			void setOpcode(uint16_t value);

			bool activatePolling(uint16_t mask);

			bool saveContext();

			uint8_t getRunEemResponseId();

			void runEvent(MessageDataPtr messageData);

			void localCallback (MessageDataPtr messageData, uint32_t clientHandle);

			void setLpmDebugging(bool enable);

			bool getLpmDebugging();

			bool activateJStatePolling(DebugEventTarget * cb);

			bool queryLpm5State();

			bool isDeviceInLpm5();

			bool wakeupDevice();

			void pausePolling();
			
			void resumePolling();

			bool syncDeviceAfterLPMx5();

			uint64_t getCycleCounterValue() { return cycleCounter_.getValue(); }
			
			void resetCycleCounterValue() { cycleCounter_.reset(); }

			bool startStoragePolling();
			
			bool stopStoragePolling();

		protected:

		private:
			DeviceHandleV3* parent;

			HalExecCommand waitForEem;
			HalExecCommand waitForJState;
			HalExecCommand waitForStorage;

			uint8_t clockControl;
			uint16_t genclkcntrl;
			uint16_t mclkcntrl0;
			uint16_t defaultMclkcntrl0;

			uint8_t emulationLevel;

			char** moduleStrings;
			uint32_t nModuleStrings;

			char** clockStrings;
			uint32_t nClockStrings;

			size_t lastWrite;

			uint16_t internalDebugState; 

			//helper funtion to fill data behind moduleStrings pointer in a heap based way
			void createModuleStrings(const DeviceInfo::ClockMapping& clockMapping);
			void createClockStrings(const DeviceInfo::ClockNames& clockNames);

			DebugEventTarget* cbx;
			uint8_t typex;

			uint16_t opcode;

			bool lpmDebuggingEnabled;

			bool deviceInLpm5;

			CycleCounter cycleCounter_;
			bool resetCycleCounterBeforeNextStep;
			bool storagePollingActive;

			EventNotifier<MessageDataPtr> eventNotifier;
		};

	};
};

#endif /* DLL430_DEBUGMANAGERV3_H */
