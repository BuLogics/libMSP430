/*
 * DeviceHandleV3.h
 *
 * Communication with target device.
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
#ifndef DLL430_DeviceHandleV3_H
#define DLL430_DeviceHandleV3_H

#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/array.hpp>

#include <DLL430/DeviceHandle.h>
#include "MemoryManagerV3.h"
#include "DebugManagerV3.h"
#include "DeviceInfo.h"
#include "FileFuncImpl.h"
#include <boost/shared_ptr.hpp>
#include "WatchdogControl.h"
#include "FuncletCode.h"
#include <DLL430/DeviceChainInfo.h>

namespace TI
{
	namespace DLL430
	{
		class FetHandleV3;
		class DeviceInfo;
		class DeviceChainInfo;
		class FetControl;
		class HalCommand;
		class ClockCalibration;

		class DeviceHandleV3  : public DeviceHandle
		{
		public:
			DeviceHandleV3 (FetHandleV3*, DeviceChainInfo*);
			~DeviceHandleV3 ();

			EmulationManagerPtr getEmulationManager();
			MemoryManagerV3* getMemoryManager ();
			DebugManagerV3* getDebugManager ();
			FetHandleV3* getFetHandle () { return parent; }
			ClockCalibration* getClockCalibration() { return clockCalibration; }

			FileFunc* getFileRef();
			bool writeSegments();
		
			/// \brief Run the boot code writing the the specified command in the mailbox first
			/// \param command[in] Command to be put in the mailbox
			/// \return True if bootcode execution succeeded
			bool runBootCode(uint32_t command);
			long magicPatternSend();
			long identifyDevice (uint32_t activationKey);
			const std::string & getDescription();
			bool secure ();
			bool reset ();
			bool sendSyncCommand();

			bool verifySegments();

			FetControl* getControl ();

			bool send (HalExecCommand &command);

			DeviceChainInfo* getDevChainInfo ();

			void setWatchdogControl (boost::shared_ptr<WatchdogControl>);
			boost::shared_ptr<WatchdogControl> getWatchdogControl ();
			uint8_t getDeviceJtagId();
			uint8_t getJtagId();
			uint32_t getDeviceIdPtr();
			uint32_t getEemVersion();
			uint16_t getCoreIpId();
			uint32_t getIdDataAddr();
			bool isJtagFuseBlown();
			uint16_t getSubID(uint32_t info_len, uint32_t deviceIdPtr , uint32_t pc);

			typedef std::map<unsigned long, uint16_t> function_map_type;
			typedef std::map<FuncletCode::Type, FuncletCode> funclet_map_type;

			void setDeviceId (long id);

			uint32_t checkHalId(uint32_t base_id) const;
			uint32_t getBaseHalId(uint32_t mapped_id) const;

			const FuncletCode& getFunclet(FuncletCode::Type funclet);

			bool supportsQuickMemRead() const;
			uint16_t getMinFlashVcc() const;
			bool hasFram() const;
			ClockSystem getClockSystem() const;

		protected:

		private:
			uint8_t jtagId;
			uint32_t deviceIdPtr;
			uint32_t CoreIpId;
			uint32_t IdDataAddr;
			uint32_t eemVersion;

			boost::shared_ptr<WatchdogControl> wdt;
			function_map_type map;
			funclet_map_type funcletTable;
			enum DeviceHandle::jtagMode mode;
			
			FetHandleV3* parent;
			DeviceChainInfo* deviceChainInfo;
			EmulationManagerPtr emulationManager;
			MemoryManagerV3* memoryManager;
			DebugManagerV3* debugManager;
			FileFuncImpl* fileManager;
			ClockCalibration* clockCalibration;
			
			uint16_t minFlashVcc;
			bool hasTestVpp;
			bool quickMemRead;
			bool deviceHasFram;
			ClockSystem clockSystem;

			typedef boost::mutex mutex_type;
			mutex_type* myMutex;

			std::string description;

			typedef boost::array<uint8_t, DeviceInfo::nrUsedClockModules> EtwCodes;
			EtwCodes etwCodes;

			void configure (const DeviceInfo* info);
			long getDeviceIdentity(uint32_t activationKey, uint32_t* pc, uint32_t* sr);

			bool sendDeviceConfiguration(uint32_t parameter, uint32_t value);
		};

	};
};

#endif /* DLL430_DeviceHandle_H */
