/*
 * ClockCalibration.h
 *
 * Handles setting of correct clock frequency for flash accesses.
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
#ifndef DLL430_CLOCKCALIBRATION_H
#define DLL430_CLOCKCALIBRATION_H

#include <stdint.h>

namespace TI
{
	namespace DLL430
	{
		class MemoryManager;
		class ConfigManager;
		class DeviceHandleV3;
		class DeviceInfo;

		class ClockCalibration
		{
		public:
			static ClockCalibration* create(DeviceHandleV3* devHandle, MemoryManager* mm, 
											const ConfigManager* configMan, const DeviceInfo& info);

			virtual bool determineSettings() = 0;
			virtual bool backupSettings() = 0;
			virtual bool makeSettings() const = 0;
			virtual bool restoreSettings() = 0;

			virtual uint16_t getCal0() const = 0;
			virtual uint16_t getCal1() const = 0;
		};

		class ClockCalibrationDCO : public ClockCalibration
		{
		public:
			ClockCalibrationDCO(DeviceHandleV3* devHandle, MemoryManager* mm, uint32_t maxBCS);

			virtual bool determineSettings();
			virtual bool backupSettings();
			virtual bool makeSettings() const;
			virtual bool restoreSettings();

			virtual uint16_t getCal0() const;
			virtual uint16_t getCal1() const;

		private:
			DeviceHandleV3* devHandle_;
			MemoryManager* mm_;
			uint32_t maxBCS_;
			uint32_t backupDCO_;
			uint32_t backupBCS1_;
			uint32_t backupBCS2_;
			uint16_t calibDCO_;
			uint16_t calibBCS1_;
			uint16_t calibBCS2_;
			bool isCalibrated_;
			bool madeBackup_;
		};

		class ClockCalibrationFLL : public ClockCalibration
		{
		public:
			ClockCalibrationFLL(DeviceHandleV3* devHandle, MemoryManager* mm);

			virtual bool determineSettings();
			virtual bool backupSettings();
			virtual bool makeSettings() const;
			virtual bool restoreSettings();

			virtual uint16_t getCal0() const;
			virtual uint16_t getCal1() const;

		private:
			DeviceHandleV3* devHandle_;
			MemoryManager* mm_;

			uint32_t backupSCFQCTL_;
			uint32_t backupSCFI0_;
			uint32_t backupSCFI1_;
			uint32_t backupFLLCTL0_;
			uint32_t backupFLLCTL1_;
			
			uint16_t calibSCFI0_;
			uint16_t calibSCFI1_;
			uint16_t calibSCFQCTL_;
			uint16_t calibFLLCTL0_;
			uint16_t calibFLLCTL1_;
			bool isCalibrated_;
			bool madeBackup_;
		};

		class ClockCalibrationNone : public ClockCalibration
		{
		public:
			virtual bool determineSettings() { return true; }
			virtual bool backupSettings() { return true; }
			virtual bool makeSettings() const { return true; }
			virtual bool restoreSettings() { return true; }

			virtual uint16_t getCal0() const { return 0; }
			virtual uint16_t getCal1() const { return 0; }
		};
		
	};
};

#endif /* DLL430_CLOCKCALIBRATION_H */
