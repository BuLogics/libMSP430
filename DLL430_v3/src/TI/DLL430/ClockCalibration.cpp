/*
 * CpuMemoryAccess.cpp
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

#include "ClockCalibration.h"

#include <DLL430/MemoryManager.h>
#include <DLL430/ConfigManager.h>
#include "HalExecCommand.h"
#include "DeviceHandleV3.h"
#include "DeviceInfo.h"

using namespace TI::DLL430;


ClockCalibration* ClockCalibration::create(DeviceHandleV3* devHandle, MemoryManager* mm, 
											const ConfigManager* configMan, const DeviceInfo& info)
{
	if (configMan && configMan->freqCalibrationEnabled())
	{
		if (info.clockSystem() == BC_1xx)
			return new ClockCalibrationDCO(devHandle, mm, 0x7);

		if (info.clockSystem() == BC_2xx)
			return new ClockCalibrationDCO(devHandle, mm, 0xf);

		if (info.clockSystem() == FLLPLUS)
			return new ClockCalibrationFLL(devHandle, mm);
	}

	return new ClockCalibrationNone();
}


ClockCalibrationDCO::ClockCalibrationDCO(DeviceHandleV3* devHandle, MemoryManager* mm, uint32_t maxBCS)
	: devHandle_(devHandle)
	, mm_(mm)
	, maxBCS_(maxBCS)
	, backupDCO_(0)
	, backupBCS1_(0)
	, backupBCS2_(0)
	, calibDCO_(0x3) //Default values after PUC
	, calibBCS1_(maxBCS == 0xF ? 0x7 : 0x4)
	, calibBCS2_(0)
	, isCalibrated_(false)
	, madeBackup_(false)
{
}

bool ClockCalibrationDCO::determineSettings()
{
	if (!isCalibrated_)
	{
		MemoryArea* ram = mm_->getMemoryArea("system");
		if(ram==NULL)
		{
			return false;
		}

		HalExecElement* el = new HalExecElement( devHandle_->checkHalId(ID_GetDcoFrequency) );
		el->appendInputData16(ram->getStart());
		el->appendInputData16(maxBCS_);
		
		HalExecCommand cmd;
		cmd.elements.push_back(el);
		cmd.setTimeout(5000);

		//If calibration failed, we keep the existing default values
		if (devHandle_->send(cmd))
		{
			calibDCO_ = el->getOutputAt16(0);
			calibBCS1_ = el->getOutputAt16(2);
			calibBCS2_ = el->getOutputAt16(4);
		}
		isCalibrated_ = true;
	}
	return true;
}

bool ClockCalibrationDCO::backupSettings()
{
	madeBackup_ = false;

	if ( MemoryArea* peripheral = mm_->getMemoryArea("peripheral8bit") )
	{
		madeBackup_ = peripheral->read(0x56, &backupDCO_, 1) &&
					 peripheral->read(0x57, &backupBCS1_, 1) &&
					 peripheral->read(0x58, &backupBCS2_, 1) &&
					 peripheral->sync();
	}
	return madeBackup_;
}

bool ClockCalibrationDCO::makeSettings() const
{
	if ( MemoryArea* peripheral = mm_->getMemoryArea("peripheral8bit") )
	{
		//Set default values for BCSCTL1 (XT2OFF, RSEL 7), the funclet will set the actual value
		return peripheral->write(0x57, 0x87) &&
			   peripheral->write(0x58, calibBCS2_) &&
			   peripheral->sync();
	}
	return false;
}

bool ClockCalibrationDCO::restoreSettings()
{
	if (!madeBackup_)
		return true;

	madeBackup_ = false;

	if ( MemoryArea* peripheral = mm_->getMemoryArea("peripheral8bit") )
	{
		return peripheral->write(0x56, backupDCO_) &&
				peripheral->write(0x57, backupBCS1_) &&
				peripheral->write(0x58, backupBCS2_) &&
				peripheral->sync();
	}
	return false;
}

uint16_t ClockCalibrationDCO::getCal0() const
{
	return calibDCO_;
}

uint16_t ClockCalibrationDCO::getCal1() const
{
	return calibBCS1_;
}



ClockCalibrationFLL::ClockCalibrationFLL(DeviceHandleV3* devHandle, MemoryManager* mm)
	: devHandle_(devHandle)
	, mm_(mm)
	, backupSCFQCTL_(0)
	, backupSCFI0_(0)
	, backupSCFI1_(0)
	, backupFLLCTL0_(0)
	, backupFLLCTL1_(0)
	, calibSCFI0_(0x43) //Default values after PUC
	, calibSCFI1_(0x7)
	, calibSCFQCTL_(0x1f)
	, calibFLLCTL0_(0x3)
	, calibFLLCTL1_(0x20)
	, isCalibrated_(false)
	, madeBackup_(false)
{
}

bool ClockCalibrationFLL::determineSettings()
{
	if (!isCalibrated_)
	{
		MemoryArea* ram = mm_->getMemoryArea("system");
		if(ram==NULL)
		{
			return false;
		}

		HalExecElement* el = new HalExecElement( devHandle_->checkHalId(ID_GetDcoFrequency) );
		el->appendInputData16(ram->getStart());
		el->appendInputData16(0);
		
		HalExecCommand cmd;
		cmd.elements.push_back(el);
		cmd.setTimeout(5000);

		//If calibration failed, we keep the existing default values
		if (devHandle_->send(cmd))
		{
			calibSCFI0_ = el->getOutputAt16(0);
			calibSCFI1_ = el->getOutputAt16(2);
			calibSCFQCTL_ = el->getOutputAt16(4);
			calibFLLCTL0_ = el->getOutputAt16(6);
			calibFLLCTL1_ = el->getOutputAt16(8);
		}
		isCalibrated_ = true;
	}
	return true;
}

bool ClockCalibrationFLL::backupSettings()
{
	madeBackup_ = false;

	if ( MemoryArea* peripheral = mm_->getMemoryArea("peripheral8bit") )
	{
		madeBackup_ = peripheral->read(0x50, &backupSCFI0_, 1) &&
					  peripheral->read(0x51, &backupSCFI1_, 1) &&
					  peripheral->read(0x52, &backupSCFQCTL_, 1) &&
					  peripheral->read(0x53, &backupFLLCTL0_, 1) &&
					  peripheral->read(0x54, &backupFLLCTL1_, 1) &&
					  peripheral->sync();
	}
	return madeBackup_;
}

bool ClockCalibrationFLL::makeSettings() const
{
	if ( MemoryArea* peripheral = mm_->getMemoryArea("peripheral8bit") )
	{
		return peripheral->write(0x50, calibSCFI0_) &&
			   peripheral->write(0x51, calibSCFI1_) &&
			   peripheral->write(0x52, calibSCFQCTL_) &&
			   peripheral->write(0x53, calibFLLCTL0_) &&
			   peripheral->write(0x54, calibFLLCTL1_) &&
			   peripheral->sync();
	}
	return false;
}

bool ClockCalibrationFLL::restoreSettings()
{
	if (!madeBackup_)
		return true;

	madeBackup_ = false;

	if ( MemoryArea* peripheral = mm_->getMemoryArea("peripheral8bit") )
	{
		return peripheral->write(0x50, backupSCFI0_) &&
			   peripheral->write(0x51, backupSCFI1_) &&
			   peripheral->write(0x52, backupSCFQCTL_) &&
			   peripheral->write(0x53, backupFLLCTL0_) &&
			   peripheral->write(0x54, backupFLLCTL1_) &&
			   peripheral->sync();
	}
	return false;
}

uint16_t ClockCalibrationFLL::getCal0() const
{
	return 0;
}

uint16_t ClockCalibrationFLL::getCal1() const
{
	return calibSCFI1_;
}
