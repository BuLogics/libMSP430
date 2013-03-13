/*
 * ConfigManagerV3.cpp
 *
 * Functionality for configuring target device.
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

#include <DLL430/VersionInfo.h>

#include "FetHandleV3.h"
#include "ConfigManagerV3.h"
#include "DeviceDbManagerExt.h"
#include "DeviceInfo.h"
#include "HalExecCommand.h"
#include "FetControl.h"
#include "WatchdogControl.h"
#include "Record.h"

#include <boost/thread/thread.hpp>

#include <iostream>
#include <iomanip>
#include <math.h>

#include "../../Bios/include/UifHal.h"
#include "../../Bios/include/Core.h"
#include "../../Bios/include/ConfigureParameters.h"

using namespace TI::DLL430;
using namespace std;

ConfigManagerV3::ConfigManagerV3 (FetHandleV3* parent)
 : parent(parent)
 , vcc(0)
 , mode(ConfigManager::JTAG_MODE_4WIRE)
 , deviceCode(0)
 , freqCalibration(true)
{
	updateCmd.setTimeout(20000);
}

ConfigManagerV3::~ConfigManagerV3 ()
{
	FetControl * control=this->parent->getControl();
	
	// send reset command to FET
	if((control != NULL)&&(control->hasCommunication()))
	{
		this->stop();

		std::vector<uint8_t> data;
		data.push_back(0x03);
		data.push_back(0x92);
		data.push_back(0x00);
		data.push_back(0x00);
	}
}

bool ConfigManagerV3::init ()
{
	string tag;
	string value;

	// read configuration for JTAG speed EDT trace.... 
	JTAG_4WIRE_SPEED jtagSpeed = JTAG_4WIRE_SPEED_8_MHZ;
	ifstream DllV3Ini("MSP430DLL.INI");
	while (DllV3Ini && !DllV3Ini.eof())
	{
		DllV3Ini >> tag >> value; 
	
		if (tag == "JTAG_SPEED")
		{
			if (value == "JTAG_4WIRE_SPEED_8_MHZ")
			{
				jtagSpeed = JTAG_4WIRE_SPEED_8_MHZ;
			}
			if (value == "JTAG_4WIRE_SPEED_4_MHZ")
			{
				jtagSpeed = JTAG_4WIRE_SPEED_4_MHZ;
			}
			if (value == "JTAG_4WIRE_SPEED_2_MHZ")
			{
				jtagSpeed = JTAG_4WIRE_SPEED_2_MHZ;
			}
			if (value == "JTAG_4WIRE_SPEED_1_MHZ")
			{
				jtagSpeed = JTAG_4WIRE_SPEED_1_MHZ;
			}
			if (value == "JTAG_4WIRE_SPEED_500_KHZ")
			{
				jtagSpeed = JTAG_4WIRE_SPEED_500_KHZ;
			}
			if (value == "JTAG_4WIRE_SPEED_250_KHZ")
			{
				jtagSpeed = JTAG_4WIRE_SPEED_250_KHZ;
			}
		}

		if (tag == "DCO_CALIBRATION")
		{
			freqCalibration = (value != "OFF");
		}
	}
	this->setJtagSpeed(jtagSpeed);
	return true;
}

VersionInfo ConfigManagerV3::getBiosVersion() const
{
	std::vector<uint8_t> * sw_info=this->parent->getSwVersion();
	
	if (sw_info==NULL)
		return VersionInfo(0, 0, 0, 0);

	if (sw_info->size()<4)
		return VersionInfo(0, 0, 0, 0);

	unsigned char major=sw_info->at(1);
	return VersionInfo((((major&0xC0)>>6)+1),(major&0x3f),sw_info->at(0), 
		(sw_info->at(3)<<8)+sw_info->at(2));
}

bool ConfigManagerV3::checkCoreVersion()
{
	FetControl * control=this->parent->getControl();
	
	//get current core version from FET
	const uint16_t actualFetCoreVersion = control->getFetCoreVersion();
	uint16_t expectedFetCoreVersion = 0;
	
	Record fetCoreImage(coreImage, coreImage_address, coreImage_length_of_sections, coreImage_sections);
	//get core version from image (core version is stored in address 0xFDD8)
	if(fetCoreImage.getWordAtAdr(0xFDD8, &expectedFetCoreVersion))
	{
		//if core versions do not match, update core
		if(expectedFetCoreVersion != actualFetCoreVersion)
		{
			return false;
		}
	}
	return true;
}

void ConfigManagerV3::setJtagMode (enum ConfigManager::jtagMode mode)
{
	this->mode = mode;
}

uint16_t ConfigManagerV3::start()
{
	return this->start(password, deviceCode);
}

bool ConfigManagerV3::totalErase()
{
	if(this->start() != 0x1)
	{
		return false;
	}
	HalExecCommand cmd;
	//HalExecElement* el = new HalExecElement(ID_SetDeviceChainInfo);
	//el->appendInputData16(static_cast<uint16_t>(this->devHandle->getDevChainInfo()->getBusId()));
	//cmd.elements.push_back(el);
	HalExecElement* el = new HalExecElement(ID_SendJtagMailboxXv2);
	el->appendInputData16(0x11);
	el->appendInputData16(0xA55A);
	el->appendInputData16(0x1B1B);
	el->appendInputData16(0x1);
	cmd.elements.push_back(el);

	if (!this->parent->send(cmd))
	{
		return false;
	}

	/* assert hard RST/NMI and feed in magic pattern to stop device execution */
	/* thr RST/NMI will remove the register protection */ 
	if(!this->reset(false, 10, 0x99))
	{
		return false;
	}
	/* restart jtag connection and if needed feed in JTAG passowrd */ 
	if(this->start() != 0x1)
	{
		return false;
	}
	return true;
}

uint16_t AsciToHex(const char* password)
{
    uint16_t usTemp =0;
	
    for( int i = 3 ; i >= 0; i--)
    {
		char TempLetterToDig = password[i];
        TempLetterToDig = toupper(TempLetterToDig);
        if(TempLetterToDig >= 'A' && TempLetterToDig <= 'F')
        {
            usTemp |= (int)(TempLetterToDig - 7 - '0') << (12-(i*4));
        }
        else
        {
            usTemp |= (int)(TempLetterToDig - '0') << (12-(i*4));
        }      
    }
    return usTemp;
}

ConfigManager::jtagMode ConfigManagerV3::getInterfaceMode () const
{
	HalExecCommand cmd;
	HalExecElement* el = new HalExecElement(ID_GetInterfaceMode);

	el->setOutputSize(4);
	cmd.elements.push_back(el);

	if (!this->parent->send(cmd))
	{
		return JTAG_MODE_UNDEF;
	}
	uint16_t jtagID = el->getOutputAt16(0);
    uint16_t ifMode = el->getOutputAt16(2);
	if (jtagID == 0xFFFF)
	{
		return JTAG_MODE_UNDEF;

	}
	switch (ifMode)
	{
		case 0: // JTAG Mode
			return JTAG_MODE_4WIRE;
		case 1: // SBW2 Mode
			return JTAG_MODE_SPYBIWIRE;
		case 2: // SBW4 Mode
			return JTAG_MODE_4AFTER2;
		default:
			return JTAG_MODE_UNDEF;
	}
	return JTAG_MODE_UNDEF;
}

uint16_t ConfigManagerV3::start(const string& pwd, uint32_t deviceCode)
{
	uint8_t numOfDevices = 0;
	//#words in hex, ie. #characters / 4
	const uint16_t pwLength = (uint16_t)pwd.length() / 4;
	
	// if we have an L092 Rom device
	if (deviceCode == 0xDEADBABE)
	{
		if (pwLength > 4)
		{
			return 0;
		}

		HalExecElement* el = new HalExecElement(ID_UnlockC092);
		el->appendInputData16(pwLength);

		const char* pwdinternal = pwd.c_str();
		for (uint16_t i = 0; i < pwLength; i++)
		{
			uint16_t hexWord = AsciToHex(pwdinternal);		
			el->appendInputData16(hexWord);
			pwdinternal += 4;      
		} 

		el->setOutputSize(1);

		HalExecCommand cmd;
		cmd.elements.push_back(el);

		if (!this->parent->send(cmd))
		{
			return 0;
		}
		numOfDevices = 1;
		return numOfDevices;
	}
	// if we have an L092 device
	if(deviceCode == 0xA55AA55A|| deviceCode == 0x5AA55AA5) 
	{
		HalExecElement* el = new HalExecElement(ID_StartJtagActivationCode);		

		el->appendInputData8(0);
		el->appendInputData8(0);
		el->appendInputData32(deviceCode);
		
		el->setOutputSize(1);

		HalExecCommand cmd;
		cmd.elements.push_back(el);

		if (!this->parent->send(cmd))
		{
			return 0;
		}
		numOfDevices = 1;
		return numOfDevices;
	}
	// if we have a device locked with a custom password
	if ( !pwd.empty() )
	{
		if (pwLength > 60)
		{
			return 0;
		}

		HalExecElement* el = new HalExecElement(ID_UnlockDeviceXv2);
		switch (this->mode) 
		{
			case ConfigManager::JTAG_MODE_4WIRE:
				el->appendInputData16(0);
				break;
			case ConfigManager::JTAG_MODE_SPYBIWIRE:
				el->appendInputData16(1);
				break;
			case ConfigManager::JTAG_MODE_4AFTER2:	
				el->appendInputData16(2);
				break;
			default:
				delete el;
				return 0;
		}

		el->appendInputData16(pwLength);	

		const char* pwdinternal = pwd.c_str();
		for (uint16_t i = 0; i < pwLength; i++)
		{
			const uint16_t hexWord = AsciToHex(pwdinternal);		
			el->appendInputData16(hexWord);
			pwdinternal += 4;      
		}
		
		el->setOutputSize(2);

		HalExecCommand cmd;
		cmd.elements.push_back(el);

		if (!this->parent->send(cmd))
		{
			return 0;
		}
		#ifndef NDEBUG
			printf("Unlock device\n");
		#endif

		numOfDevices = 1;
		return numOfDevices;
	}
	// if we have a "normal" msp430 device without special handling
 	if((deviceCode != 0xA55AA55A && deviceCode != 0x5AA55AA5) || deviceCode == 0x80058005)
	{
		HalExecCommand startJtag;
		HalExecElement* e2 = new HalExecElement(ID_StartJtag);
		switch (this->mode) {
		case ConfigManager::JTAG_MODE_4WIRE:
			e2->appendInputData8(0);
			break;
		case ConfigManager::JTAG_MODE_SPYBIWIRE:
			e2->appendInputData8(1);
			break;
		case ConfigManager::JTAG_MODE_4AFTER2:	
			e2->appendInputData8(2);
			break;
		default:
			delete e2;
			return 0;
		}
		e2->setOutputSize(1);
		startJtag.elements.push_back(e2);
		if (!this->parent->send(startJtag))
		{
			return 0;
		}
		numOfDevices = startJtag.elements.at(0).getOutputAt8(0);
	#ifndef NDEBUG
		printf("num of devices %i\n",numOfDevices);
	#endif
		return numOfDevices;
	}
	return 0;
}

bool ConfigManagerV3::setJtagSpeed(JTAG_4WIRE_SPEED speed)
{
	HalExecElement* el = new HalExecElement(ID_Configure);
	el->appendInputData32(CONFIG_PARAM_JTAG_SPEED);
	el->appendInputData32(speed);
	el->setOutputSize(0);

	HalExecCommand configCmd;		
	configCmd.elements.push_back(el);
	return !this->parent->send(configCmd);
}

bool ConfigManagerV3::stop ()
{
	HalExecCommand stopJtag;
	stopJtag.elements.push_back(new HalExecElement(ID_StopJtag));
	return this->parent->send(stopJtag);
}

bool ConfigManagerV3::reset (bool vcc, uint16_t pin, uint16_t JtagId)
{
	//Bit definition:
 //         Bit 0 - TMS
 //         Bit 1 - TDI
 //         Bit 2 - TDO (writing this bit has no effect since its an input)
 //         Bit 3 - TCK
 //         Bit 4 - ------
 //         Bit 5 - SELT#
 //         Bit 6 - TGTRST
 //         Bit 7 - ------
 //         Bit 8 - ------
 //         Bit 9 - ------
 //         Bit 10- ------
 //         Bit 11- ------
 //         Bit 12- ------
 //         Bit 13- ------
 //         Bit 14- ------
 //         Bit 15- ------
	/* For the CPUxV2 (5xx and higher) we use the JTAG mailbox, that no user code is executed after a RST/NMI or VCC RST */
	if (JtagId != 0x89)
	{
		if (pin != 0) 
		{
			HalExecCommand cmd;
			HalExecElement* el = new HalExecElement(ID_ResetXv2);
			cmd.elements.push_back(el);
			if (!this->parent->send(cmd))
			{
				return false;
			}
		} 
		if (vcc) 
		{
			uint16_t voltage = this->getDeviceVcc();
			if (!this->setDeviceVcc(0))
			{
				return false;
			}
			// keep voltate 0 for minmum 5 seconds 
			boost::this_thread::sleep(boost::get_system_time() + boost::posix_time::seconds(5));

			if (!this->setDeviceVcc(voltage))
			{
				return false;
			}
			this->start();

			HalExecCommand cmd;
			HalExecElement* el = new HalExecElement(ID_ResetXv2);
			cmd.elements.push_back(el);
			if (!this->parent->send(cmd))
			{
				return false;
			}
		}
		return true;
	}
	else
	{
		/* for all other CPU architectures we just toggle the RST pint to create a BOR */
		if (pin != 0) 
		{
			HalExecCommand cmd;
			HalExecElement* el = new HalExecElement(ID_BitSequence);
			el->appendInputData8(2);
			el->appendInputData16(0);      //Both pins zero
			el->appendInputData16(3 << 5); //SELT# and RST pin
			el->appendInputData16(pin);
			el->appendInputData16(1 << 6); //RST pin
			el->appendInputData16(3 << 5); //SELT# and RST pin
			el->appendInputData16(0);
			cmd.elements.push_back(el);
			if (!this->parent->send(cmd))
			{
				return false;
			}
		}
		if (vcc) 
		{
			uint16_t voltage = this->getDeviceVcc();
			if (!this->setDeviceVcc(0))
			{
				return false;
			}
			// keep voltate 0 for minmum 5 seconds 
			boost::this_thread::sleep(boost::get_system_time() + boost::posix_time::seconds(5));
			if (!this->setDeviceVcc(voltage))
			{
				return false;
			}
		}
		return true;
	}
}

bool ConfigManagerV3::setDeviceVcc (uint16_t vcc)
{
	HalExecCommand cmd;
#ifndef NDEBUG
	printf("VCC  in[mV]: %i\n",vcc);
#endif
	HalExecElement* el = new HalExecElement(ID_SetVcc);
	el->appendInputData16(vcc);
	el->setInputMinSize(2);
	cmd.elements.push_back(el);
	if (!this->parent->send(cmd))
	{
		return false;
	}
	this->vcc = vcc;
#ifndef NDEBUG
	printf("VCC out[mV]: %i\n",this->getDeviceVcc ());
#endif
	if(vcc)
	{
		boost::this_thread::sleep(boost::get_system_time() + boost::posix_time::milliseconds(1000));
	}
#ifndef NDEBUG
	printf("VCC out[mV]: %i\n",this->getDeviceVcc ());
#endif
	return true;
}

uint16_t ConfigManagerV3::getDeviceVcc () const
{
	HalExecCommand cmd;
	HalExecElement* el = new HalExecElement(ID_GetVcc);
	el->setOutputSize(4);
	cmd.elements.push_back(el);
	if (!this->parent->send(cmd))
		return 0;

	return el->getOutputAt16(0);
}

uint16_t ConfigManagerV3::getExternalVcc () const
{
	HalExecCommand cmd;
	HalExecElement* el = new HalExecElement(ID_GetVcc);
	el->setOutputSize(4);
	cmd.elements.push_back(el);
	if (!this->parent->send(cmd))
		return 0;

	return el->getOutputAt16(2);
}


bool ConfigManagerV3::firmWareUpdate(const char *fname, FetNotifyCallback callback)
{
	FetControl * control=this->parent->getControl();
	
	if (control == NULL)
		return false;

	std::string serial = control->getSerial();

	const uint32_t biosVersion = getBiosVersion().get();

	// core/HAL communication has changed with version 3.2.0.8
	// and after HAL update the core/HAL communication would not work
	// do not update as long as there is no direct core update 
	if ((biosVersion > 30200000) && (biosVersion < 30200008) && (fname == NULL))
	{
		printf("This version need a core update\n");
		printf("Please check release documentation\n");
		return false;
	}
	
	bool doCoreUpdate=false;
	
	//get current core version from FET
	const uint16_t actualFetCoreVersion = control->getFetCoreVersion();
	uint16_t expectedFetCoreVersion = 0;
	
	Record fetCoreImage(coreImage, coreImage_address, coreImage_length_of_sections, coreImage_sections);
	//get core version from image (core version is stored in address 0xFDD8)
	if (fetCoreImage.getWordAtAdr(0xFDD8, &expectedFetCoreVersion))
	{
		//if core versions do not match, update core
		if (expectedFetCoreVersion != actualFetCoreVersion)
		{
			doCoreUpdate=true;
		}
	}

	// core will be updated through internal image
	if (doCoreUpdate)
	{
		if (callback)
		{
			callback(BL_INIT,0,0);
		}
		if (!this->upInit(1))
		{
			return false;
		}
		if (callback)
		{
			callback(BL_ERASE_FIRMWARE,0,0);
		}
		if (!this->upCoreErase())
		{
			return false;
		}
		if (callback)
		{
			callback(BL_PROGRAM_FIRMWARE,0,0);
		}
		if (!this->upCoreWrite())
		{
			return false;
		}
		if (!this->upCoreRead())
		{
			return false;
		}
		if (callback)
		{
			callback(BL_EXIT,0,0);
		}
		
		// create 0x55 command erase signature
		// command forces safecore to search for new core on reset
		std::vector<uint8_t> data_55;
		data_55.push_back(0x03);
		data_55.push_back(0x55);
		uint8_t id=control->createResponseId();
		data_55.push_back(id);
		data_55.push_back(0x00);

		control->sendData(data_55);
		control->clearResponse();

		boost::this_thread::sleep(boost::get_system_time() + boost::posix_time::seconds(8));
	}

	FileFuncImpl firmware;

	if (fname)
	{
		//Do not allow using a file if there is no valid HAL on the UIF
		if ((biosVersion < 20000000) || !firmware.readFirmware(fname))
		{
			return false;
		}
	}
	else
	{
		firmware.readFirmware(halImage, halImage_address, halImage_length_of_sections, halImage_sections);
	}

	if ((firmware.getNumberOfSegments())==0)
		return false;

	// start HAL update routine
	if (callback)
	{
		callback(BL_INIT,0,0);
	}
	if (!this->upInit(1))
	{
		return false;
	}
	if (callback)
	{
		callback(BL_ERASE_FIRMWARE,0,0);
	}
	if (!this->upErase(firmware))
	{
		return false;
	}
	if (callback)
	{
		callback(BL_PROGRAM_FIRMWARE,0,0);
	}
	if (!this->upWrite(firmware, callback))
	{
		return false;
	}
	if (callback)
	{
		callback(BL_EXIT,0,0);
	}
	if (!this->upInit(0))
	{
		return false;
	}

	parent->getControl()->resetCommunication();
	parent->getControl()->setObjectDbEntry(0);

	// activate the new HAL (in case of downgrade: change to VCP)
	HalExecCommand initCmd;
	HalExecElement* el = new HalExecElement(ID_Init);
	initCmd.elements.push_back(el);
	
	const bool initFailed = !this->parent->send(initCmd);

	// give the firmware time to execute initialisation
	boost::this_thread::sleep(boost::get_system_time() + boost::posix_time::seconds(1));

	//Only an error if no user specified file is used (ie. init will fail for a downgrade)
	if (initFailed && !fname)
	{
		return false;
	}

	if (!initFailed)
	{
		//Perform a reset to make sure everything is correctly initialized
		HalExecElement* el = new HalExecElement(ID_Zero);
		el->appendInputData8(STREAM_CORE_ZERO_PUC_RESET);

		HalExecCommand cmd;
		cmd.elements.push_back(el);

		parent->send(cmd);
		boost::this_thread::sleep(boost::get_system_time() + boost::posix_time::seconds(2));

		if(callback)
			callback(BL_UPDATE_DONE,0,0);
	}

	return true;
}

bool ConfigManagerV3::upInit(unsigned char level)
{
	HalExecElement* el = new HalExecElement(ID_Zero, UpInit);
	el->setAddrFlag(false);
	el->appendInputData8(level);

	HalExecCommand cmd;
	cmd.elements.push_back(el);

	return this->parent->send(cmd);
}

bool ConfigManagerV3::upErase(const FileFuncImpl& firmware)
{
	for (size_t i = 0; i < firmware.getNumberOfSegments(); ++i)
	{
		const DownloadSegment *seg = firmware.getFirmwareSeg(i);
		if (seg == NULL)
			return false;

		HalExecElement* el = new HalExecElement(ID_Zero, UpErase);
		el->setAddrFlag(false);
		el->appendInputData32(seg->startAddress&0xfffffffe);
		el->appendInputData32(seg->size);

		updateCmd.elements.clear();
		updateCmd.elements.push_back(el);
		if (!this->parent->send(updateCmd))
		{
			return false;
		}
	}
	return true;
}

bool ConfigManagerV3::upWrite(const FileFuncImpl& firmware, FetNotifyCallback callback)
{
	uint32_t percent = 100/(uint32_t)firmware.getNumberOfSegments();

	for (size_t i = 0; i < firmware.getNumberOfSegments(); ++i)
	{
		if (callback)
			callback(BL_DATA_BLOCK_PROGRAMMED, (uint32_t)i*percent, 0);

		const DownloadSegment *seg = firmware.getFirmwareSeg(i);
	
		if (seg == NULL)
			return false;

		// create Core telegram -> update write 
		HalExecElement* el = new HalExecElement(ID_Zero, UpWrite);
		// no HAL id needed for update
		el->setAddrFlag(false);

		const uint32_t padding = seg->size % 2;
		const uint32_t data2send = seg->size + padding;

		// add address data
		el->appendInputData32(seg->startAddress&0xfffffffe);
		el->appendInputData32(data2send);

		// add update data
		for (uint32_t n=0; n < seg->size; n++)
			el->appendInputData8(seg->data[n] & 0xff);

		for (uint32_t p=0 ; p < padding; p++)
			el->appendInputData8(0xff);

		updateCmd.elements.clear();		
		updateCmd.elements.push_back(el);
		if (!this->parent->send(updateCmd))
		{
			return false;
		}
	}
	if(callback)
		callback(BL_DATA_BLOCK_PROGRAMMED,100,0);

	return true;
}


bool ConfigManagerV3::upRead(const FileFuncImpl& firmware)
{
	for (size_t i = 0; i < firmware.getNumberOfSegments(); ++i)
	{
		const DownloadSegment *seg = firmware.getFirmwareSeg(i);
		if ( seg == NULL )
			return false;

		const uint32_t padding = seg->size % 2;
		const uint32_t data2receive = seg->size + padding;

		updateCmd.elements.clear();

		HalExecElement* el = new HalExecElement(ID_Zero, UpRead);
		// no HAL id needed for update
		el->setAddrFlag(false);	
		el->appendInputData32(seg->startAddress&0xfffffffe);
		el->appendInputData32(data2receive);

		updateCmd.elements.push_back(el);
		if (!this->parent->send(updateCmd))
			return false;

		for (size_t j = 0; j < seg->size; ++j)
		{
			if ( el->getOutputAt8(j) != seg->data[j] )
				return false;
		}
	}
	return true;
}

bool ConfigManagerV3::upCoreErase()
{
	//create erase command
	updateCmd.elements.clear();
	HalExecElement* el = new HalExecElement(ID_Zero, UpErase);
	el->setAddrFlag(false);
	//append start address and length in bytes
	el->appendInputData32(0x2500);
	el->appendInputData32(0xbb00);

	updateCmd.elements.push_back(el);
	return this->parent->send(updateCmd);
}

bool ConfigManagerV3::upCoreWrite()
{
	Record fetCoreImage(coreImage, coreImage_address, coreImage_length_of_sections, coreImage_sections);

	updateCmd.elements.clear();
	HalExecElement* el = new HalExecElement(ID_Zero, UpWrite);
	el->setAddrFlag(false);
	//append flash start-address where following data will be flashed to
	el->appendInputData32(0x2500);
	//append number of data bytes, including number of sections ,section start addresses and core signature
	el->appendInputData32((fetCoreImage.getNumOfAllDataWords() + fetCoreImage.getNumOfManageWords(true))*2);
	//append core signature
	el->appendInputData32(Record::coreSignature);
	//append number of sections
	el->appendInputData16( (uint16_t) fetCoreImage.getSectCount());

	while(fetCoreImage.hasNextSect())
	{
		//append start address and length information of each section
		el->appendInputData16( (uint16_t) fetCoreImage.getSectStartAdr());
		el->appendInputData16( (uint16_t) fetCoreImage.getSectLength());

		while(fetCoreImage.sectHasNextWord())
		{
			el->appendInputData16(fetCoreImage.getNextWord());
		}
		fetCoreImage.nextSection();
	}
	updateCmd.elements.push_back(el);
	return this->parent->send(updateCmd);
}

//reads back flashed core data and compares it to data in core image
//returns: true, if data on FET and core image are equal 
bool ConfigManagerV3::upCoreRead()
{
	Record fetCoreImage(coreImage, coreImage_address, coreImage_length_of_sections, coreImage_sections);

	updateCmd.elements.clear();
	HalExecElement* el = new HalExecElement(ID_Zero, UpRead);
	el->setAddrFlag(false);
	//append start address for read access
	el->appendInputData32(0x2500);
	//append number of data bytes, including number of sections ,section start addresses and core signature
	el->appendInputData32((fetCoreImage.getNumOfAllDataWords() + fetCoreImage.getNumOfManageWords(true))*2);
	updateCmd.elements.push_back(el);
	
	if (!this->parent->send(updateCmd))
	{
		return false;
	}
	//compare core signature
	if (el->getOutputAt32(0) != Record::coreSignature)
	{
		return false;
	}
	//compare number of sections
	if(el->getOutputAt16(4) != fetCoreImage.getSectCount())
	{
		return false;
	}
	uint32_t overhead = 6;

	//compare data of current section, including start adress and length
	while(fetCoreImage.hasNextSect())
	{
		if(el->getOutputAt16(fetCoreImage.getCurrentPosByte()-2 + overhead) != fetCoreImage.getSectStartAdr())
		{
			return false;
		}
		overhead += 2;
		if(el->getOutputAt16(fetCoreImage.getCurrentPosByte()-2 + overhead) != fetCoreImage.getSectLength())
		{
			return false;
		}
		overhead += 2;

		while(fetCoreImage.sectHasNextWord())
		{
			if(el->getOutputAt16(fetCoreImage.getCurrentPosByte()-2 + overhead) != fetCoreImage.getNextWord())
			{
				return false;
			}
		}
		fetCoreImage.nextSection();
	}
	
	return true;
}

void ConfigManagerV3::setPassword(const string& pwd)
{
	this->password = pwd;
}

bool ConfigManagerV3::setDeviceCode(uint32_t deviceCode)
{
	 this->deviceCode = deviceCode;
	 return true;
}

bool ConfigManagerV3::freqCalibrationEnabled() const
{
	return freqCalibration;
}
