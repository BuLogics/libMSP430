/*
 * ConfigManager.h
 *
 * Handles JTAG communication.
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
#ifndef DLL430_CONFIGMANAGER_H
#define DLL430_CONFIGMANAGER_H

#include <boost/function.hpp>

#include <DLL430/VersionInfo.h>
#include <DLL430/FileFunc.h>

#include <DLL430_SYMBOL.h>
namespace TI
{
	namespace DLL430
	{
		enum UPDATE_STATUS_MESSAGES {
			BL_INIT = 0, /**< Initializing Update Bootloader */
			BL_ERASE_INT_VECTORS = 1, /**< Erasing mapped interrupt vectors */
			BL_ERASE_FIRMWARE = 2, /**< Erasing firmware memory section */
			BL_PROGRAM_FIRMWARE = 3, /**< Program new firmware */
			BL_DATA_BLOCK_PROGRAMMED = 4, /**< One data block of the new firmware was successfully programmed */
			BL_EXIT = 5, /**< Exit Update Bootlader and reboot firmware */
			BL_UPDATE_DONE = 6, /**< Update was successfully finished */
			BL_UPDATE_ERROR = 7, /**< An error occured during firmware update */
			BL_WAIT_FOR_TIMEOUT = 8 /**< An error occured during firmware update */
		};

		/** \brief manage the target device and the connection between FET and target device */
		class DLL430_SYMBOL ConfigManager
		{
		public:
			/** \brief get the version of the FET BIOS
			 *
			 * \return the version information
			 */
			virtual VersionInfo getBiosVersion () const = 0;

			/** \brief set the VCC for the target device
			 *
			 * \param vcc the VCC value in mV
			 * \return true if the VCC was set successfully, else false
			 */
			virtual bool setDeviceVcc (uint16_t vcc) = 0;

			/** \brief get the VCC of the target device
			 *
			 * \return the VCC value in mV
			 */
			 virtual uint16_t getDeviceVcc () const = 0;

			/** \brief read out Biso core version and compare it against the internal image
			*
			 * \return true when core version is identical 
			 */
			virtual bool checkCoreVersion ()  = 0;

			/** \brief get the external VCC of the target device
			 *
			 * \return the external VCC in mV
			 */
			virtual uint16_t getExternalVcc () const = 0;

			/** \brief JTAG mode */
			enum jtagMode {
				JTAG_MODE_4WIRE = 0, /**< 4-wire JTAG directly */
				JTAG_MODE_SPYBIWIRE = 1, /**< SpyBiWire (2-wire JTAG) */
				JTAG_MODE_4AFTER2 = 2, /**< use 4-wire JTAG after entry sequence with SpyBiWire */
				JTAG_MODE_AUTOMATIC =3, /**< no interface given -> detect */
				JTAG_MODE_UNDEF =4, /**< jtag mode not defined */
			};

			/** \brief JTAG speed */
			enum JTAG_4WIRE_SPEED
			{
				JTAG_4WIRE_SPEED_8_MHZ = 2, 
				JTAG_4WIRE_SPEED_4_MHZ = 4,
				JTAG_4WIRE_SPEED_2_MHZ = 8,
				JTAG_4WIRE_SPEED_1_MHZ = 16,
				JTAG_4WIRE_SPEED_500_KHZ = 32,
				JTAG_4WIRE_SPEED_250_KHZ = 64	
			};
			/** \brief select a JTAG mode
			 *
			 * \param mode select the mode to use (default: JTAG_MODE_4WIRE)
			 */
			virtual void setJtagMode (jtagMode mode) = 0;
		     /** \brief select JTAG speed
			 *
			 * \param speed is the devicer for SPI moudule
			 */
			virtual bool setJtagSpeed(JTAG_4WIRE_SPEED speed) = 0;
		    /** \get Interface mode SBW2, SBW4 or JTAG
			 *
			 * \return selected JTAG protocol
			 */
			virtual jtagMode getInterfaceMode() const = 0;
			/** \brief start JTAG control (without sync)
			 *
			 * \return true if JTAG was started successfully, else false
			 */
			virtual uint16_t start() = 0;
			 /** \brief start JTAG control (without sync)
			 *
			 * \return true if JTAG was started successfully, else false
			 */
			virtual uint16_t start(const std::string& pwd, uint32_t deviceCode) = 0;
			
			/** \brief stop JTAG control
			 *
			 * \param pwd start with password
			 * \param deviceCode start with deviceCode
			 * \return true if JTAG was stopped successfully, else false
			 */
			virtual bool stop () = 0;

			/** \brief reset the target device
			 * 
			 * \param vcc turn VCC off and on
			 * \param pin pull the reset pin (in microSeconds)
			 * \return true if every step was successful, else false
			 */

			virtual bool reset (bool vcc, uint16_t pin = 0 ,uint16_t JtagId =0) = 0;

			typedef boost::function3<void, uint32_t, uint32_t, uint32_t> FetNotifyCallback;
			/** \brief perform firmwareupdate
			 *
			 * \param fname defines the TI-txt file to be used for update or NULL for internal image
			 * \param callback defines the callback for update messages or NULL for no messages
			 * \param clientHandle reference given by the caller instance, returned in callback
			 * \return true on success
			 */	
			virtual bool firmWareUpdate(const char *fname, FetNotifyCallback callback = 0) = 0;

			/** \brief set device password
			 *
			 * \param pwd is the password for the device in hex notation, beginning with "0x"
			 */
			virtual void setPassword(const std::string& pwd) = 0;

			/** \brief set device code
			 *
			 * \param deviceCode for devices that need more specific identification (L092)
			 */
			virtual bool setDeviceCode(uint32_t deviceCode = 0) = 0;

			/** \brief check if calibration before flash access is enabled
			 */
			virtual bool freqCalibrationEnabled() const = 0;

			virtual bool totalErase()  = 0;
		};
	};
};

#endif /* DLL430_CONFIGMANAGER_H */
