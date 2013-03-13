/*
 * DeviceDbManagerExt.cpp
 *
 * Gives access to TemplateDeviceDb.
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

#include "DeviceDbManagerExt.h"

#include "DeviceInfo.h"
#include "TemplateDeviceDb/Registration.h"
#include "TemplateDeviceDb/Utility.h"

using namespace TI::DLL430;
using namespace TemplateDeviceDb;

TemplateDeviceDbManagerExt::TemplateDeviceDbManagerExt ()
{

}

TemplateDeviceDbManagerExt::~TemplateDeviceDbManagerExt ()
{
}

DeviceInfoPtr TemplateDeviceDbManagerExt::queryDb (size_t id) const
{
	DeviceInfoPtr info = TemplateDeviceDb::Registration().GetDeviceInfo(id);	
	return info;
}

//converts MSP430_device_idCode to IdCodeImpl
//creates a match
//prepares Registration to point to according device when queryDb (size_t id) is called
size_t TemplateDeviceDbManagerExt::queryDb (const MSP430_device_idCode& idCode) const
{
	using TemplateDeviceDb::Registration;
	using TemplateDeviceDb::IdCodeImpl;

	return Registration().FindAndPrepareDevice(
		IdCodeImpl(	
			idCode.verId, idCode.verSubId, 
			idCode.revisison, idCode.fab, 
			idCode.self, idCode.config, idCode.fuses,
			idCode.activationKey
		)
	);
}

const char* TemplateDeviceDbManagerExt::getEntryName (size_t id) const
{
	if (id > getMaxId())
		return NULL;

	return TemplateDeviceDb::Registration().GetCurrentDeviceDescription();
}

size_t TemplateDeviceDbManagerExt::getMaxId () const
{
	return TemplateDeviceDb::Registration().GetDatabaseSize() - 1;
}

bool TemplateDeviceDbManagerExt::hasTestVpp (size_t) const
{
	return TemplateDeviceDb::Registration().HasCurrentDeviceVpp();
}
