/*
 * TemplateTypes.h
 *
 * General templates to force type safety.
 *
 * Copyright (C) 2011 Texas Instruments Incorporated - http://www.ti.com/ 
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

#ifndef TEMPLATE_DEVICE_DB_TEMPLATETYPES_H
#define TEMPLATE_DEVICE_DB_TEMPLATETYPES_H

#if _MSC_VER > 1000
#pragma once
#endif

#include "Utility.h"

//default types to default file
//all base files serve as static check opportunity, when template parameters are given - see BOOST_STATIC_ASSERT.
namespace TI { namespace DLL430 { namespace TemplateDeviceDb {
	template<class T>
	inline int toInt(const T& t)
	{
		return static_cast<int>(t);
	}

	template<uint32_t N> 
	struct SingleUint32
	{
		const uint32_t value;
		SingleUint32() : value(N) {}
	};

	template<uint8_t N> 
	struct SingleUint8
	{
		const uint8_t value;
		SingleUint8() : value(N) {}
	};

	template<size_t N> 
	struct SingleSize_t
	{
		const size_t value;
		SingleSize_t() : value(N) {}
	};

	template<bool b> 
	struct SingleBool
	{
		const bool value;
		SingleBool() : value(b) {}
	};

	class BitsBase{};
	template<uint32_t bits> 
	struct Bits : public SingleUint32<bits>, BitsBase {};

	class DefaultBitsBase : BitsBase {};
	template<uint32_t bits> 
	struct DefaultBits : public SingleUint32<bits>, DefaultBitsBase {};

	//MSP430 device wide defaults set for memory if BitsDeviceDefaultType is chosen
	typedef DefaultBits<8>		DefaultBits8Type;
	typedef DefaultBits<16>		DefaultBits16Type;
	typedef DefaultBits<20>		DefaultBits20Type;

	class ObjectIdBase{};
	template<size_t size> 
	struct ObjectId : public SingleSize_t<size>, ObjectIdBase {};
} //namespace TemplateDeviceDb
}//namespace DLL430
}//namespace TI
#endif //TEMPLATE_DEVICE_DB_TEMPLATETYPES_H
