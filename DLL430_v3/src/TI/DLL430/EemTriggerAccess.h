/*
 * EemTriggerAccess.h
 *
 * Base class for EEM-Triggers.
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
#ifndef DLL430_EEMTRIGGERACCESS_H
#define DLL430_EEMTRIGGERACCESS_H

#include <inttypes.h>

namespace TI
{
	namespace DLL430
	{

		enum TriggerType
		{
			MEMBUS_ADDRESS = 0, /* MAB */
			MEMBUS_DATA    = 1, /* MDB */
			REGISTER	   = 2	/* Register */
		};

		enum Register
		{
			REG_PC       = 0x0,
			REG_R1       = 0x1,
			REG_SR       = 0x2,
			REG_SR_FBITS = 0x3,
			REG_R4       = 0x4,
			REG_R5       = 0x5,
			REG_R6       = 0x6,
			REG_R7       = 0x7,
			REG_R8       = 0x8,
			REG_R9       = 0x9,
			REG_R10      = 0xA,
			REG_R11      = 0xB,
			REG_R12      = 0xC,
			REG_R13      = 0xD,
			REG_R14      = 0xE,
			REG_R15      = 0xF
		};


		/* Some devices can only use EQ,
		 * others can only use EQ and NE,
		 * and there are devices that can handle all modes.
		 */
		enum CompareMode
		{
			COMPARE_EQ = 0, /* equal */
			COMPARE_GE = 1, /* greater than or equal */			
			COMPARE_LE = 2, /* less than or equal */
			COMPARE_NE = 3  /* not equal */
		};

		enum TriggerCondition
		{
			/* These 4 conditions can be used by all devices */
			COND_FETCH            = 0x0,
			COND_FETCH_HOLD       = 0x1,
			COND_NOFETCH          = 0x2,
			COND_DONT_CARE        = 0x3,

			/* some devices cannot use the following 12 conditions */
			COND_FETCH_READ       = 0x4,
			COND_FETCH_WRITE      = 0x5,
			COND_READ             = 0x6,
			COND_WRITE            = 0x7,

			/* some devices cannot use the following 8 conditions
			 * but the 4 conditions above
			 */
			COND_FETCH_NODMA      = 0x8,
			COND_DMA              = 0x9,
			COND_NODMA            = 0xA,
			COND_WRITE_NODMA      = 0xB,
			COND_FETCH_READ_NODMA = 0xC,
			COND_READ_NODMA       = 0xD,
			COND_READ_DMA         = 0xE,
			COND_WRITE_DMA        = 0xF
		};

		class EemTriggerAccess
		{
		public:
			EemTriggerAccess ();
			~EemTriggerAccess ();
	
			// init to MAB - trigger
			void init(uint8_t id);					// private/friend???
			void reset();

			// set restrictions to trigger type
			bool setType (TriggerType);
			// set trigger type independent values
			void setValue (uint32_t);
			void setControl (uint32_t);
			void setDefaultCombination();
			void setCombination(uint32_t);
			void setCompare (CompareMode);
			bool setMask (uint32_t);
			// set trigger type dependent values
			void setCondition(TriggerCondition);	// only bus
			void setRegister(Register);				// only register
			void enableHold (bool);					// only register
			// get register values
			uint32_t getValue ();
			uint32_t getControl ();					// ??? only sub-bitmaps?
			uint32_t getMask ();
			uint32_t getCombination ();
			TriggerType getTriggerType();

			// get register-id (0-15)
			uint8_t getId();
			// get value addresses
			uint32_t getValueAdr() const;
			uint32_t getControlAdr();
			uint32_t getMaskAdr();
			uint32_t getCombinationAdr();

		private:
			uint8_t base;
			uint32_t mask;
			uint32_t value;
			uint32_t control;
			uint32_t combination;

			TriggerType type;
		};
	};
};

#endif /* DLL430_EEMTRIGGERACCESS_H */
