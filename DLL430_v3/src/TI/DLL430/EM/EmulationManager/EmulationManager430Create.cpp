/*
 * EmulationManager430Create.cpp
 *
 * Creators for different EEM modules for MSP430 
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


#include <boost/make_shared.hpp>

#include "EmulationManager430.h"

#include "../TriggerManager/TriggerManager430.h"
#include "../TriggerCondition/TriggerConditionManager430.h"
#include "../BreakpointManager/BreakpointManager430.h"
#include "../CycleCounter/CycleCounter430.h"
#include "../StateStorage430/StateStorage430.h"
#include "../Sequencer/Sequencer430.h"


using namespace TI::DLL430;

namespace {

	class EmNone : public EmulationManager430
	{
	public:
		static EmulationManagerPtr create()
		{
			boost::shared_ptr<EmNone> em = boost::make_shared<EmNone>();
			return em;
		}
	};


	class EmSmall : public EmulationManager430
	{
	public:
		static EmulationManagerPtr create()
		{
			boost::shared_ptr<EmSmall> em = boost::make_shared<EmSmall>();

			em->mTriggerManager = boost::make_shared<TriggerManager430>(2, 0, 2, 0);

			em->mTriggerConditionManager = boost::make_shared<TriggerConditionManager430>(em->mTriggerManager);	
			em->mBreakpointManager = boost::make_shared<BreakpointManager430>();
			return em;
		}
	};


	class EmMedium : public EmulationManager430
	{
	public:
		static EmulationManagerPtr create()
		{
			boost::shared_ptr<EmMedium> em = boost::make_shared<EmMedium>();
			
			em->mTriggerManager = boost::make_shared<TriggerManager430>(3, 0, 3, 0);
			em->mTriggerManager->setExtendedComparisons();

			em->mTriggerConditionManager = boost::make_shared<TriggerConditionManager430>(em->mTriggerManager);	
			em->mBreakpointManager = boost::make_shared<BreakpointManager430>();
			return em;
		}
	};


	class EmLarge : public EmulationManager430
	{
	public:
		static EmulationManagerPtr create()
		{
			boost::shared_ptr<EmLarge> em = boost::make_shared<EmLarge>();
			
			em->mTriggerManager = boost::make_shared<TriggerManager430>(8, 2, 8, 7);	
			em->mTriggerManager->setExtendedComparisons();
			em->mTriggerManager->setExtendedAccessTypes();
			em->mTriggerManager->setBitwiseMasking();

			em->mTriggerConditionManager = boost::make_shared<TriggerConditionManager430>(em->mTriggerManager);	
			em->mBreakpointManager = boost::make_shared<BreakpointManager430>();			
			em->mSequencer = boost::make_shared<Sequencer430>(em->mTriggerManager, false);
			em->mTrace = boost::make_shared<StateStorage430>();
			return em;
		}
	};


	class EmExtraSmall_5xx : public EmulationManager430
	{
	public:
		static EmulationManagerPtr create()
		{
			boost::shared_ptr<EmExtraSmall_5xx> em = boost::make_shared<EmExtraSmall_5xx>();

			em->mTriggerManager = boost::make_shared<TriggerManager430>(2, 0, 2, 0);
			em->mTriggerManager->setExtendedAccessTypes();

			em->mTriggerConditionManager = boost::make_shared<TriggerConditionManager430>(em->mTriggerManager);	
			em->mBreakpointManager = boost::make_shared<BreakpointManager430>();	
			em->mCycleCounter = boost::make_shared<CycleCounter430>(1);
			return em;
		}
	};


	class EmSmall_5xx : public EmulationManager430
	{
	public:
		static EmulationManagerPtr create()
		{
			boost::shared_ptr<EmSmall_5xx> em = boost::make_shared<EmSmall_5xx>();

			em->mTriggerManager = boost::make_shared<TriggerManager430>(3, 1, 4, 0);
			em->mTriggerManager->setExtendedComparisons();
			em->mTriggerManager->setExtendedAccessTypes();

			em->mTriggerConditionManager = boost::make_shared<TriggerConditionManager430>(em->mTriggerManager);	
			em->mBreakpointManager = boost::make_shared<BreakpointManager430>();	
			em->mCycleCounter = boost::make_shared<CycleCounter430>(1);
			return em;
		}
	};


	class EmMedium_5xx : public EmulationManager430
	{
	public:
		static EmulationManagerPtr create()
		{
			boost::shared_ptr<EmMedium_5xx> em = boost::make_shared<EmMedium_5xx>();

			em->mTriggerManager = boost::make_shared<TriggerManager430>(5, 1, 6, 5);
			em->mTriggerManager->setExtendedComparisons();
			em->mTriggerManager->setExtendedAccessTypes();

			em->mTriggerConditionManager = boost::make_shared<TriggerConditionManager430>(em->mTriggerManager);	
			em->mBreakpointManager = boost::make_shared<BreakpointManager430>();	
			em->mCycleCounter = boost::make_shared<CycleCounter430>(1);
			em->mSequencer = boost::make_shared<Sequencer430>(em->mTriggerManager, true);
			return em;
		}
	};


	class EmLarge_5xx : public EmulationManager430
	{
	public:
		static EmulationManagerPtr create()
		{
			boost::shared_ptr<EmLarge_5xx> em = boost::make_shared<EmLarge_5xx>();

			em->mTriggerManager = boost::make_shared<TriggerManager430>(8, 2, 10, 7);
			em->mTriggerManager->setExtendedComparisons();
			em->mTriggerManager->setExtendedAccessTypes();
			em->mTriggerManager->setBitwiseMasking();

			em->mTriggerConditionManager = boost::make_shared<TriggerConditionManager430>(em->mTriggerManager);	
			em->mBreakpointManager = boost::make_shared<BreakpointManager430>();	
			em->mCycleCounter = boost::make_shared<CycleCounter430>(2);
			em->mSequencer = boost::make_shared<Sequencer430>(em->mTriggerManager, false);

			boost::shared_ptr<StateStorage430> stateStorage = boost::make_shared<StateStorage430>();
			em->mTrace = stateStorage;
			em->mVariableWatch = stateStorage;			
			return em;
		}
	};
}

	
EmulationManagerPtr EmulationManager430::create(uint8_t emulationLevel)
{
	typedef EmulationManagerPtr (*CreatorFunction)();
	const CreatorFunction creatorFunctions[8] = 
	{
		EmNone::create,
		EmSmall::create,
		EmMedium::create,
		EmLarge::create,
		EmExtraSmall_5xx::create,
		EmSmall_5xx::create,
		EmMedium_5xx::create,
		EmLarge_5xx::create
	};

	return (emulationLevel < 8) ? creatorFunctions[emulationLevel]() : EmNone::create();		
}
