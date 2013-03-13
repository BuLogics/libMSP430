/*
 * CycleCounter.cpp
 *
 * Handles cycle counter functionality
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

#include "DeviceInfo.h"
#include "CycleCounter.h"
#include "DebugManagerV3.h"
#include "DeviceHandleV3.h"
#include "EemMemoryAccess.h"
#include "CpuMemoryAccess.h"
#include "TemplateDeviceDb/MSP430Defaults.h"
#include <boost/foreach.hpp>

using namespace TI::DLL430;
using namespace TI::DLL430::TemplateDeviceDb;

static const unsigned resetBit = (1 << 6);
static const unsigned defaultMode = 0x5;


//Used for old software cycle counter code
static uint32_t CPUCycles = 0;
static bool deviceHasMSP430X = false;

uint32_t GetCycles(uint16_t Instruction);
uint32_t GetExtensionCycles(uint16_t wExtensionWord, uint16_t Instruction);



CycleCounter::CycleCounter(DeviceHandleV3* deviceHandle, const DeviceInfo* info) 
	: deviceHandle(deviceHandle)
	, emulationLevel((EMEX_MODE)info->getEmulationLevel())
	, cycleCounterControl(resetBit | defaultMode)
	, counterValue(0)
	, isCpuX(false)
{
	BOOST_FOREACH(const DeviceInfo::memoryInfo& memInfo, info->getMemoryInfo())
	{
		if (memInfo.name == "CPU" && memInfo.bits == 20)
		{
			isCpuX = true;
		}
	}
}



void CycleCounter::reset()
{
	if (emulationLevel >= EMEX_EXTRA_SMALL_5XX)
	{
		MemoryManager* mm = deviceHandle->getMemoryManager();
		EemMemoryAccess* ema = (EemMemoryAccess*)mm->getMemoryArea("EEM");

		counterValue = 0;
		cycleCounterControl = resetBit | defaultMode;
		ema->writeEemRegister(CCNT0CTL, cycleCounterControl) && ema->sync();
	}
}


void CycleCounter::configure()
{
	if (emulationLevel >= EMEX_EXTRA_SMALL_5XX)
	{
		MemoryManager* mm = deviceHandle->getMemoryManager();
		EemMemoryAccess* ema = (EemMemoryAccess*)mm->getMemoryArea("EEM");

		cycleCounterControl = defaultMode;
		ema->writeEemRegister(CCNT0CTL, cycleCounterControl) && ema->sync();
	}
}


uint64_t CycleCounter::read()
{
	if (emulationLevel >= EMEX_EXTRA_SMALL_5XX)
	{ 
		MemoryManager* mm = deviceHandle->getMemoryManager();
		EemMemoryAccess* ema = (EemMemoryAccess*)mm->getMemoryArea("EEM");

		union CycleCount { struct { uint32_t low, high; }; uint64_t value; };
		CycleCount cycleCount;

		ema->readEemRegister(CCNT0L, &cycleCount.low) && ema->sync();
		ema->readEemRegister(CCNT0H, &cycleCount.high) && ema->sync();
		
		counterValue = 0;

		uint32_t factor = 1;
		uint32_t lsfr2hex[16] = {0x0, 0x1, 0x2, 0x7, 0x5, 0x3, 0x8, 0xb, 0xe, 0x6, 0x4, 0xa, 0xd, 0x9, 0xc, 0};
		for (int i = 0; i < 10; ++i)
		{
			counterValue += factor * lsfr2hex[(cycleCount.value & 0xf)];
			cycleCount.value >>= 4;
			factor *= 15;
		}	
	}

	return counterValue;
}





void CycleCounter::countInstruction(uint32_t instruction, bool steppedIntoInterrupt)
{
	if (emulationLevel < EMEX_EXTRA_SMALL_5XX)
	{
		CPUCycles = 0;
		deviceHasMSP430X = isCpuX;

		uint16_t instructionWord = (instruction & 0xffff);
		uint16_t extensionWord = 0;

		if ((instruction & 0xF800) == 0x1800)
		{
			extensionWord = (instruction & 0xffff);
			instruction >>= 16;
		}

		GetCycles(instructionWord);
		if (extensionWord != 0)
		{
			GetExtensionCycles(extensionWord, instructionWord);
		}

		counterValue += CPUCycles;

		if (steppedIntoInterrupt)
		{
			counterValue += isCpuX ? 5 : 6;
		}
	}
}





#define PC 0
#define SP 1
#define SR 2
#define CG1 2
#define CG2 3

// #defines. ------------------------------------------------------------------

// BIT MASKS FOR ADDRESSING MODES AND DOUBLE AND SINGLE OPERAND INSTRUCTION FORMATS (As/Ad):
#define AM_RegMode      0x0000   //Register Mode Rn
#define AM_IdxMode      0x0001   //Indexed Mode X(Rn)
#define AM_IdtMode      0x0002   //Indirect Register Mode @Rn
#define AM_IdAMode      0x0003   //Indirect Autoincrement Register Mode @Rn+
#define IM_SrcMask      0x0f00   //Isolation of source register
#define IM_DstMask      0x000f   //Isolation of destination register
#define IM_AsMask       0x0030   //Isolation of source addresss mode
#define IM_AdMask       0x0080   //Isolation of destination addresss mode
#define IM_ByteOpMask   0x0040   //Isolation of byte operation identifier
#define IF_DOpCodeMask  0xf000   //OpCode of Double Instruction Format
#define IF_SOpCodeMask  0xff80   //OpCode of Single Instruction Format
#define IF_JumpCond     0xfc00   //OpCode and condition for all jump instructions
#define IM_JumpSign     0x0200   //Isolation of offset sign bit
#define IM_JumpOffset   0x01ff   //Isolation of offset

// DOUBLE OPERAND INSTRUCTION IDENTIFICATION:
#define DOI_MOV         0x4000
#define DOI_ADD         0x5000
#define DOI_ADDC        0x6000
#define DOI_SUBC        0x7000
#define DOI_SUB         0x8000
#define DOI_CMP         0x9000
#define DOI_DADD        0xA000
#define DOI_BIT         0xB000
#define DOI_BIC         0xC000
#define DOI_BIS         0xD000
#define DOI_XOR         0xE000
#define DOI_AND         0xF000

// SINGLE OPERAND INSTRUCTION IDENTIFICATION:
#define SOI_RRC         0x1000
#define SOI_SWPB        0x1080
#define SOI_RRA         0x1100
#define SOI_SXT         0x1180
#define SOI_PUSH        0x1200
#define SOI_CALL        0x1280

// NO OPERAND INSTRUCTION IDENTIFICATION:
#define NOI_RETI        0x1300

// CONDITIONAL JUMP INSTRUCTION IDENTIFICATION:
#define CJ_JNE          0x2000
#define CJ_JEQ          0x2400
#define CJ_JNC          0x2800
#define CJ_JC           0x2C00
#define CJ_JN           0x3000
#define CJ_JGE          0x3400
#define CJ_JL           0x3800
#define CJ_JMP          0x3C00

// File local variables. ---------------------------------------------------

// CYCLES FOR DOUBLE OPERAND INSTRUCTIONS:
// The following Array contains the cycles for each DOIF. To get the cycles for a DOIF,
// place As into Bit 2+3 and Ad into Bit 0 of a Byte. Bit 1 and 4 to 7 are zero. This Byte
// has to be used as index into the following array 

static unsigned int const DOICycles[16] = {
/*Src,Dst           AdAs   Index                   Cycles[AdAs] */
/*Rn,Rn             0000   0    */                 1,
/*X(Rn),Rn          0001   1    */                 3,
/*@Rn,Rn            0010   2    */                 2,
/*@Rn+,Rn           0011   3    */                 2,
/*Rn,@Rn            ----   4    not possible*/     0,
/*X(Rn),@Rn         ----   5    not possible*/     0,  
/*@Rn,@Rn           ----   6    not possible*/     0,  
/*@Rn+,@Rn          ----   7    not possible*/     0,  
/*Rn,X(Rn)      1000   8    */                 4,
/*X(Rn),X(Rn)       1001   9    */                 6,
/*@Rn,X(Rn)         1010   A    */                 5,
/*@Rn+,X(Rn)        1011   B    */                 5,
/*Rn,@Rn+           ----   C    not possible*/     0,
/*X(Rn),@Rn+        ----   D    not possible*/     0,
/*@Rn,@Rn+          ----   E    not possible*/     0,
/*@Rn+,@Rn+         ----   F    not possible*/     0
};

// CYCLES FOR SINGLE OPERAND INSTRUCTIONS:
// The following Arrays contain the cycles for each SOIF. To get the cycles for a SOIF,
// place Ad into Bit 0 and 1 of a Byte. Bit 2 to 7 are zero. This Byte has to be used as
// index into the corresponding array 

// RRC, RRA, SWPB, SXT:
static unsigned int const SOICycles_1[4] = {
/*Dst         Ad   Index       Cycles[AsAd] */
/*Rn          00   0    */     1,
/*X(Rn)       01   1    */     4,
/*@Rn         10   2    */     3,
/*@Rn+        11   3    */     3
};

// PUSH:
static unsigned int const SOICycles_2[4] = {
/*Dst         Ad   Index       Cycles[AsAd] */
/*Rn          00   0    */     3,
/*X(Rn)       01   1    */     5,
/*@Rn         10   2    */     4,
/*@Rn+        11   3    */     4
};

// CALL:
static unsigned int const SOICycles_3[4] = {
/*Dst         Ad   Index       Cycles[AsAd] */
/*Rn          00   0    */     4,
/*X(Rn)       01   1    */     5,
/*@Rn         10   2    */     4,
/*@Rn+        11   3    */     5
};

// PUSH in CPUX:
static unsigned int const XSOICycles_2[4] = {
/*Dst         Ad   Index       Cycles[AsAd] */
/*Rn          00   0    */     3,
/*X(Rn)       01   1    */     4,   // == EDE, &EDE
/*@Rn         10   2    */     3,
/*@Rn+        11   3    */     3    // == #N
};

// CALL in CPUX:
static unsigned int const XSOICycles_3[4] = {
/*Dst         Ad   Index       Cycles[AsAd] */
/*Rn          00   0    */     3,
/*X(Rn)       01   1    */     4,   // == EDE, &EDE
/*@Rn         10   2    */     4,
/*@Rn+        11   3    */     4    // == #N
};


// CYCLES FOR NO OPERAND INSTRUCTIONS:

static unsigned int const NOICycles = 5;

// CYCLES FOR CONDITONIAL JUMP INSTRUCTIONS:

static unsigned int const JMPCycles = 2;

// Functions. --------------------------------------------------------------

//--------------------------------------------------------------------------
// void DOIF(uint16_t Instruction)
//
void DOIF(uint16_t Instruction)
{
  uint16_t SrcRegNo  = (Instruction & IM_SrcMask) >> 8;
  uint16_t DstRegNo  = Instruction & IM_DstMask;
  uint16_t As        = (Instruction & IM_AsMask) >> 4;
  uint16_t Ad        = (Instruction & IM_AdMask) >> 7;
  uint16_t DOpCode   = (Instruction & IF_DOpCodeMask);
  //bool bByteInst = Instruction & IM_ByteOpMask;

    //Store used cycles of the instruction.
    if ((SrcRegNo == CG1 && As != AM_IdxMode) || SrcRegNo == CG2) As = AM_RegMode; //constant generator can be handled as register mode
    CPUCycles += DOICycles[((Ad<<3)|As)];
/*    if (DstRegNo == PC && Ad == AM_RegMode && (As == AM_RegMode || (As == AM_IdAMode && SrcRegNo != PC)))
      CPUCycles++;
*/
    if (DstRegNo == PC && Ad == AM_RegMode && (As == AM_IdtMode || As == AM_IdAMode))
      CPUCycles++;
/*
	// Seems like this line reflects behaviour of the real silicon
	// Specification says that 3 cycles are needed, actually this line will add one more cycle.
	// affected instr: <INSTR> #N, PC
    if (DstRegNo == PC && Ad == AM_RegMode && (As == AM_RegMode || (As == AM_IdAMode && SrcRegNo == PC)))
      CPUCycles++;
*/
    if (DstRegNo == PC && Ad == AM_RegMode && As == AM_RegMode)
      CPUCycles++;

    // check if the device has CPUX
    if(deviceHasMSP430X)
    {
      // check instruction: MOV || CMP || BIT
      if(DOpCode == DOI_MOV || DOpCode == DOI_CMP || DOpCode == DOI_BIT)
      {
        // check addressing DST mode: x(Rn) || EDE || &EDE
        if(Ad == AM_IdxMode)
        {
          CPUCycles--;
        }
      }
    }
}

//--------------------------------------------------------------------------
// void SOIF(uint16_t Instruction)
//
void SOIF(uint16_t Instruction)
{
  uint16_t DstRegNo  = Instruction & IM_DstMask;
  uint16_t Ad        = (Instruction & IM_AsMask) >> 4;
  uint16_t SOICycles = 0;

  switch(Instruction & IF_SOpCodeMask)
  {
    case SOI_PUSH: SOICycles = 2;
           break;
    case SOI_CALL: SOICycles = 3;
           break;
    case SOI_RRC:  SOICycles = 1;
           break;
    case SOI_RRA:  SOICycles = 1;
           break;
    case SOI_SWPB: SOICycles = 1;
           break;
    case SOI_SXT:  SOICycles = 1;
           break;
    case NOI_RETI: SOICycles = 0;
           break;
    default:
           break;
  }

  if (DstRegNo == CG2 || (DstRegNo == CG1 && Ad != AM_IdxMode)) Ad = AM_RegMode; //constant generator can be handled as register mode
  switch (SOICycles)
    {
      case 1:  CPUCycles += SOICycles_1[Ad];
               break;
      case 2:  if (deviceHasMSP430X)
               {
                 CPUCycles += XSOICycles_2[Ad];
                 // one more cycle for X(Rn) when Rn == SP
                 if(DstRegNo == SP && Ad == AM_IdxMode)
                 {
                   CPUCycles++;
                 }
               }
               else
               {
                 CPUCycles += SOICycles_2[Ad];
               }
               break;
      case 3:  if (deviceHasMSP430X)
               {
                 CPUCycles += XSOICycles_3[Ad];
                 // one more cycle for X(Rn) when Rn == SP
                 if(DstRegNo == SP && Ad == AM_IdxMode)
                 {
                   CPUCycles++;
                 }
               }
               else
               {
                 CPUCycles += SOICycles_3[Ad];
               }
               break;
      default: CPUCycles += NOICycles;
               if (deviceHasMSP430X)
               {
                 // RETI needs 3 cycles
                 CPUCycles -= 2; // CPUX needs two cycles less
               }
               break;
    }
}

//--------------------------------------------------------------------------
// void Jumps(uint16_t Instruction)
//
void Jumps(uint16_t Instruction)
{
    CPUCycles += JMPCycles;
};

//--------------------------------------------------------------------------
#define ADDRI_OpCodeMask        0xF0F0

#define MOVA_ATRsrcRdst         0x0000
#define MOVA_ATRsrcPlusRdst     0x0010
#define MOVA_Abs20Rdst          0x0020
#define MOVA_IdxRsrcRdst        0x0030
#define MOVA_RsrcAbs20          0x0060
#define MOVA_RsrcIdxRdst        0x0070

#define MOVA_Imm20Rdst          0x0080
#define CMPA_Imm20Rdst          0x0090
#define ADDA_Imm20Rdst          0x00A0
#define SUBA_Imm20Rdst          0x00B0
#define MOVA_RsrcRdst           0x00C0
#define CMPA_RsrcRdst           0x00D0
#define ADDA_RsrcRdst           0x00E0
#define SUBA_RsrcRdst           0x00F0

#define Rotate_OpCodeId         0xF0E0 // must be 0x0040 to be an rotate instruction!!

//--------------------------------------------------------------------------
// void ADDRI(uint16_t Instruction)
//
void ADDRI(uint16_t Instruction)
{
  uint16_t DstRegNo  = Instruction & IM_DstMask;
  uint16_t BitLoc    = (Instruction & 0x0C00) >> 10;

  // is is a rotate instruction??
  if((Instruction & Rotate_OpCodeId) == 0x0040)
  {
    // The rotate instructions take 1, 2, 3 or 4 CPU cycles according to the
    // number of bit positions shifted.
    CPUCycles += (BitLoc + 1);
  }
  else
  {
    switch(Instruction & ADDRI_OpCodeMask)
    {
    case MOVA_ATRsrcRdst:
    case MOVA_ATRsrcPlusRdst:
      CPUCycles += 3;
      break;

    case MOVA_Abs20Rdst:
    case MOVA_IdxRsrcRdst:
    case MOVA_RsrcAbs20:
    case MOVA_RsrcIdxRdst:
      CPUCycles += 4;
      break;

    case MOVA_Imm20Rdst:
    case CMPA_Imm20Rdst:
    case ADDA_Imm20Rdst:
    case SUBA_Imm20Rdst:
      CPUCycles += 2;
      if(DstRegNo == PC)
      {
        CPUCycles++;
      }
      break;
    case MOVA_RsrcRdst:
    case CMPA_RsrcRdst:
    case ADDA_RsrcRdst:
    case SUBA_RsrcRdst:
      CPUCycles ++;
      if(DstRegNo == PC)
      {
        CPUCycles++;
      }
      break;
    }
  }
}


#define CALLA_Rdst        0x0040
#define CALLA_IdxRdst     0x0050
#define CALLA_AtRdst      0x0060
#define CALLA_AtRdstPlus  0x0070
#define CALLA_Abs20       0x0080
#define CALLA_Symbolic    0x0090
#define CALLA_Imm20       0x00B0

#define PUSHM_A           0x1400
#define PUSHM_W           0x1500
#define POPM_A            0x1600
#define POPM_W            0x1700

//--------------------------------------------------------------------------
// void SOIF(uint16_t Instruction)
//
bool XSOIF(uint16_t Instruction)
{
  bool RetState  = true;
  uint16_t DstRegNo  = Instruction & IM_DstMask;
  uint16_t MultBits  = (Instruction & 0x00F0) >> 4;

  switch(Instruction & 0xFF00)
  {
  // RETI or CALLA??
  case NOI_RETI:
    // RETI??
    if(Instruction == NOI_RETI)
    {
      // RETI needs 3 cycles
      CPUCycles += 3;
    }
    else
    {
      switch(Instruction & 0x00F0)
      {
      case CALLA_Imm20:
      case CALLA_Rdst:       CPUCycles += 4; break;

      case CALLA_AtRdst:
      case CALLA_AtRdstPlus: CPUCycles += 5; break; 

      case CALLA_IdxRdst:    if(DstRegNo == SP) CPUCycles++; CPUCycles += 6; break; 
      case CALLA_Abs20:
      case CALLA_Symbolic:   CPUCycles += 6; break; 
      }
    }
    RetState   = false;
    break;

  case PUSHM_A:
  case POPM_A:
    CPUCycles += (2 + 2*(MultBits + 1));
    RetState   = false;
	break;
  case PUSHM_W:
  case POPM_W:
    CPUCycles += (2 + 1*(MultBits + 1));
    RetState   = false;
    break;
  }

  return (RetState);
}

//--------------------------------------------------------------------------
// void GetCycles(uint16_t Instruction)
//
uint32_t GetCycles(uint16_t Instruction)
{
  uint32_t lastCPUCycles = CPUCycles;
  bool RetState  = true;

  switch (Instruction & 0xf000)
      {
    case 0x0000: ADDRI(Instruction); break;  // CPUX Address Instructions
    case 0x1000:
      // look for CPUX instructions first
      if(deviceHasMSP430X)
      {
          RetState = XSOIF(Instruction);
      }
      // and decive if we have to look at the original instruction set
      if (RetState)
      {
        SOIF(Instruction);
      }
      break;   //Single Operand Instruction Format
    case 0x2000:                        //Single Conditional Instruction Jump Format
    case 0x3000: Jumps(Instruction); break;
    default:     DOIF(Instruction);           //Double Operand Instruction Format
      };

  return (CPUCycles - lastCPUCycles);
}

#define EXT_AL  0x0040
#define EXT_GATTER 0x0080

uint32_t GetExtensionCycles(uint16_t wExtensionWord, uint16_t Instruction)
{
  uint32_t lastCPUCycles = CPUCycles;
  uint16_t SrcRegNo  = (Instruction & IM_SrcMask) >> 8;
  uint16_t DstRegNo  = (Instruction & IM_DstMask);
  uint16_t As        = (Instruction & IM_AsMask) >> 4;
  uint16_t Ad        = (Instruction & IM_AdMask) >> 7;
  uint16_t DOpCode   = (Instruction & IF_DOpCodeMask);

  // basically one more CPU cycle is needed
  CPUCycles++;
  // handle a few exceptions

  // 1. <INSTR> @Rn,PC
  if(DstRegNo == PC && As == AM_IdtMode && Ad == AM_RegMode)
  {
    CPUCycles--;
  }

  // 1. <INSTR> @Rn,PC
  if(DstRegNo == PC && SrcRegNo == PC && As == AM_IdAMode && Ad == AM_RegMode)
  {
    CPUCycles--;
  }

  // 2. <INSTR> x(Rn),PC == <INSTR> x(Rn),EDE == <INSTR> x(Rn),&EDE
  if(DstRegNo == PC && As == AM_IdxMode && Ad == AM_RegMode)
  {
    // but only if it's NOT MOV, ADD or SUB
    if(!(DOpCode == DOI_MOV || DOpCode == DOI_ADD || DOpCode == DOI_SUB))
    {
//      CPUCycles++;
    }
  }

  // ADDRESS access?? EXT_AL must be 0!
  if(!(wExtensionWord & EXT_AL))
  {
    // <INSTR> Rn,x(Rm)
    // <INSTR> #N,x(Rm)
    if(Ad == AM_IdxMode)
    {
      if(As == AM_RegMode || As == AM_IdtMode || SrcRegNo == PC)
      {
        CPUCycles += 1;
		if(As == AM_IdxMode)
		{
          CPUCycles += 1;
		}
      }
      else
      {
        CPUCycles += 2;
      }
    }
    // <INSTR> @Rx,Rx
	// <INSTR> @Rx+,Rx
    if(Ad == AM_RegMode && (As == AM_IdxMode || As == AM_IdtMode || As == AM_IdAMode))
    {
        CPUCycles += 1;
    }
  // RRA, RRC
  if((Instruction & 0xF000) == 0x1000 && As != AM_RegMode)
  {
    CPUCycles += 1;
  }
  // PUSHX Rx
  if((Instruction &0xFFF0) == 0x1240)
  {
    CPUCycles += 1;
  }

     if(!(DOpCode == DOI_MOV || DOpCode == DOI_CMP || DOpCode == DOI_BIT))
      {
        // check addressing DST mode: x(Rn) || EDE || &EDE
        if(Ad == AM_IdxMode)
        {
          CPUCycles++;
        }
      }
   }

  // repeat instruction, lower nibbel of wExtensionword
  // defines the number of repetitions
  if(!(wExtensionWord & EXT_GATTER))
  {
    CPUCycles += (wExtensionWord & 0x000F);
  }

  return (CPUCycles - lastCPUCycles);
}
