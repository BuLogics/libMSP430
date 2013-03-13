/*
 * MSP430_Debug.h
 *
 * API for accessing debugging functionality of MSP430 library.
 *
 * Copyright (C) 2004 - 2011 Texas Instruments Incorporated - http://www.ti.com/ 
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

/**            
 \file MSP430_Debug.h
 
 \brief       This file contains the DLL function headers and definitions
              to support the debugging functions of the MSP430.DLL
 
              The MSP430.DLL provides the following debug functionalities:
              - Read/Write CPU registers
              - Set/Clear code breakpoints
              - Single step/Run to breakpoint/Free run the CPU
              - Get CPU state
              - Access the Enhanced Emultation Module registers
 
 \attention   The debugging functions of the DLL are made available only 
              to third party tool developers. Using the Enhanced Emulation Module
              API strictly permits the usage of the functions provided via this
              header file. The functions are only made available due to downward
              compatibility reasons.
 
 \par         Project:
              MSP430 JTAG Interface (MSP430.dll)
 
 \par         Developed using:
              MS Visual C++ 2003/2010
  
 \par         Supported API calls:
              - MSP430_Registers()
              - MSP430_Register()
              - MSP430_Run()
              - MSP430_State()
			  - MSP430_CcGetClockNames()
              - MSP430_CcGetModuleNames()
*/

#ifndef MSP430_DEBUG_H
#define MSP430_DEBUG_H

#include "MSP430.h"

#if defined(__cplusplus)
extern "C" {
#endif

#ifndef MSP430_DEBUG_TYPES
#define MSP430_DEBUG_TYPES

/// Temporary work-around to #defines in stdconst.h (and others)
#undef R0
#undef R1
#undef R2
#undef R3
#undef R4
#undef R5
#undef R6
#undef R7
#undef R8
#undef R9
#undef R10
#undef R11
#undef R12
#undef R13
#undef R14
#undef R15
#undef PC
#undef SP
#undef SR
#undef CG1
#undef CG2
#undef VCC

/// Device registers.
typedef enum DEVICE_REGISTERS {
	R0 = 0,
	R1 = 1,
	R2 = 2,
	R3 = 3,
	R4 = 4,
	R5 = 5,
	R6 = 6,
	R7 = 7,
	R8 = 8,
	R9 = 9,
	R10 = 10,
	R11 = 11,
	R12 = 12,
	R13 = 13,
	R14 = 14,
	R15 = 15,
} DEVICE_REGISTERS_t;

#define PC R0
#define SP R1
#define SR R2
#define CG1 R2
#define CG2 R3

/// Device registers masks.
#define MASKREG(REG) (1 << REG)
/// Device registers mask including all registers.
#define ALL_REGS 0xffff
	
/// Run modes.
typedef enum RUN_MODES {
	FREE_RUN = 1, /**< Run the device. Set breakpoints (if any) are disabled */
	SINGLE_STEP = 2, /**< A single device instruction is executed. Interrupt processing is supported */
	RUN_TO_BREAKPOINT = 3, /**< Run the device. Set breakpoints (if any) are enabled */
} RUN_MODES_t;

/// State modes.
typedef enum STATE_MODES {
	STOPPED = 0, /**< The device is stopped */
	RUNNING = 1, /**< The device is running or is being single stepped */
	SINGLE_STEP_COMPLETE = 2, /**< The device is stopped after the single step operation is complete */
	BREAKPOINT_HIT = 3, /**< The device is stopped as a result of hitting an enabled breakpoint */
	LPMX5_MODE = 4, /**< The device is in LPMx.5 low power mode  */
	LPMX5_WAKEUP = 5, /**<	The device woke up from LPMx.5 low power mode */
} STATE_MODES_t;

/// One of the following enumerations is returned in device.emulation
typedef enum EMEX_MODE {
	/// Device has no Emex module.
	EMEX_NONE = 0,
	/// Device Emex module has two breakpoints.
	EMEX_LOW = 1,
	/// Device Emex module has three breakpoints and range comparison.
	EMEX_MEDIUM = 2,
	/// Device Emex module has eight breakpoints, range comparison, state storage, and trigger sequencer,
	EMEX_HIGH = 3,
    /// Device Emex module has 2 breakpoints and range comparison
    EMEX_EXTRA_SMALL_5XX = 4,
	/// Device Emex module has 4 breakpoints and range comparison
    EMEX_SMALL_5XX = 5,
	/// Device Emex module has 6 breakpoints, range comparison and trigger sequencer,
    EMEX_MEDIUM_5XX =6,
	/// Device Emex module has 8 or 10 breakpoints, range comparison, state storage, and trigger sequencer,
	EMEX_LARGE_5XX = 7
} EMEX_MODE_t;

/// One of the following enumerations is returned in device.clockControl
typedef enum DEVICE_CLOCK_CONTROL {
	GCC_NONE = 0, /**< Device has no clock control. The system clock continue to function when the device is stopped by JTAG */
	GCC_STANDARD = 1, /**< Device has General Clock Control register */
	GCC_EXTENDED = 2 /**< Device has Extended General Clock Control register and Module Clock Control register 0. */
} DEVICE_CLOCK_CONTROL_t;

#if ! defined(uController)
/// An array of NULL terminated string pointers that point 
/// to the string descriptions for each bit of the device
/// EEM General Clock Control register GENCLKCNTRL.
typedef struct EEM_GCLKCTRL {
   /// MCLKCTRL0F to MCLKCTRL00 reflect the bit description strings for MCLKCTRL0
   CHAR* GENCLKCTRLF; 
   CHAR* GENCLKCTRLE;
   CHAR* GENCLKCTRLD; 
   CHAR* GENCLKCTRLC;
   CHAR* GENCLKCTRLB; 
   CHAR* GENCLKCTRLA;
   CHAR* GENCLKCTRL9; 
   CHAR* GENCLKCTRL8;
   CHAR* GENCLKCTRL7; 
   CHAR* GENCLKCTRL6;
   CHAR* GENCLKCTRL5; 
   CHAR* GENCLKCTRL4;
   CHAR* GENCLKCTRL3; 
   CHAR* GENCLKCTRL2;
   CHAR* GENCLKCTRL1; 
   CHAR* GENCLKCTRL0;
} EemGclkCtrl_t;

/// An array of NULL terminated string pointer that point 
/// to the string descriptions for each bit of the device
/// EEM Module Clock Control registers MCLKCTRL0 and MCLKCTRL1.
typedef struct EEM_MCLKCTRL {
	/// MCLKCTRL0F to MCLKCTRL00 reflect the bit description strings for MCLKCTRL0
	CHAR* MCLKCTRL0F;
	CHAR* MCLKCTRL0E;
	CHAR* MCLKCTRL0D;
	CHAR* MCLKCTRL0C;
	CHAR* MCLKCTRL0B;
	CHAR* MCLKCTRL0A;
	CHAR* MCLKCTRL09;
	CHAR* MCLKCTRL08;
	CHAR* MCLKCTRL07;
	CHAR* MCLKCTRL06;
	CHAR* MCLKCTRL05;
	CHAR* MCLKCTRL04;
	CHAR* MCLKCTRL03;
	CHAR* MCLKCTRL02;
	CHAR* MCLKCTRL01;
	CHAR* MCLKCTRL00;

	/// MCLKCTRL1F to MCLKCTRL10 reflect the bit description strings for MCLKCTRL1
	CHAR* MCLKCTRL1F;
	CHAR* MCLKCTRL1E;
	CHAR* MCLKCTRL1D;
	CHAR* MCLKCTRL1C;
	CHAR* MCLKCTRL1B;
	CHAR* MCLKCTRL1A;
	CHAR* MCLKCTRL19;
	CHAR* MCLKCTRL18;
	CHAR* MCLKCTRL17;
	CHAR* MCLKCTRL16;
	CHAR* MCLKCTRL15;
	CHAR* MCLKCTRL14;
	CHAR* MCLKCTRL13;
	CHAR* MCLKCTRL12;
	CHAR* MCLKCTRL11;
	CHAR* MCLKCTRL10;
} EemMclkCtrl_t;
#endif // end of def uController

/**
 \brief     EEM (Extended) General Clock Control.
 
 \full      The EEM (Extended) General Clock Control allows one to configure the behavior of the system clocks
            (i.e., ACLK, MCLK, SMCLK, TACLK) when the device is stopped by JTAG (i.e., when the device is not
			running or being single stepped). The default behavior of the clocks is to stop	when the device 
			is stopped by JTAG.
*/

/// Bits of the EEM General Clock Control register (F41x).
#define TCE_SMCLK (1 << 0) /**< Clock SMCLK with TCLK. See Note 1. */
#define ST_ACLK   (1 << 1) /**< Stop ACLK */
#define ST_SMCLK  (1 << 2) /**< Stop SMCLK */
#define TCE_MCLK  (1 << 3) /**< Clock functional MCLK with TCLK. See Note 1 */
#define JT_FLLO   (1 << 4) /**< Switch off FLL */
#define ST_TACLK  (1 << 5) /**< Stop TACLK */

/// Bits of the EEM Extended General Clock Control register (F43x/F44x).
#define ECLK_SYN  (1 << 0) /**< Emulation clock synchronization. See Note 1 */
//#define ST_ACLK   (1 << 1) /**< Stop ACLK. See Note 2 */
//#define ST_SMCLK  (1 << 2) /**< Stop SMCLK. See Note 2 */
#define ST_MCLK   (1 << 3) /**< Stop MCLK */
//#define JT_FLLO   (1 << 4) /**< Switch off FLL. See Note 2 */
#define FORCE_SYN (1 << 5) /**< Force JTAG synchronization. See Note 1 */

/**
 \brief     EEM (Extended) General Clock Control.

 \note      Note 1: Not recommended for use with MSP430_Configure(CLK_CNTRL_MODE, <bits>);

 \note      Note 2: ST_ACLK, ST_SMCLK, and JT_FLLO *are* supported by the EEM Extended General Clock Control register;
            Their definitions were commented-out so as not to conflict with the identical definitions for the
			EEM General Clock Control register.

 \full      In addition to the above EEM Extended General Clock Control register features, the MSP430F449
            supports four emulation modes (EMU_MODE_F4xx_xxx) using the EEM General Control register.
			Use function MSP430_Configure(EMULATION_MODE, <bits>); to configure the bits of the EEM General Control
			register.

 \note      Note 1. Only the EMU_MODE_F4xx_xxx bits are supported by EMULATION_MODE.

 \note      Note 2. MSP430_Configure(EMULATION_MODE, <bits>); will reset the device with PUC in order to change the
            emulation mode.
*/
 
/// Bits of the EEM General Control register.
/// Do not use with MSP430_Configure(EMULATION_MODE, <bits>);
#define	EEM_EN          (1 << 0)
/// Do not use with MSP430_Configure(EMULATION_MODE, <bits>);
#define	CLEAR_STOP      (1 << 1)
/// Do not use with MSP430_Configure(EMULATION_MODE, <bits>);
#define EMU_CLK_EN      (1 << 2)
/// Do not use with MSP430_Configure(EMULATION_MODE, <bits>);
#define EMU_FEAT_EN     (1 << 3)
/// Do not use with MSP430_Configure(EMULATION_MODE, <bits>);
#define DEB_TRIG_LATCH  (1 << 4)

/// The actual name of this bit is STOPPED.
/// Do not use with MSP430_Configure(EMULATION_MODE, <bits>);
#define	EEM_STOPPED     (1 << 7) 

/// (Emulate) F44x 100 pins. This is the normal device mode.
#define EMU_MODE_F44X_100	0x0000 
/// Emulate F43x 100 pins.
#define EMU_MODE_F43X_100	0x4000 
/// Emulate F4xx 64 pins.
#define EMU_MODE_F4XX_64	0x5000 
/// Emulate F4xx 80 pins.
#define EMU_MODE_F4XX_80	0x6000 

#endif /* MSP430_DEBUG_TYPES */

/**
\fn    STATUS_T MSP430_Registers(LONG registers[], LONG mask, LONG rw);

\brief   Read and write the device registers.

\note    1. It is not possible to read and write registers CG1 and CG2.
\note    2. This function does not read and write the actual device registers. Rather, copies of the device
            registers are read and written. The register copies are updated (for read) after a device reset,
	        and after MSP430_State() indicates BREAKPOINT_HIT, SINGLE_STEP_COMPLETE, or STOPPED (after
	        stopping a running device [stop = TRUE]). The register copies are written to the device prior to
	        MSP430_Run() with FREE_RUN, SINGLE_STEP, and RUN_TO_BREAKPOINT.
 
\param   registers: The destination of registers read from the device (rw = READ),\n
                    and the source of registers written to the device (rw = WRITE).
\param   mask:		A bit-mask which identifies which registers are read/written\n
                    (Bit 0: R0, Bit 1: R1, and so on).
\param   rw:		Specify a read (READ) or write (WRITE) operation.

\return  STATUS_OK:    The registers were read or written.
\n       STATUS_ERROR: The registers were not read or written.

\par     Error codes:
         PARAMETER_ERR
*/
DLL430_SYMBOL STATUS_T WINAPI MSP430_Registers(LONG* registers, LONG mask, LONG rw);

#define MSP430_Read_Registers(REGISTERS, MASK) MSP430_Registers(REGISTERS, MASK, READ)
#define MSP430_Write_Registers(REGISTERS, MASK) MSP430_Registers(REGISTERS, MASK, WRITE)

/**
\fn    STATUS_T MSP430_ExtRegisters(LONG registers[], LONG mask, LONG rw);

\brief   Read and write the extended device registers.
 
\param   address:   The address of the extended register area
\param   buffer:    The destination of registers read from the device (rw = READ),\n
                    and the source of registers written to the device (rw = WRITE).
\param   rw:		Specify a read (READ) or write (WRITE) operation.

\return  STATUS_OK:    The registers were read or written.
\n       STATUS_ERROR: The registers were not read or written.

\par     Error codes:
         PARAMETER_ERR
*/
DLL430_SYMBOL STATUS_T WINAPI MSP430_ExtRegisters(LONG address, CHAR * buffer, LONG count,LONG rw);

#define MSP430_Read_ExtRegister(ADDRESS, BUFFER) MSP430_ExtRegisters(ADDRESS, BUFFER,1 ,READ)
#define MSP430_Write_ExtRegister(ADDRESS, BUFFER) MSP430_ExtRegisters(ADDRESS, BUFFER, 1, WRITE)

/**
\fn    STATUS_T MSP430_Register(LONG* reg, LONG regNb, LONG rw);

\brief   Read and write only one register of the device.

\note    1. It is not possible to read and write registers CG1 and CG2.
\note    2. This function does not read and write the actual device registers. Rather, copies of the device
            registers are read and written. The register copies are updated (for read) after a device reset,
	        and after MSP430_State() indicates BREAKPOINT_HIT, SINGLE_STEP_COMPLETE, or STOPPED (after
	        stopping a running device [stop = TRUE]). The register copies are written to the device prior to
	        MSP430_Run() with FREE_RUN, SINGLE_STEP, and RUN_TO_BREAKPOINT.
 
\param   reg:   The destination of the register read from the device (rw = READ),\n
                and the source of the register written to the device (rw = WRITE).
\param   regNb:	Number of the register to be read/written (0 - 15).
\param   rw:	Specify a read (READ) or write (WRITE) operation.

\return  STATUS_OK:    The register was read or written.
\n       STATUS_ERROR: The register was not read or written.

\par     Error codes:
         PARAMETER_ERR
*/
DLL430_SYMBOL STATUS_T WINAPI MSP430_Register(LONG* reg, LONG regNb, LONG rw);

#define MSP430_Read_Register(REG, REGNB) MSP430_Register(REG, REGNB, READ)
#define MSP430_Write_Register(REG, REGNB) MSP430_Register(REG, REGNB, WRITE)


/**
\fn    STATUS_T MSP430_Run(LONG mode, LONG releaseJTAG);

\brief   Run the device using the specified mode. JTAG control signals are optionally released.

\note    1. MSP430_OpenDevice() must have been called prior to calling this function.
\note    2. Use MSP430_State() to determine the device state after an MSP430_Run() operation.
\note    3. Use MSP430_State() to update the device registers before using MSP430_Registers() to read the
            device registers.
\note    4. DO NOT call this function twice without stopping device's CPU in between by calling
            MSP430_State() with parameter 'stop' set to TRUE.
 
\param   mode:		  The specified run mode:
			- FREE_RUN:          Run the device. Set breakpoints (if any) are disabled.
			- SINGLE_STEP:       A single device instruction is executed. Interrupt processing is supported.
			- RUN_TO_BREAKPOINT: Run the device. Set breakpoints (if any) are enabled.
\param   releaseJTAG: The JTAG control signals are released when TRUE.

\return  STATUS_OK:    The run operation encountered no errors.
\n       STATUS_ERROR: The run operation encountered errors.

\par     Error codes:
         DEVICE_UNKNOWN_ERR
\n		 NO_DEVICE_ERR
\n		 THREAD_ACTIVE_ERR
\n       STEP_ERR
\n       RUN_ERR
\n       BREAKPOINT_ERR
\n       PARAMETER_ERR
*/
DLL430_SYMBOL STATUS_T WINAPI MSP430_Run(LONG mode, LONG releaseJTAG);

/**
\fn    STATUS_T MSP430_State(LONG* state, LONG stop, LONG* pCPUCycles);

\brief   Determine the state of the device. The device is optionally stopped.
         During single step operations, a count of CPU cycles executed is maintained.

\note    1. MSP430_OpenDevice() must have been called prior to calling this function.
\note    2. Limitations:
			- After MSP430_Run() was called DO NOT call MSP430_State() other than with
			  parameter 'stop' set to TRUE.
			- Calling MSP430_State() with parameter 'stop' set to TRUE will cause the device's CPU
			  to be stopped. DLLv3 will block until the device is stopped.
			- When the device's CPU is not running (MSP430_Run() was not called prior) MSP430_State()
			  can be called without any restrictions.
\note    3. To determine the device state, the JTAG control signals are required. If the JTAG control signals were
	        released, the signals are reconnected.
\note    4. The number of CPU cycles executed is zeroed when the device is reset (MSP430_Reset()), and when the
            device is run (MSP430_Run()) with FREE_RUN and RUN_TO_BREAKPOINT.
\note    5. Use MSP430_State() to update the device registers before using MSP430_Registers() to read the
            device registers.
 
\param   state:		 The device state:
			  - STOPPED:              The device is stopped.
			  - RUNNING:              The device is running or is being single stepped.
			  - SINGLE_STEP_COMPLETE: The device is stopped after completing the single step operation.
			  - BREAKPOINT_HIT:       The device is stopped as a result of hitting an enabled breakpoint.
\param   stop:		 The device is stopped when TRUE.
\param   pCPUCycles: The cumulative number of CPU cycles executed during single step (or -1 if invalid).

\return  STATUS_OK:    The device state was determined.
\n       STATUS_ERROR: The device state was not determined.

\par     Error codes:
         PARAMETER_ERR
\n       STATE_ERR
\n       STEP_ERR
*/
DLL430_SYMBOL STATUS_T WINAPI MSP430_State(LONG* state, LONG stop, LONG* pCPUCycles);


/**
\fn   STATUS_T WINAPI MSP430_CcGetClockNames(LONG localDeviceId, EemGclkCtrl_t** CcClockNames)

\brief   Returns the string descriptions of the devices' EEM General Clock Control
         register GENCLKCNTRL.

\note    1. The reference of a pointer of the type EemGclkCtrl_t needs to be passed
            to this function as parameter 2 (see application example for details).

\param   localDeviceId: Not currently used. Always returns clock names for current device.
\param   CcClockNames:	A pointer to a pointer of the struct type EemGclkCtrl_t
                        defined in MSP430_Debug.h (see application example).
\code
  Application example:

  #include "MSP430_Debug.h"
          
  EemGclkCtrl_t* globalCcClockNames;

  MSP430_CcGetClockNames(deviceId, &globalCcClockNames);

\endcode

\return  STATUS_OK:    The function was executed successfully.
*/
#ifndef uController
STATUS_T WINAPI MSP430_CcGetClockNames(LONG localDeviceId, EemGclkCtrl_t** CcClockNames);
#endif


/**
\fn   STATUS_T MSP430_CcGetModuleNames(LONG localDeviceId, EemMclkCtrl_t** CcModuleNames)

\brief   Returns the string descriptions of the devices' EEM Module Clock Control
         registers MCLKCTRL0 and MCLKCTRL1.

\note    1. The reference of a pointer of the type EemMclkCtrl_t needs to be passed
            to this function as parameter 2 (see application example for details).

\param   localDeviceId: Not currently used. Always returns module names for current device.
\param   CcModuleNames:	A pointer to a pointer of the struct type EemMclkCtrl_t
                        defined in MSP430_Debug.h (see application example).
\code
  Application example:

  #include "MSP430_Debug.h"
          
  EemMclkCtrl_t* globalCcModuleNames;

  MSP430_CcGetModuleNames(deviceId, &globalCcModuleNames);

\endcode

\return  STATUS_OK:    The function was executed successfully.
*/
#if ! defined(uController)
DLL430_SYMBOL STATUS_T WINAPI MSP430_CcGetModuleNames(LONG localDeviceId, EemMclkCtrl_t** CcModuleNames);
#endif



#if defined(__cplusplus)
}
#endif

#endif // MSP430_DEBUG_H
