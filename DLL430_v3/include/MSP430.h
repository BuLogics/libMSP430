/*
 * MSP430.h
 *
 * API for accessing MSP430 devices via JTAG.
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
 * \mainpage MSP430 DLL API Documentation
 *
 * \section intro Introduction
 *
 * The MSP430.DLL is the interface between the JTAG/EEM (Enhanced Emulation Module)
 * on the MSP430 and the IDE (Integrated Development Environment). The DLL usually is
 * running on the same platform the IDE is running.
 *
 * The MSP430.dll consists of three parts:
 * - one for the basic functions like initializing the hardware interface to the device,
 *   setting the supply voltage, erasing and programming an MSP430 device.\n
 *   All of these functions are available via the MSP430.h header file.
 * - a second part to enable debugging user code programmed in the device's
 *   flash memory. The documentation describes the Application Programming Interface (API)
 *   to access the Enhanced Emulation Module (EEM) of a MSP430 device
 *   using the MSP430.DLL. Referenced within this documentation as EEM API.\n
 *   TI recommends to use the Application Programming Interface (API) defined through
 *   MSP430_EEM.h to access the Enhanced Emulation Module (EEM) of an MSP430 device.
 *   For downward compatibility reasons to former releases of this DLL the functions
 *   declared in MSP430_Debug.h are still available but should UNDER NO CIRCUMSTANCES
 *   be mixed with the funtions of the newer EEM API.
 *   The DLL provides the following functionalities:\n
 *   - Caller notifying mechanism using callback notify mechanism
 *   - Autonom detection of state changes (e.g. breakpoint hit)\n
 * - and a third part to maintain the MSP-FET430UIF (TI USB FET) hardware and firmware.
 *   Defined through functions and definitions in MSP430_FET.h
*/


/**            
 \file MSP430.h
 
 \brief       This file contains the Application Programming Interface (API)
              to access an MSP430 microcontroller via JTAG using the MSP430.DLL.
              This file contains the DLL function headers and definitions.
 
              The MSP430.DLL provides the following functionalities:
              - Initialize a selected JTAG interface (FET)
              - Setting target's supply voltage
              - Identifing a device
              - Programming/Erasing the identified device
              - Program/Erase check of device memory
  
 \par         Project:
              MSP430 JTAG Interface (MSP430.dll)
 
 \par         Developed using:
              MS Visual C++ 2003/2010
 
 \version     3.02.05.004
  
 \par         Supported API calls:
              - MSP430_GetNumberOfUsbIfs()
              - MSP430_GetNameOfUsbIf()
			  - MSP430_SET_SYSTEM_NOTIFY_CALLBACK()
              - MSP430_Initialize()
              - MSP430_Close()
              - MSP430_GetJtagID()
			  - MSP430_GetFoundDevice()
			  - MSP430_OpenDevice()
			  - MSP430_Identify()
              - MSP430_Device()
              - MSP430_Configure()
              - MSP430_VCC()
              - MSP430_GetCurVCCT()
              - MSP430_GetExtVoltage()
              - MSP430_Reset()
              - MSP430_Erase()
              - MSP430_Memory()
              - MSP430_Secure()
              - MSP430_ReadOutFile()
              - MSP430_ProgramFile()
              - MSP430_VerifyFile()
              - MSP430_VerifyMem()
              - MSP430_EraseCheck()
              - MSP430_Error_Number()
              - MSP430_Error_String()

 \par         Version History:
              - Version 3.2.00.007  -  02.11.2010
			  - Version 3.2.01.009  -  10.18.2011

*/


#ifndef MSP430_H
#define MSP430_H

#include <stdint.h>
#include <DLL430_SYMBOL.h>

#if defined(_WIN32) || defined(_WIN64)
#include <windows.h>
#include <stddef.h>
#else
#include "wtypes.h"
#endif

#if defined(__cplusplus)
extern "C" {
#endif

#ifndef MSP430_TYPES
#define MSP430_TYPES

/// this is the definition for the DLL functions return value
typedef LONG STATUS_T;

/**
 \brief  Status codes of the DLL functions.
*/
typedef enum STATUS_CODE {
    STATUS_ERROR = -1, /**< DLL functions return this value on failure */
    STATUS_OK = 0, /**< this value is returned on success */
} STATUS_CODE_t;

/**
 \brief  Device information structure.
*/
#pragma pack(1)
typedef union DEVICE_T {
   /// this buffer holds the complete device information
   /// and is overlayed by the following information structure
    CHAR buffer[112];
    struct {  // actually 108 Bytes
      /// The value 0xaa55.
        WORD  endian;        
      /// Identification number.
        WORD  id;            
      /// Identification string.
        BYTE  string[32];    
      /// MAIN MEMORY starting address.
        WORD  mainStart;     
      /// INFORMATION MEMORY starting address.
        WORD  infoStart;     
      /// RAM ending address.
        WORD  ramEnd;        
      /// Number of breakpoints.
        WORD  nBreakpoints;  
      /// Emulation level.
        WORD  emulation;     
      /// Clock control level.
        WORD  clockControl;  
      /// LCD starting address.
        WORD  lcdStart;      
      /// LCD ending address.
        WORD  lcdEnd;        
      /// Vcc minimum during operation [mVolts].
        WORD  vccMinOp;      
      /// Vcc maximum during operation [mVolts].
        WORD  vccMaxOp;      
      /// Device has TEST/VPP.
        WORD  hasTestVpp;    
      /// RAM starting address.
        WORD  ramStart;      
      /// RAM2 starting address.
        WORD  ram2Start;     
      /// RAM2 ending address.
        WORD  ram2End;       
      /// INFO ending address.
        WORD  infoEnd;       
      /// MAIN ending address.
        ULONG mainEnd;       
      /// BSL starting  address.
        WORD  bslStart;       
      /// BSL ending address.
        WORD  bslEnd;       
      /// Number of CPU Register Trigger.
        WORD  nRegTrigger;  
      /// Number of EEM Trigger Combinations.
        WORD  nCombinations;  
      /// The MSP430 architecture (non-X, X or Xv2).
        BYTE  cpuArch;  
      /// The JTAG ID - value returned on an instruction shift.
        BYTE  jtagId;  
      /// The CoreIP ID.
        WORD  coreIpId;  
      /// The Device-ID Pointer.
        ULONG deviceIdPtr;  
      /// The EEM Version Number.
        WORD  eemVersion;  
      ///  Breakpoint Modes
        WORD nBreakpointsOptions;
        WORD nBreakpointsReadWrite;
        WORD nBreakpointsDma;
      /// Trigger Mask for Breakpoint 
        WORD TrigerMask;    
      /// Register Trigger modes
        WORD nRegTriggerOperations;
      /// MSP430 has Stage Storage
        WORD nStateStorage;
      /// Number of cycle counters of MSP430
        WORD nCycleCounter;
      /// Cycle couter modes
        WORD nCycleCounterOperations;
      /// Msp430 has Sequencer
        WORD nSequencer;		
      /// Msp430 has FRAM Memory
        WORD HasFramMemroy;
    };
} DEVICE_T_t;
#pragma pack()


/// Device id for unknown device
#define DEVICE_UNKNOWN   0

typedef enum CPU_ARCH_TYPE
{
    CPU_ARCH_ORIGINAL,
    CPU_ARCH_X,
    CPU_ARCH_XV2,
} CPU_ARCH_TYPE_t;

typedef enum READ_WRITE {
	WRITE = 0,
	READ = 1,
} READ_WRITE_t;

typedef enum ENABLE_DISABLE {
	DISABLE = 0,
	ENABLE = 1,
} ENABLE_DISABLE_t;

/// Device reset methods.
typedef enum RESET_METHOD {
	PUC_RESET   = (1 << 0), /**< Power up clear (i.e., a "soft") reset */
	RST_RESET   = (1 << 1), /**< RST/NMI (i.e., "hard") reset */
	VCC_RESET   = (1 << 2), /**< Cycle Vcc (i.e., a "power on") reset */
	FORCE_RESET = (1 << 3),

	/// combines all possible reset methods enumerated in enum RESET_METHOD
	ALL_RESETS = (PUC_RESET | RST_RESET | VCC_RESET),
	/// forces a Power up clear reset
	FORCE_PUC_RESET = (FORCE_RESET | PUC_RESET),
	/// forces a RST/NMI clear reset
	/// Non-Xv2 devices will be running and executing code after the reset
	FORCE_RST_RESET = (FORCE_RESET | RST_RESET),
	/// forces a Vcc clear reset
	/// Non-Xv2 devices will be running and executing code after the reset
	FORCE_VCC_RESET = (FORCE_RESET | VCC_RESET),
} RESET_METHOD_t;


/// FLASH erase type.
typedef enum ERASE_TYPE {
	ERASE_SEGMENT = 0, /**< Erase a segment */
	ERASE_MAIN = 1, /**< Erase all MAIN memory */
	ERASE_ALL = 2, /**< Erase all MAIN and INFORMATION memory not including IP protected area */
	ERASE_TOTAL = 3,/**< Erase all MAIN and INFORMATION memory including IP protected area */
} ERASE_TYPE_t;

/// Configurations to set with MSP430_Configure.
typedef enum CONFIG_MODE {
	VERIFICATION_MODE = 0, /**< Verify data downloaded to FLASH memories */
	EMULATION_MODE = 1, /**< 4xx emulation mode */
	LOCKED_FLASH_ACCESS = 5, /**< Allows Locked Info Mem Segment A access (if set to '1') */
	EDT_TRACE_MODE = 7, /**< Trace mode in EDT file format */
	INTERFACE_MODE = 8, /**< Configure interface protocol: JTAG or Spy-bi-Wire (see enum INTERFACE_TYPE) */

	/// Configure a value that will be placed on the devices' MemoryDataBus
	/// right before the device gets released from JTAG.
	/// Used for Software Breakpoints.
	SET_MDB_BEFORE_RUN = 9,

	/// Configure whether RAM content should be preserved/restored
	/// in MSP430_Erase() and MSP430_Memory() or not.
	/// RAM_PRESERVE_MODE is set to ENABLE by default.
	/// Usage Example for initial flash programming:
	/// (1) MSP430_Configure(RAM_PRESERVE_MODE, DISABLE);
	/// (2) MSP430_Erase(ERASE_ALL,..,..);
	/// (3) MSP430_Memory(..., ..., ..., WRITE );
	/// (4) MSP430_Memory(..., ..., ..., READ );
	/// ..... Flash Programming/Download finished
	/// (n) MSP430_Configure(RAM_PRESERVE_MODE, ENABLE);
    RAM_PRESERVE_MODE = 10,
   /// Configure the DLL to allow read/write/erase access to the 5xx
   /// Bootstrap Loader (BSL) memory segments.
    UNLOCK_BSL_MODE =11,
    // just used internal for the device code of L092 and C092
    DEVICE_CODE = 12,
    // set true to write the external SPI image of the L092
    WRITE_EXTERNAL_MEMORY = 13,
    // set DEBUG_LPM_X true to start debugging of LPMx.5
	// this will start polling for LPMX.5 events if a system notify callback was 
	// previously set using MSP430_SET_SYSTEM_NOTIFY_CALLBACK()
    DEBUG_LPM_X = 14,
    // Configure JTAG speed
    JTAG_SPEED = 15,
	// total device erase including IP protection 
	TOTAL_ERASE_DEVICE = 16

} CONFIG_MODE_t;

/// Configurations values for CONFIG_MODE INTERFACE_MODE
typedef enum INTERFACE_TYPE {
	JTAG_IF = 0, /**< 4 Wire JTAG protocol used */
	SPYBIWIRE_IF = 1, /**< 2 Wire (Spy-bi-wire) JTAG protocol used */
	SPYBIWIREJTAG_IF = 2, /**< 2 Wire Devices accessed by 4wire JTAG protocol */
	AUTOMATIC_IF = 3 /**< Protocol will be detected automatically */
} INTERFACE_TYPE_t;


/// File types.
typedef enum FILE_TYPE {
	FILETYPE_AUTO, /**< Auto detect */
	FILETYPE_TI_TXT, /**< TI text */
	FILETYPE_INTEL_HEX, /**< Intel hex */
} FILE_TYPE_t;

#define ERROR_DEFINITIONS \
	ERROR_DEF(NO_ERR, "No error") \
	ERROR_DEF(INITIALIZE_ERR, "Could not initialize device interface") \
	ERROR_DEF(CLOSE_ERR, "Could not close device interface") \
	ERROR_DEF(PARAMETER_ERR, "Invalid parameter(s)") \
	ERROR_DEF(NO_DEVICE_ERR, "Could not find device (or device not supported)") \
	ERROR_DEF(DEVICE_UNKNOWN_ERR, "Unknown device") \
	ERROR_DEF(READ_MEMORY_ERR, "Could not read device memory") \
	ERROR_DEF(WRITE_MEMORY_ERR, "Could not write device memory") \
	ERROR_DEF(READ_FUSES_ERR, "Could not read device configuration fuses") \
	ERROR_DEF(CONFIGURATION_ERR, "Incorrectly configured device; device derivative not supported") \
	ERROR_DEF(VCC_ERR, "Could not set device Vcc") \
	ERROR_DEF(RESET_ERR, "Could not reset device") \
	ERROR_DEF(PRESERVE_RESTORE_ERR, "Could not preserve/restore device memory") \
	ERROR_DEF(FREQUENCY_ERR, "Could not set device operating frequency") \
	ERROR_DEF(ERASE_ERR, "Could not erase device memory") \
	ERROR_DEF(BREAKPOINT_ERR, "Could not set device breakpoint") \
	ERROR_DEF(STEP_ERR, "Could not single step device") \
	ERROR_DEF(RUN_ERR, "Could not run device (to breakpoint)") \
	ERROR_DEF(STATE_ERR, "Could not determine device state") \
	ERROR_DEF(EEM_OPEN_ERR, "Could not open Enhanced Emulation Module") \
	ERROR_DEF(EEM_READ_ERR, "Could not read Enhanced Emulation Module register") \
	ERROR_DEF(EEM_WRITE_ERR, "Could not write Enhanced Emulation Module register") \
	ERROR_DEF(EEM_CLOSE_ERR, "Could not close Enhanced Emulation Module") \
	ERROR_DEF(FILE_OPEN_ERR, "File open error") \
	ERROR_DEF(FILE_DETECT_ERR, "File type could not be identified") \
	ERROR_DEF(FILE_END_ERR, "File end error") \
	ERROR_DEF(FILE_IO_ERR, "File input/output error") \
	ERROR_DEF(FILE_DATA_ERR, "File data error") \
	ERROR_DEF(VERIFY_ERR, "Verification error") \
	ERROR_DEF(BLOW_FUSE_ERR, "Could not blow device security fuse") \
	ERROR_DEF(FUSE_BLOWN_ERR, "Security Fuse has been blown") \
	ERROR_DEF(INTEL_HEX_CODE_ERR, "Error within Intel Hex file") \
	ERROR_DEF(WRITE_REGISTER_ERR, "Could not write device Register") \
	ERROR_DEF(READ_REGISTER_ERR, "Could not read device Register") \
	ERROR_DEF(INTERFACE_SUPPORT_ERR, "Not supported by selected Interface or Interface is not initialized") \
	ERROR_DEF(COMM_ERR, "Interface Communication error") \
	ERROR_DEF(NO_EX_POWER, "No external power supply detected") \
	ERROR_DEF(LOW_EX_POWER, "External power too low") \
	ERROR_DEF(EX_POWER_OK, "External power detected") \
	ERROR_DEF(HIGH_EX_POWER, "External power too high") \
	ERROR_DEF(SELFTEST_ERR, "Hardware Self Test Error") \
	ERROR_DEF(FLASH_TIMEOUT_ERR, "Fast Flash Routine experienced a timeout") \
	ERROR_DEF(THREAD_ERR, "Could not create thread for polling") \
	ERROR_DEF(EEM_INIT_ERR, "Could not initialize Enhanced Emulation Module") \
	ERROR_DEF(RESOURCE_ERR, "Insufficent resources") \
	ERROR_DEF(CLK_CTRL_ERR, "No clock control emulation on connected device") \
	ERROR_DEF(STATE_STOR_ERR, "No state storage buffer implemented on connected device") \
	ERROR_DEF(READ_TRACE_ERR, "Could not read trace buffer") \
	ERROR_DEF(VAR_WATCH_EN_ERR, "Enable the variable watch function") \
	ERROR_DEF(SEQUENCER_ERR, "No trigger sequencer implemented on connected device") \
	ERROR_DEF(SEQ_ENABLE_ERR, "Could not read sequencer state - Sequencer is disabled") \
	ERROR_DEF(CLR_SEQ_TRIGGER, "Could not remove trigger - Used in sequencer") \
	ERROR_DEF(SET_SEQ_TRIGGER, "Could not set combination - Trigger is used in sequencer") \
	ERROR_DEF(SPMA_ACTIVE_ERR, "System Protection Module A is enabled - Device locked") \
	ERROR_DEF(SPMA_INVALID_KEY_ERR, "Invalid SPMA key was passed to the target device - Device locked") \
	ERROR_DEF(SPMA_MAX_TRIALS, "Device does not accept any further SPMA keys - Device locked") \
	ERROR_DEF(USB_FET_BSL_ACTIVE_ERR, "MSP-FET430UIF Firmware erased - Bootloader active") \
	ERROR_DEF(USB_FET_NOT_FOUND_ERR, "Could not find MSP-FET430UIF on specified COM port") \
	ERROR_DEF(USB_FET_BUSY_ERR, "MSP-FET430UIF is already in use") \
    ERROR_DEF(THREAD_ACTIVE_ERR, "EEM polling thread is already active") \
    ERROR_DEF(THREAD_TERMINATE_ERR, "Could not terminate EEM polling thread") \
    ERROR_DEF(UNLOCK_BSL_ERR, "Could not unlock BSL memory segments") \
    ERROR_DEF(BSL_MEMORY_LOCKED_ERR, "Could not perform access, BSL memory segments are protected") \
    ERROR_DEF(FOUND_OTHER_DEVICE, "Another device as selected was found") \
    ERROR_DEF(WRONG_PASSWORD, "Could not enable JTAG wrong password") \
    ERROR_DEF(UPDATE_MULTIPLE_UIF_ERR, "Only one UIF must be connected during update to v3") \
    ERROR_DEF(CDC_UIF_ERR, "CDC-USB-FET-Driver was not installed. Please install the driver") \
    ERROR_DEF(UIF_MANUAL_POWERCYCLE_NEEDED, "Manual reboot of USB-FET needed ! PLEASE unplug and reconnect your USB-FET!!") \
    ERROR_DEF(INTERNAL_ERR, "Internal error") \
    ERROR_DEF(INVALID_ERR, "Invalid error number")

/// Error codes
#define ERROR_DEF(errorEnum, errorString) errorEnum,
typedef enum ERROR_CODE { ERROR_DEFINITIONS } ERROR_CODE_t;
#undef ERROR_DEF

// System events
typedef enum SYSTEM_EVENT_MSP
{
	/// System event FET connection is lost
    FET_CONNECTION_LOST,
	/// System event device connection is lost
    DEVICE_CONNECTION_LOST,
	/// System event FET restart needed
    FET_RESTART_NEEDED,
	/// System event device entered LPMx.5
    DEVICE_IN_LPM5_MODE,
	/// System event devices wakes up from LPMx.5
    DEVICE_WAKEUP_LPM5_MODE,           
} SYSTEM_EVENT_MSP_t;

typedef void (* SYSTEM_NOTIFY_CALLBACK) (SYSTEM_EVENT_MSP_t MySystemEvent);

#endif /* MSP430_TYPES */

// Functions. -----------------------------------------------------------------
/**
\fn			STATUS_T WINAPI MSP430_SET_SYSTEM_NOTIFY_CALLBACK(SYSTEM_NOTIFY_CALLBACK parSystemNotifyCallback);

\brief		Initialize the SYSTEM NOTIFYCALLBACK.
\note		If DEBUG_LPM_X has been previously configured, this will start polling for LPMX.5 events.
\note		This function should be called after MSP430_OpenDevice() function.

\param		parSystemNotifyCallback: To initialize the system notify callback an instance of the 
			enum SYSTEM_EVENT_MSP must be provided to the set function.		

\return		STATUS_OK:    The callback was initialized successfully.
\n			STATUS_ERROR: The callback was not initialized successfully.

\par		Error codes:
			INITIALIZE_ERR
*/
STATUS_T WINAPI MSP430_SET_SYSTEM_NOTIFY_CALLBACK(SYSTEM_NOTIFY_CALLBACK parSystemNotifyCallback);


/**
\fn    STATUS_T MSP430_Initialize(CHAR* port, LONG* version);

\brief   Initialize the interface.

\note    1. This function must be called first.
\note    2. MSP430_VCC() must be called second (after MSP430_Initialize() is called).
\note    3. When initializing the MSP-FET430UIF (TI USB FET) parameter 'version' could
            contain the value -1 or -3. This means that the Dll and the MSP-FET430UIF do not
			have the same version (-3 means a major internal update is required).
			MSP430_FET_FwUpdate() should be called.
			MSP430_FET_FwUpdate() is part of the Maintenance API of the Dll.
			When -3 was returned and calling MSP430_FET_FwUpdate(), the file CDC.log must exist
			in the directory of the executed binary and the content must be the string "True" 
			without a newline. This file signals that a CDC driver is installed and prevents
			the update from making the MSP-FET430UIF unusable.
 
\param   port:    Interface port reference (application specific).
                  - To initialize a MSP-FET430PIF LPT Jtag adapter the parameter port should point to
                    a string which respresents the corresponding LPT port 
                    (e.g. '1','2',... or 'LPT1','LPT2',...).
                  - To initialize TI's MSP-FET430UIF USB Jtag adapter the parameter
                    port should point to the string 'TIUSB' or just 'USB'.
                  - TI's MSP-FET430UIF USB Jtag adapter create Virtual Com Ports (VCPs)
                    on the PC system (see Device Manager). It is also possible to directly
                    pass the name of a dedicated VCP via this parameter (e.g. 'COM4, COM23,...).
                    This can be used to support multiple MSP-FET430UIF interfaces on one PC.
                    The later generation of USB development tools (eZ430-RF2500) no longer
                    uses the VCP approach. These tools enumerate as Human Interface Devices (HID)
                    on the USB. Since DLL version 2.03.00.000 it is also possible to directly 
                    pass the name of a dedicated HID via this parameter.
					When using a v3 MSP-FET430UIF it is enumerated as Communication Device Class (CDC).
                    Refer to MSP430_GetNumberOfUsbIfs() and MSP430_GetNameOfUsbIf()
                    for more information on VCP, HID and CDC.
\param   version: The version number of the MSP430 DLL is returned (if version is not NULL).
                  A value of -1 or -3 reports a version conflict between the Dll and USB FET f/w.
                  In that case please refer to MSP430_FET_FwUpdate() on how to update
                  the firmware of the MSP-FET430UIF.

\return  STATUS_OK:    The interface was initialized.
\n       STATUS_ERROR: The interface was not initialized.

\par     Error codes:
         INITIALIZE_ERR
\n       USB_FET_NOT_FOUND_ERR
\n       USB_FET_BUSY_ERR
\n		 COMM_ERR
*/
DLL430_SYMBOL STATUS_T WINAPI MSP430_Initialize(CHAR* port, LONG* version);

/**
\fn    STATUS_T MSP430_Close(LONG vccOff);

\brief   Close the interface.

\note    1. If called, this function must be called last.
 
\param   vccOff: Turn off the device Vcc (0 volts) if TRUE.

\return  STATUS_OK:    The interface was closed.
\n       STATUS_ERROR: The interface was not closed.

\par     Error codes:
         CLOSE_ERR
\n		 VCC_ERR
*/
DLL430_SYMBOL STATUS_T WINAPI MSP430_Close(LONG vccOff);


/**
\fn    STATUS_T MSP430_GetJtagID(LONG* JtagId);

\brief   Return JTAG Id.

\note    1. MSP430_Initialize() must be called prior to this function.
 
\param   JtagId: JTAG Id of the connected device will be returned.

\return  STATUS_OK:    JTAG Id could be read.
\n       STATUS_ERROR: JTAG Id could NOT be read.

\par     Error codes:
*/
DLL430_SYMBOL STATUS_T WINAPI MSP430_GetJtagID(LONG* JtagId);

/**
\fn		 STATUS_T WINAPI MSP430_GetFoundDevice(CHAR* FoundDevice, LONG count);

\brief   Return an instance of the Found device.

\note    It is possible to successfully (i.e. return status: STATUS_OK) and an "UNKNOWN DEVICE".
 
\param   FoundDevice: A pointer to a buffer where the device identity and information are stored. See DEVICE_T for details.

\param   count: The number of bytes to return in the buffer (i.e., the buffer size). Must be at least sizeof(DEVICE_T).

\return  STATUS_OK:    The device information was obtained.
\n       STATUS_ERROR: The device information was not obtained.

\par     Error codes:
         To be defined
*/
STATUS_T WINAPI MSP430_GetFoundDevice(CHAR* FoundDevice, LONG count);

/**
\fn    STATUS_T WINAPI MSP430_OpenDevice(CHAR* Device,CHAR* Password, LONG PwLength,LONG DeviceCode, LONG setId);

\brief   Identify the device, and compare the found device to the expected device.

\note	 1. MSP430_Initialize() and MSP430_VCC() must have been called prior to calling this function.
\note    2. It is possible to successfully (i.e. return status: STATUS_OK) identify another device as set"FOUND_OTHER_DEVICE".
\note    3. Use the setId parameter and MSP430_State(state, TRUE, FALSE) to stop a running device (without resetting it).
\note    4. Following MSP430_OpenDevice():
            - the device is reset, is not executing, and is under JTAG control.
            - the JTAG interface is enabled and the JTAG signals are negated.
 

\param	 Device:		Device, which should be initialized
\param   Password:		JTAG password to access JTAG 
\param   PwLength:		Password length in WORDS 
\param	 DeviceCode:	Activation code for devices. Example: L092 or C092
\param   setId:			If setId is not DEVICE_UNKNOWN, the device is set to setId. Otherwise the device is determined.

\return  STATUS_OK:    Target device was connected and found in the database.
\n       STATUS_ERROR: The device was not obtained.

\par     Error codes:
         PARAMETER_ERR
\n       NO_DEVICE_ERR
\n		 DEVICE_UNKNOWN_ERR
\n       READ_MEMORY_ERR
\n       READ_FUSES_ERR
\n		 FUSE_BLOWN_ERR
\n       CONFIGURATION_ERR
\n		 FOUND_OTHER_DEVICE
\n		 WRONG_PASSWORD
*/
STATUS_T WINAPI MSP430_OpenDevice(CHAR* Device,CHAR* Password, LONG PwLength,LONG DeviceCode, LONG setId);


/**
\fn    STATUS_T MSP430_Device(LONG localDeviceId, CHAR* buffer, LONG count);

\brief   Obtain the device information.

\note    1. This function can be used to determine the devices supported by the driver; a device
            identification of NUMofDevices is returned to indicate that there are no additional devices.
 
\param   localDeviceId: An index (from zero) used to specify the device.
\param   buffer: A pointer to a buffer where the device identity and information is stored. See DEVICE_T for details.
\param   count: The number of bytes to return in the buffer (i.e., the buffer size). Must be at least sizeof(DEVICE_T)

\return  STATUS_OK:    The device identify and information was obtained.
\n       STATUS_ERROR: The device identity and information was not obtained.

\par     Error codes:
         PARAMETER_ERR
*/
DLL430_SYMBOL STATUS_T WINAPI MSP430_Device(LONG localDeviceId, CHAR* buffer, LONG count);

/**
\fn    STATUS_T MSP430_Configure(LONG mode, LONG value);

\brief   Configure the mode(s) of the device and/or the software.
         Possible configuration modes are covered by the enum CONFIG_MODE.
		 See notes for details of different modes to be configured.

\note    1. VERIFICATION_MODE
            value = TRUE : Verify data downloaded to FLASH memories.
			value = FALSE: No verification.
         2. EMULATION_MODE
		    Configure F4xx emulation mode according the set value.
         3. CLK_CNTRL_MODE
			Configure the EEM Clock control register value.
			Do NOT use after calling MSP430_Eem_Init().
         4. MCLK_CNTRL_MODE
            Configure the EEM Module Clock control register value.
			Do NOT use after calling MSP430_Eem_Init().
         5. LOCKED_FLASH_ACCESS
            value = TRUE : Allows Locked Info Mem Segment A erase/write access.
			value = FALSE: Flash Segment stays locked during erase/write operations.
         6. EDT_TRACE_MODE
            Can be used to trace JTAG commands in EDT file format.
			Only applicable with LPT tool.
            value = TRUE : Trace ON
			value = FALSE: Trace OFF
         7. INTERFACE_MODE
            Configure JTAG interface protocol: JTAG or Spy-bi-Wire (see enum INTERFACE_TYPE)
         8. SET_MDB_BEFORE_RUN
            Configure a value that will be placed on the devices' MemoryDataBus
            right before the device gets released from JTAG.
            Used for Software Breakpoints.
         9. RAM_PRESERVE_MODE
            Configure whether RAM content should be preserved/restored
            in MSP430_Erase() and MSP430_Memory() or not.
            RAM_PRESERVE_MODE is set to ENABLE by default.

\param   mode:     Mode (enum CONFIG_MODE) to be configured. See notes above for details.
\param   value:    Mode value.

\return  STATUS_OK:    Mode was configured.
\n       STATUS_ERROR: Mode was not configured.

\par     Error codes:
         PARAMETER_ERR
 \n		 UNLOCK_BSL_ERR
 \n		 INTERFACE_SUPPORT_ERR
*/
DLL430_SYMBOL STATUS_T WINAPI MSP430_Configure(LONG mode, LONG value);

/**
\fn    STATUS_T MSP430_VCC(LONG voltage);

\brief   Set the device Vcc pin to voltage/1000 volts.\n
         The USB JTAG adaptor also checks if external power is supplied to the traget device. 
		 If an external voltage is detected MSP430_VCC() returns STATUS_ERROR and one of the 
		 corresponding error codes.

\note    1. A "voltage" of zero (0) turns off voltage to the device.
\note    2. This function must be called second (after MSP430_Initialize() is called).
 
\param   voltage: The device Vcc pin is set to voltage/1000 volts.

\return  STATUS_OK:    The Vcc was set to voltage.
\n       STATUS_ERROR: The Vcc was not set to voltage.

\par     Error codes:
         PARAMETER_ERR
\n       VCC_ERR
\n       LOW_EX_POWER           
\n       EX_POWER_OK           
\n       HIGH_EX_POWER
*/
DLL430_SYMBOL STATUS_T WINAPI MSP430_VCC(LONG voltage);


/**
\fn   STATUS_T MSP430_GetCurVCCT(LONG* voltage);

\brief   Report back the current voltage supplied to the target device.

\note    1. This function is only supported by the MSP-FET430UIF USB JTAG adaptor.
         2. Beeing called when using Parallel Port tool the function returns
            a dummmy value of 3000 in case MSP430_VCC() has been called prior.

\param   voltage: The current voltage supplied to the device.

\return  STATUS_OK:    The current voltage is reported back.
\n       STATUS_ERROR: The current voltage can not be reported back.

\par     Error codes:
         COMM_ERR
*/
DLL430_SYMBOL STATUS_T WINAPI MSP430_GetCurVCCT(LONG* voltage);

/**
\fn   STATUS_T MSP430_GetExtVoltage(LONG* voltage, LONG* state);

\brief   Detect if external voltage is supplied to the target device.
         Report back the value of an external voltage as well as an corresponding state.

\note    1. This function is only supported by the MSP-FET430UIF USB JTAG adaptor.
         2. Beeing called when using Parallel Port tool the function returns
            a dummmy voltage value of 0 and NO_EX_POWER as the state value.
            Prallel Port tool is not be able to measure external power supply!

\param   voltage: The current external voltage.
\param   state:	  External voltage can have one of the following states:
                  - NO_EX_POWER, no external power was detected 
                  - LOW_EX_POWER, external power is too low
                  - EX_POWER_OK, external power detected
                  - HIGH_EX_POWER, external power too high

\return  STATUS_OK:    The function was executed successfully.
\n       STATUS_ERROR: The function was not executed successfully.

\par     Error codes:
         COMM_ERR
*/
DLL430_SYMBOL STATUS_T WINAPI MSP430_GetExtVoltage(LONG* voltage, LONG* state);


/**
\fn    STATUS_T MSP430_Reset(LONG method, LONG execute, LONG releaseJTAG);

\brief   Reset the device using the specified method(s). Optionally start device execution,
         and release the JTAG control signals.

\note    1. It is possible to combine reset methods. The methods are applied in the following order:
            PUC then RST_NMI then VCC. If a reset operation fails, the next reset method (if any) is applied.
\note    2. Following reset by RST/NMI and/or VCC, a PUC is automatically executed to reset the device in
            such a way that it does not begin to execute a resident program (or garbage).
\note    3. The device registers are updated (for read using MSP430_Registers()) following MSP430_Reset().

\param   method:        The bit mask specifying the method(s) to use to reset the device:
           - PUC_RESET: The device is reset using PUC (i.e., a "soft" reset).
           - RST_RESET: The device is reset using the RST/NMI pin (i.e., a "hard" reset).
           - VCC_RESET: The device is reset by cycling power to the device.
           - FORCE_PUC_RESET, FORCE_RST_RESET, FORCE_VCC_RESET

\param   execute:     Start device execution (when TRUE).
\param   releaseJTAG: Release the JTAG control signals (when TRUE). execute must be TRUE.

\return  STATUS_OK:    The device was reset (and optionally started [and JTAG control released])
\n       STATUS_ERROR: The device was not reset (and optionally started [and JTAG control released]).

\par     Error codes:
         PARAMETER_ERR
\n       RESET_ERR
\n       RUN_ERR
\n       NO_DEVICE_ERR
*/
DLL430_SYMBOL STATUS_T WINAPI MSP430_Reset(LONG method, LONG execute, LONG releaseJTAG);

/**
\fn    STATUS_T MSP430_Erase(LONG type, LONG address, LONG length);

\brief   Erase the device FLASH/FRAM memory.

\note    1. MSP430_OpenDevice() must have been called prior to calling this function.
\note    2. Use MSP430_Read_Memory() to determine the address of a failed erase operation.
\note    3. If address+length extends beyond the segment containing address, intermediate segments are erased
            (and checked).
\note    4. Following erasure of the memory, the device oscillator will be set to approx. 2.1MHz.
\note    5. Some device families have a locked INFO memory segment (INFO A).
            MSP430_Configure(LOCKED_FLASH_ACCESS,...) can be used to unlock/lock this segment.
            Having parameter 'type' set to ERASE_ALL will erase the complete INFO memory in case
			the INFO A segment was unlocked prior.
			Otherwise only INFO segments B, C and D will be erased.
\note    6. Some device families have a lockable BSL memory segment.
            MSP430_Configure(UNLOCK_BSL_MODE,...) can be used to unlock/lock this segment.
            Having parameter 'type' set to ERASE_ALL will erase the complete BSL memory in case
			the BSL memory was unlocked prior.
\note	 7. Devices using FRAM memory can be erased even though it is technically not necessary.
			Doing so will fill the erased memory with 0xFFFF to be consistent with FLASH devices.

\param   type:      The type parameter specifies what should be erased.
           - ERASE_SEGMENT: Erase the segment containing 'address'.
           - ERASE_MAIN:    Erase the Main memory.
           - ERASE_ALL:     Erase the Main, Information and Bootstrap Loader memory.

\param   address:   Starting address of erase check operation. Must be word aligned.
\param   length:    Length of erase check operation (even number of bytes).

\return  STATUS_OK:    The device FLASH/FRAM memory was erased. 
\n       STATUS_ERROR: The device FLASH/FRAM memory was not erased.

\par     Error codes:
         PARAMETER_ERR
\n       PRESERVE_RESTORE_ERR
\n       FREQUENCY_ERR
\n       ERASE_ERR
*/
DLL430_SYMBOL STATUS_T WINAPI MSP430_Erase(LONG type, LONG address, LONG length);

/**
\fn    STATUS_T MSP430_Memory(LONG address, CHAR* buffer, LONG count, LONG rw);

\brief   Read and write the device memory. "Device memory" includes the Special Function Registers (i.e., peripheral
         registers), RAM, Information (FLASH/FRAM) memory, Bootstrap Loader memory (BLS) and Main (FLASH/FRAM) memory.

\note    1. A data write operation can be verified by performing a data read operation followed by a comparison at
            the application level or by using MSP430_VerifyMem().
\note    2. Usually, FLASH memory must be erased before the write to FLASH memory operation is performed.
            *** The write to FLASH memory operation DOES NOT erase the FLASH before the operation is performed. ***
\note    3. Following a write to FLASH, the device oscillator will be set to approx. 2.1MHz.

\param   address: The starting address of the device memory to be read or written.
\param   buffer:  The buffer into which device memory is read, or from which device memory is written.
\param   count:   The number of bytes of device memory read or written.
\param   rw:      Specify a read (READ) or write (WRITE) operation.

\return  STATUS_OK:    The memory operation encountered no errors.
\n       STATUS_ERROR: The memory operation encountered errors.

\par     Error codes:
         PARAMETER_ERR
\n		 NO_DEVICE_ERR
\n       READ_MEMORY_ERR
\n       WRITE_MEMORY_ERR
\n		 BSL_MEMORY_LOCKED_ERR
\n       PRESERVE_RESTORE_ERR
\n       FREQUENCY_ERR
*/
DLL430_SYMBOL STATUS_T WINAPI MSP430_Memory(LONG address, CHAR* buffer, LONG count, LONG rw);

#define MSP430_Read_Memory(ADDRESS, BUFFER, COUNT) MSP430_Memory(ADDRESS, BUFFER, COUNT, READ)
#define MSP430_Write_Memory(ADDRESS, BUFFER, COUNT) MSP430_Memory(ADDRESS, BUFFER, COUNT, WRITE)

/**
\fn    STATUS_T MSP430_Secure(void);

\brief   The device is secured (i.e., the JTAG security fuse is blown).
\n       The MSP-FET430PIF LPT JTAG adaptor >>> DOES NOT <<< implement the functionality required to
         blow the device security fuse.
\n       MSP-FET430UIF USB JTAG adaptor >>> DOES <<< support this feature.

\note    1. MSP430_OpenDevice() must have been called prior to calling this function.
\note    2. Once a device is secure, further communications via JTAG is not possible.
\note    3. After MSP430_Secure(), the device is reset with Vcc set to ((MaxOp + MinOp) / 2) volts (*)
            and Vpp set to 0 volts (*). (*) Implementation dependent.

\return  STATUS_OK:    The device was secured.
\n       STATUS_ERROR: The device was not secured.

\par     Error codes:
         DEVICE_UNKNOWN_ERR
\n       BLOW_FUSE_ERR
*/
DLL430_SYMBOL STATUS_T WINAPI MSP430_Secure(void);

/**
\fn    STATUS_T MSP430_ReadOutFile(LONG wStart, LONG wLength, CHAR* lpszFileName, LONG iFileType);

\brief   Read the specified range of device memory, and write it to the specified file.

\note    
\param   wStart:       The starting address of the device memory to read.
\param   wLength:      The length of the device memory to read (even number of bytes).
\param   lpszFileName: Pointer to the filename into which the read device memory is written.
\param   iFileType:    The type of the file into which the read device memory is written:
                       - TI text: FILETYPE_TI_TXT
                       - Intel hex: FILETYPE_INTEL_HEX

\return  STATUS_OK:    The device memory was read and written to the specified file.
\n       STATUS_ERROR: The device memory was not read or written to the specified file.

\par     Error codes:
         PARAMETER_ERR
\n		 NO_DEVICE_ERR
\n       READ_MEMORY_ERR
\n       FILE_IO_ERR
\n		 INTERNAL_ERR
*/
#if ! defined(uController)
DLL430_SYMBOL STATUS_T WINAPI MSP430_ReadOutFile(LONG wStart, LONG wLength, CHAR* lpszFileName, LONG iFileType);
#endif

/**
\fn    STATUS_T MSP430_ProgramFile(CHAR* File, LONG eraseType, LONG verifyMem);

\brief   The contents of the specified file are input, and then written to the device.
         The device is optionally erased prior to being written. The device is optionally
         verified after being written.

\note    1. MSP430_Identify() must have been called prior to calling this function.    
\note    2. The file type must be one of TI text or Intel hex. 
\note    3. verifyMem operation does not read the device memory and compare it to the file.
            Instead, this operation computes a checksum for the device memory, and
            then computes a checksum for the file, and finally compares the two checksums.
\note    4. Following a write to FLASH/FRAM, the device oscillator will be set to approx. 2.1MHz.
\note    5. Following verifyMem, the device is reset.

\param   File:         Pointer to filename.
\param   eraseType:    Type of device erasure:
                        - ERASE_ALL: Erase all Main and Information memories.
                        - ERASE_MAIN: Erase all Main memories.
\param   verifyMem:    Verify the device if TRUE.

\return  STATUS_OK:    The file contents were input and were written to the device.
\n       STATUS_ERROR: The file contents were not input or were not written to the device.

\par     Error codes:
         DEVICE_UNKNOWN_ERR
\n		 NO_DEVICE_ERR
\n       PARAMETER_ERR
\n		 FILE_OPEN_ERR
\n       FILE_END_ERR
\n       FILE_DETECT_ERR
\n       FILE_IO_ERR
\n       FILE_DATA_ERR
\n       PRESERVE_RESTORE_ERR
\n       FREQUENCY_ERR
\n       ERASE_ERR
\n       READ_MEMORY_ERR
\n       WRITE_MEMORY_ERR
\n       VERIFY_ERR
\n       ERR_INTEL_HEX_CODE
*/
#if ! defined(uController)
DLL430_SYMBOL STATUS_T WINAPI MSP430_ProgramFile(CHAR* File, LONG eraseType, LONG verifyMem);
#endif

/**
\fn    STATUS_T MSP430_VerifyFile(CHAR* File);

\brief   Compare the MSP430 memory and the contents of the specified file.

\note    1. The file type must be one of TI text or Intel hex. 
\note    2. This function does not read the device memory and compare it to the file.
            Instead, this operation computes a checksum for the device memory, and
            then computes a checksum for the file, and finally compares the two checksums.
\note    3. The device is reset following this operation.

\param   File:         Pointer to filename.

\return  STATUS_OK:    The file contents were input and compare with the device memory.
\n       STATUS_ERROR: The file contents were not input or did not compare with the device memory.

\par     Error codes:
         PARAMETER_ERR
\n		 NO_DEVICE_ERR
\n		 FILE_OPEN_ERR
\n       FILE_END_ERR
\n       FILE_DETECT_ERR
\n       FILE_IO_ERR
\n       FILE_DATA_ERR
\n       VERIFY_ERR
\n       ERR_INTEL_HEX_CODE
*/
#if ! defined(uController)
DLL430_SYMBOL STATUS_T WINAPI MSP430_VerifyFile(CHAR* File);
#endif

/**
\fn    STATUS_T MSP430_VerifyMem(LONG StartAddr, LONG Length, CHAR *DataArray);

\brief   Compare the MSP430 memory and the specified data.

\note     1. This function does not read the specified memory region and compare it to the data.
             Instead, this function computes a checksum for the specified memory region, and
             then computes a checksum for the data, and finally compares the two checksums.
\note     2. The device is reset following this operation.

\param   StartAddr:    Start address of memory to be compared (must be even).
\param   Length:       Number of bytes to be compared (must be even).
\param   *DataArray:   Pointer to data array.

\return  STATUS_OK:    The device memory and data match.
\n       STATUS_ERROR: The device memory and data do not match.

\par     Error codes:
         PARAMETER_ERR
\n		 NO_DEVICE_ERR
\n       VERIFY_ERR
*/
DLL430_SYMBOL STATUS_T WINAPI MSP430_VerifyMem(LONG StartAddr, LONG Length, CHAR* DataArray);

/**
\fn    STATUS_T MSP430_EraseCheck(LONG StartAddr, LONG Length);

\brief   Verify that the specified memory range is erased.

\note    1. This function does not read the specified memory region and compare it to erased data.
            Instead, this function computes a checksum for the specified memory region, and
            then computes a checksum for erased data, and finally compares the two checksums.
\note    2. The device is reset following this operation.

\param   StartAddr:    Start address of memory to be verified (must be even).
\param   Length:       Number of BYTEs to be verified (must be even).

\return  STATUS_OK:    The device memory in the specified range is erased.
\n       STATUS_ERROR: The device memory in the specified range is not erased.

\par     Error codes:
         PARAMETER_ERR
\n		 NO_DEVICE_ERR
\n       VERIFY_ERR
*/
DLL430_SYMBOL STATUS_T WINAPI MSP430_EraseCheck(LONG StartAddr, LONG Length);

/**
\fn    STATUS_T MSP430_Error_Number(void);

\brief   Determine the number of the error when a MSP430_xxx() function returns STATUS_ERROR.

\note    1. The error number is reset (to NO_ERR) after the error number is returned.

\return  The number of the last error.

*/
DLL430_SYMBOL LONG WINAPI MSP430_Error_Number(void);

/**
\fn    const CHAR* MSP430_Error_String(LONG errorNumber);

\brief   Determine the string associated with errorNumber.

\param   errorNumber: Error number.

\return  The string associated with errorNumber.

*/
#if ! defined(uController)
DLL430_SYMBOL const CHAR* WINAPI MSP430_Error_String(LONG errorNumber);
#endif

/**
\fn    STATUS_T MSP430_GetNumberOfUsbIfs(LONG* Number);

\brief   Returns the number of MSP-FET430UIF USB FETs connected to the PC system.

\param   Number:   Return parameter. Number of MSP-FET430UIFs connected.

\return  STATUS_OK:    USB was sucessfully scanned for connected MSP-FET430UIFs.
\n       STATUS_ERROR: Could not scan USB for connected MSP-FET430UIFs.

\par     Error codes:
         INITIALIZE_ERR
\n       PARAMETER_ERR
*/
#if ! defined(uController)
DLL430_SYMBOL STATUS_T WINAPI MSP430_GetNumberOfUsbIfs(LONG* Number);
#endif

/**
\fn    STATUS_T MSP430_GetNameOfUsbIf(LONG Idx, CHAR** Name, LONG* Status);

\brief   Get the name of a Virtual Com Port (VCP), Human Interface Device (HID) 
		 or Communication Device Class (CDC) device index assigned to a 
		 certain MSP-FET430UIF USB FET.
         Also the status of the MSP-FET430UIF is returned (ENABLE/DISABLE).

\note    1. MSP430_GetNumberOfUsbIfs() must be called prior to this function.

\param   Idx:    Zero based index to the name of a certain MSP-FET430UIF.
                 Must be a value between 0 and 'number' returned
                 by MSP430_GetNumberOfUsbIfs();
\param   Name:   Pointer to the MSP-FET430UIF's name string.
\param   Status: Status of the MSP-FET430UIF,
                 - ENABLE  (The MSP-FET430UIF is already used by another debugger instance)
                 - DISABLE (The MSP-FET430UIF is free to be used)

\return  STATUS_OK:    MSP-FET430UIF information was returned.
\n       STATUS_ERROR: MSP-FET430UIF information was NOT returned.

\par     Error codes:
         INITIALIZE_ERR
\n       PARAMETER_ERR
\n		 USB_FET_NOT_FOUND_ERR
*/
#if ! defined(uController)
DLL430_SYMBOL STATUS_T WINAPI MSP430_GetNameOfUsbIf(LONG Idx, CHAR** Name, LONG* Status);
#endif

#if defined(__cplusplus)
}
#endif

#endif // MSP430_H
