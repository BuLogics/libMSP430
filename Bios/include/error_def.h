/**
* \ingroup MODULMACROS
*
* \file error_def.h
*
* \brief
*
*/
/*
 * Copyright (C) 2012 Texas Instruments Incorporated - http://www.ti.com/ 
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

#ifndef ERROR_DEFS_H
#define ERROR_DEFS_H

#define HALLERR_NO_ERROR                                    0x0000
#define HALERR_UNDEFINED_ERROR                              0xFFFF

/*
#define HALERR_WRITE_FLASH_BLOCK_NO_RAM_START               0xFFFE
#define HALERR_WRITE_FLASH_BLOCK_NO_RAM_SIZE                0xFFFD
#define HALERR_WRITE_FLASH_BLOCK_NO_FLASH_ADDRESS           0xFFFC
#define HALERR_WRITE_FLASH_BLOCK_NO_FLASH_SIZE              0xFFFB
#define HALERR_WRITE_FLASH_BLOCK_NO_LOCKA                   0xFFFA
#define HALERR_WRITE_FLASH_BLOCK_TIMEOUT_EXECUTE            0xFFF9
#define HALERR_WRITE_FLASH_BLOCK_TIMEOUT_FINISH             0xFFF8
#define HALERR_WRITE_FLASH_BLOCK_TIMEOUT_INIT               0xFFF7
#define HALERR_WRITE_FLASH_BLOCK_MISSING_STREAM_DATA        0xFFF6  
*/

#define HALERR_EXECUTE_FUNCLET_NO_RAM_START               0xFFFE
#define HALERR_EXECUTE_FUNCLET_NO_RAM_SIZE                0xFFFD
#define HALERR_EXECUTE_FUNCLET_NO_OFFSET                  0xFFFC
#define HALERR_EXECUTE_FUNCLET_NO_ADDRESS                 0xFFFB
#define HALERR_EXECUTE_FUNCLET_NO_LENGTH                  0xFFFA
#define HALERR_EXECUTE_FUNCLET_NO_TYPE                    0xFFF9
#define HALERR_EXECUTE_FUNCLET_NO_LOCKA                   0xFFF8
#define HALERR_EXECUTE_FUNCLET_EXECUTION_TIMEOUT          0xFFF7
#define HALERR_EXECUTE_FUNCLET_EXECUTION_ERROR            0xFFF6

#define HALERR_WRITE_MEM_WORD_NO_RAM_ADDRESS                0xFFF5
#define HALERR_WRITE_MEM_WORD_NO_RAM_SIZE                   0xFFF4
#define HALERR_WRITE_MEM_WORD_UNKNOWN                       0xFFF3

#define HALERR_WRITE_MEM_BYTES_NO_RAM_ADDRESS               0xFFF2
#define HALERR_WRITE_MEM_BYTES_NO_RAM_SIZE                  0xFFF1
#define HALERR_WRITE_MEM_BYTES_UNKNOWN                      0xFFF0

#define HALERR_WRITE_FLASH_WORD_NO_FLASH_ADDRESS            0xFFEF
#define HALERR_WRITE_FLASH_WORD_NO_FLASH_SIZE               0xFFEE
#define HALERR_WRITE_FLASH_WORD_UNKNOWN                     0xFFED

#define HALERR_WRITE_FLASH_QUICK_UNKNOWN                    0xFFEC

#define HALERR_START_JTAG_NO_PROTOCOL                       0xFFEB
#define HALERR_START_JTAG_PROTOCOL_UNKNOWN                  0xFFEA

#define HALERR_SET_CHAIN_CONFIGURATION_STREAM               0xFFE9

#define HALERR_RESTORECONTEXT_RELEASE_JTAG_NO_WDT_ADDRESS   0xFFE8
#define HALERR_RESTORECONTEXT_RELEASE_JTAG_NO_WDT_VALUE     0xFFE7
#define HALERR_RESTORECONTEXT_RELEASE_JTAG_NO_PC            0xFFE6
#define HALERR_RESTORECONTEXT_RELEASE_JTAG_NO_SR            0xFFE5
#define HALERR_RESTORECONTEXT_RELEASE_JTAG_NO_CONTROL_MASK  0xFFE4
#define HALERR_RESTORECONTEXT_RELEASE_JTAG_NO_MDB           0xFFE3

#define HALERR_READ_MEM_WORD_NO_ADDRESS                     0xFFF2
#define HALERR_READ_MEM_WORD_NO_SIZE                        0xFFF1

#define HALERR_READ_MEM_UNKNOWN                             0xFFE0

#define HALERR_READ_MEM_BYTES_NO_ADDRESS                    0xFFDF
#define HALERR_READ_MEM_BYTES_NO_SIZE                       0xFFDE

#define HALERR_PSA_NO_ADDRESS                               0xFFDD
#define HALERR_PSA_NO_SIZE                                  0xFFDC

#define HALERR_SYNC_JTAG_ASSERT_POR_JTAG_TIMEOUT            0xFFDB
#define HALERR_SYNC_JTAG_ASSERT_POR_NO_WDT_ADDRESS          0xFFDA
#define HALERR_SYNC_JTAG_ASSERT_POR_NO_WDT_VALUE            0xFFD9

#define HALERR_WRITE_ALL_CPU_REGISTERS_STREAM               0xFFD8

#define HALERR_WRITE_MEM_WORD_XV2_NO_RAM_ADDRESS            0xFFD7
#define HALERR_WRITE_MEM_WORD_XV2_NO_RAM_SIZE               0xFFD6

#define HALERR_SECURE_NO_TGT_HAS_TEST_PIN                   0xFFD5 // Missing TestPin parameter

#define HALERR_SYNC_JTAG_CONDITIONAL_JTAG_TIMEOUT           0xFFD4
#define HALERR_SYNC_JTAG_CONDITIONAL_NO_WDT_ADDRESS         0xFFD3
#define HALERR_SYNC_JTAG_CONDITIONAL_NO_WDT_VALUE           0xFFD2

#define HALERR_INSTRUCTION_BOUNDARY_ERROR                   0xFFD1

#define HALERR_JTAG_VERSION_MISMATCH                        0xFFD0 // Indicates the read JTAG version was not as expected

#define HALERR_JTAG_MAILBOX_IN_TIMOUT                       0xFFCF // JTAG Milbox timeout during shifting in Data
#define HALERR_JTAG_PASSWORD_WRONG                          0xFFCE // JTAG Milbox timeout during shifting in Data

#define HALERR_START_JTAG_NO_ACTIVATION_CODE                0xFFCD
#define HALERR_SINGLESTEP_WAITFOREEM_TIMEOUT                0xFFCC

#define HALERR_CONFIG_NO_PARAMETER                          0xFFCB // Indicates that the parameter is missing
#define HALERR_CONFIG_NO_VALUE                              0xFFCA // Indicates that the value to be assigned to the parameter is missing
#define CONFIG_PARAM_UNKNOWN_PARAMETER                      0xFFC9 // Invalid parameter has been given

#define HALERR_NO_NUM_BITS                                  0xFFC8 // Number of bit sequences parameter not specified
#define HALERR_ARRAY_SIZE_MISMATCH                          0xFFC7 // The array of bit sequences is smaller than expected

#define HALERR_NO_COMMAND                                   0xFFC6 // Hil command not specified
#define HALERR_UNKNOWN_COMMAND                              0xFFC5 // Hil command invalid
#define HALERR_NO_DATA                                      0xFFC4 // Data/instruction not specified
#define HALERR_NO_BIT_SIZE                                  0xFFC3 // No bitsize specified
#define HALERR_INVALID_BIT_SIZE                             0xFFC2 // Invalid bitsize specified (must be 8,16,20,32,64)

#define HALERR_UNLOCK_NO_PASSWORD_LENGTH                    0xFFC1 // No password length in message
#define HALERR_UNLOCK_INVALID_PASSWORD_LENGTH               0xFFC0 // Password in message shorter than specified

#define HALERR_EXECUTE_FUNCLET_FINISH_TIMEOUT               0xFFBF

#define HALERR_EXECUTE_FUNCLET_NO_MAXRSEL                   0xFFBE

#endif
