/**
* \ingroup MODULBIOS
*
* \file v3_0p_hw_uif.c
*
* \brief Bios included HAL (zero) and update functions
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

//!  \author  Detlef Fink (03/09/2010)
//!  
//! \li forwarding (execute) messages to HAL
//! \li upload of HAL macros
//! \li loop management

#include "hw_compiler_specific.h"
#include "bios.h"
#include "../v3_0p.h"
#include "../stream.h"
#include <stdlib.h>


// values for memory flashing functions
const unsigned short BIOS_WRITE_ALLOWED_SEGMENTS[] = { 0x1000, 0x18FF, 0x2500, 0xDFFF, 0xFDFA, 0xFDFF };
const unsigned short BIOS_ERASE_ALLOWED_SEGMENTS[] = { 0x1000, 0x18FF, 0x2500, 0xDFFF };
const unsigned short BIOS_INFO_SEGMENTS[] = { 0x1000, 0x10FF };
const unsigned short BIOS_RAM_SEGEMENTS[] = { 0x0200, 0x09ff, 0x1100, 0x18FF };
const unsigned char  BIOS_FLASH_SEGMENT = 0;
const unsigned char  BIOS_FLASH_SEGMENT_N = 1;
const unsigned char  BIOS_INFO_SEGMENT = 2;
const unsigned char  BIOS_RAM_SEGMENT = 3;
const unsigned short BIOS_SEGMENT_SIZES[] = {256, 128, 64, 1 }; // segment size in words (2 byte) of Flash, 
const unsigned char  BIOS_FLASH_STEP[] = { 1, 1, 1, 1 };   // in future here can switch to flash longs

// informations about first and last measages 
unsigned char v3op_core_flash_enabled_ = 0;

// Prototypes
void v3opHwReset(void);
unsigned char v3opGetSegmentType(unsigned short addr);
unsigned char v3opWriteAllowed(unsigned short addr);
unsigned char v3opEraseAllowed(unsigned short addr);
short v3opCoreFlashFunctionInit(unsigned char *payload);
short v3opCoreFlashFunctionErase(unsigned char *payload);
short v3opCoreFlashFunctionWrite(unsigned char *payload);
short v3opCoreFlashFunctionRead(unsigned char *payload);
void v3opUpCore(void);


void v3opHwReset(void)
{
    for(unsigned short i=0; i < 0xFFFF; i++) __no_operation();  // wait, give rx function time for sending response by irq-service
    WDTCTL = 0;
}

//! \brief test address on memory type
//! \param[in] addr address to test
//! \return 0 -> flash memory
//! \return 1 -> info memory
//! \return 2 -> RAM
unsigned char v3opGetSegmentType(unsigned short addr)
{
    unsigned char i;
    unsigned char segment_type = BIOS_FLASH_SEGMENT;
  
    for(i = 0; i < (sizeof(BIOS_INFO_SEGMENTS)/sizeof(unsigned short)); i+=2)
    {
        if((addr >= BIOS_INFO_SEGMENTS[i]) && (addr <= BIOS_INFO_SEGMENTS[i+1]))
        {
            break;
        }
    }
    if(i < (sizeof(BIOS_INFO_SEGMENTS)/sizeof(unsigned short))) // not INFO-Memory
    {
        segment_type = BIOS_INFO_SEGMENT;
    }
    else
    {
        for(i = 0; i < (sizeof(BIOS_RAM_SEGEMENTS)/sizeof(unsigned short)); i+=2)
        {
            if((addr >= BIOS_RAM_SEGEMENTS[i]) && (addr <= BIOS_RAM_SEGEMENTS[i+1]))
            {
                break;
            }
        }
        if(i < (sizeof(BIOS_RAM_SEGEMENTS)/sizeof(unsigned short))) // not INFO-Memory
        {
            segment_type = BIOS_RAM_SEGMENT;
        }
    }
    if((segment_type == BIOS_FLASH_SEGMENT) && (addr >= 0x2500) && (addr < 0x25FF))
    {
        segment_type = BIOS_FLASH_SEGMENT_N;
    }
    return(segment_type);
}



//! \brief test address on write access
//! \param[in] addr address to test
//! \return 0 -> no write access
//! \return 1 -> write allow
unsigned char v3opWriteAllowed(unsigned short addr)
{
    unsigned char i;
    unsigned char write_allowed = 0;
    
    for(i = 0; i < (sizeof(BIOS_WRITE_ALLOWED_SEGMENTS)/sizeof(unsigned short)); i+=2)
    {
        if((addr >= BIOS_WRITE_ALLOWED_SEGMENTS[i]) && (addr <= BIOS_WRITE_ALLOWED_SEGMENTS[i+1]))
        {
            break;
        }
    }
    if(i < (sizeof(BIOS_WRITE_ALLOWED_SEGMENTS)/sizeof(unsigned short)))
    {
        write_allowed = 1;
    }
    return(write_allowed);
}



//! \brief test address on erase access
//! \param[in] addr address to test
//! \return 0 -> no erase access
//! \return 1 -> erase allow
unsigned char v3opEraseAllowed(unsigned short addr)
{
    unsigned char i;
    unsigned char erase_allowed = 0;
    
    for(i = 0; i < (sizeof(BIOS_ERASE_ALLOWED_SEGMENTS)/sizeof(unsigned short)); i+=2)
    {
        if((addr >= BIOS_ERASE_ALLOWED_SEGMENTS[i]) && (addr <= BIOS_ERASE_ALLOWED_SEGMENTS[i+1]))
        {
            break;
        }
    }
    if(i < (sizeof(BIOS_ERASE_ALLOWED_SEGMENTS)/sizeof(unsigned short))) // not INFO-Memory
    {
        erase_allowed = 1;
    }
    return(erase_allowed);
}



//! \brief lock/unlock write/erase to UIF (HAL) flash memory
//! \param[in] *payload pointer to receive buffer
//! \return 0 -> flash write/erase locked
//! \return 1 -> flash write/erase released
//! \return <0 -> error
short v3opCoreFlashFunctionInit(unsigned char *payload)
{
    short ret_value = -1;

    if(payload[4] == 0)
    {
        v3op_core_flash_enabled_ = 0;
        FCTL3 = FWKEY+LOCK;
        biosHalInterfaceInit();
        biosLedAlternate(0);
        biosLedOff(BIOS_LED_MODE);
        biosLedOn(BIOS_LED_POWER);
        ret_value = 0;
    }
    else if(payload[4] == 1)
    {
        v3op_core_flash_enabled_ = 1;
        biosLedAlternate(30);
        v3opKillAllLoops();
        ret_value = 1;
    }  
    return(ret_value);
}

//! \brief erase on UIF (HAL) all blocks which included in start address + size
//! \param[in] *payload pointer to receive buffer
//! \return 1 -> flash erase done
//! \return <0 -> error
short v3opCoreFlashFunctionErase(unsigned char *payload)
{
    unsigned short *address_pointer;
    unsigned short start_addr = *(unsigned long*)&payload[4] & 0xFFFF;
    unsigned short end_addr = start_addr + (*(unsigned long*)&payload[8] & 0xFFFF)-2;
    unsigned char  segment_type;
    short ret_value = -1;

    if(v3op_core_flash_enabled_ == 1)
    {
        switch(v3opGetSegmentType(start_addr))
        {
            case 0: // BIOS_FLASH_SEGMENT
                start_addr >>= 9;
                start_addr <<= 9;
                break;
            case 1: // BIOS_FLASH_SEGMENT_N
                start_addr >>= 8;
                start_addr <<= 8;
                break;
            case 2: // BIOS_INFO_SEGMENT
                start_addr >>= 7;
                start_addr <<= 7;
                break;
            case 3: // BIOS_INFO_SEGMENT
                start_addr >>= 1;
                start_addr <<= 1;
                break;
        }
        switch(v3opGetSegmentType(end_addr))
        {
            case 0: // BIOS_FLASH_SEGMENT
                end_addr >>= 9;
                end_addr <<= 9;
                break;
            case 1: // BIOS_FLASH_SEGMENT_N
                end_addr >>= 8;
                end_addr <<= 8;
                break;
            case 2: // BIOS_INFO_SEGMENT
                end_addr >>= 7;
                end_addr <<= 7;
                break;
           case 3: // BIOS_INFO_SEGMENT
                end_addr >>= 1;
                end_addr <<= 1;
                break;
        }
        end_addr += BIOS_SEGMENT_SIZES[v3opGetSegmentType(end_addr)] * 2;
        ret_value = -2;
        if(!(start_addr & 0x0001) && (start_addr < end_addr))
        {
            while(FCTL3 & BUSY);                // Test BUSY - Loop while busy
            FCTL2 = FWKEY+FSSEL1+19;            // SMCLK/20 = 8 Mhz/20 ~ 400 kHz
            FCTL3 = FWKEY;                      // Lock = 0
                        
            ret_value = 1;
            for (address_pointer = (unsigned short*)start_addr; address_pointer < (unsigned short*)end_addr; address_pointer += BIOS_SEGMENT_SIZES[segment_type])
            {
                segment_type = v3opGetSegmentType((unsigned short)address_pointer);
                if(v3opEraseAllowed((unsigned short)address_pointer))
                {         
                    if((segment_type == BIOS_FLASH_SEGMENT) || (segment_type == BIOS_FLASH_SEGMENT_N) || (segment_type == BIOS_INFO_SEGMENT))
                    {     
                        FCTL1 = FWKEY+ERASE;                // Erase = 1
                        *address_pointer = 0xFFFF;          // no while (FCTL3 & BUSY), code is running in flash
                        while(FCTL3 & BUSY);                // Test BUSY - Loop while busy
                    }
                    else if(segment_type == BIOS_RAM_SEGMENT)
                    {
                        *address_pointer = 0xFFFF;
                    }
                }
                else
                {
                    FCTL1 = FWKEY;                      // Clear Erase
                    FCTL3 = FWKEY+LOCK;                 // Lock = 1
                    ret_value = -2;
                    break;
                }
            }
            FCTL1 = FWKEY;                      // Clear Erase
            FCTL3 = FWKEY+LOCK;                 // Lock = 1
        }
    }
    return(ret_value);
}



//! \brief write payload to UIF (HAL) flash memory, RAM is allowed, too
//! \param[in] *payload pointer to receive buffer
//! \details on first function call the:
//! \li bytes 4 -7: the flash address
//! \li bytes 8 -11: data length 
//! \li bytes 12 ... payload
//! \details if data length is larger then message size the function returns after writing the message. Direct follow calls 
//! of this function continue writing.
//! \return 1 -> flash write done
//! \return <0 -> error
short v3opCoreFlashFunctionWrite(unsigned char *payload)
{
    static unsigned short *address_pointer = NULL;
    static unsigned short start_addr = 0;
    static unsigned short end_addr = 0;
    unsigned char  segment_type;
    short ret_value = -1;
    unsigned short *data_ptr = NULL;
    unsigned short *data_ptr_end = NULL;
    
    if(v3op_core_flash_enabled_ == 1)
    {
        if(v30p_stream_flags_ & MESSAGE_NEW_MSG)
        {
            start_addr = *(unsigned long*)&payload[4] & 0xFFFF;
            end_addr = start_addr + *(unsigned long*)&payload[8] & 0xFFFF;
            address_pointer = (unsigned short*)start_addr;
            data_ptr = (unsigned short*)&payload[12];
            data_ptr_end = data_ptr + (payload[0] - 11) / sizeof(unsigned short);
            if(!(start_addr & 0x0001) && (start_addr < end_addr))
            {
                FCTL2 = FWKEY+FSSEL1+19;            // SMCLK/20 = 8 Mhz/20 ~ 400 kHz
                FCTL3 = FWKEY;
                FCTL1 = FWKEY + WRT;
            }
            else
            {
                ret_value = -3;
            }
        }
        else
        {
            data_ptr = (unsigned short*)&payload[4];
            data_ptr_end = data_ptr + (payload[0] - 3) / sizeof(unsigned short);
        }
        if(ret_value == -1)
        {
            segment_type = v3opGetSegmentType((unsigned short)address_pointer);
            for(; (address_pointer < (unsigned short*)end_addr) && (data_ptr < data_ptr_end); address_pointer += BIOS_FLASH_STEP[segment_type], data_ptr++)
            {
                segment_type = v3opGetSegmentType((unsigned short)address_pointer);
                if(v3opWriteAllowed((unsigned short)address_pointer))
                {
                    *address_pointer = *data_ptr;
                }
                else
                {
                    ret_value = -4;
                    break;
                }
            }
            ret_value = 1;
        }
    }
    return(ret_value);
}


//! \brief reads UIF memory, inclusive BIOS area
//! \param[in] *payload pointer to receive buffer
//! \li bytes 4 -7: the flash address
//! \li bytes 8 -11: data length 
//! \return 0 
#pragma optimize = low
short v3opCoreFlashFunctionRead(unsigned char *payload)
{
    unsigned short address_pointer;
    unsigned short end_addr = 0;
    short ret_value = 0;
    
    if(payload[0] == 11)
    {
        if(STREAM_out_init(payload[MESSAGE_MSG_ID_POS], RESPTYP_DATA) >= 0)
        {
            address_pointer = *(unsigned short*)&payload[4] & 0xFFFF;
            end_addr = address_pointer + *(unsigned short*)&payload[8] & 0xFFFF;
            for(; address_pointer < end_addr; address_pointer +=2 )
            {
                STREAM_put_word(*(unsigned short*)address_pointer);
            }
            ret_value = MESSAGE_NO_RESPONSE;
        }
        else
        {
            ret_value = EXCEPTION_TX_NO_BUFFER;
        }
    }
    else
    {
        ret_value = EXCEPTION_RX_LENGTH;
    }
    if(ret_value == MESSAGE_NO_RESPONSE)
    {
        STREAM_flush();
    }
    return(ret_value);
}

#pragma optimize = low
void v3opUpCore(void)
{
    unsigned short i;
  
    unsigned short *address_pointer = 0;       
    address_pointer = (unsigned short *)0xFDFC;  
    // Erase Segment with Core Signature, to start install new core on restart
    FCTL2 = FWKEY+FSSEL1+19;            // SMCLK/20 = 8 Mhz/20 ~ 400 kHz
    FCTL3 = FWKEY;                      // Lock = 0
    FCTL1 = FWKEY+ERASE;                // Erase = 1                                    
    *address_pointer = 0xFFFF;       
    while(FCTL3 & BUSY);                // Test BUSY - Loop while busy
    FCTL1 = FWKEY;                      // Clear Erase
    FCTL3 = FWKEY+LOCK;                 // Lock = 1 
    
    for(i=0;i<100;i++)
    {
      biosLedBlink(0,30);
    }
    WDTCTL=0;                           // force PUC reset
                                      // on restart the safecore copy the new core
                                      // to the target location
}
