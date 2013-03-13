/*
 * MemoryManager.h
 *
 * Handles access to different memory modules by means of address.
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
#ifndef DLL430_MEMORYMANAGER_H
#define DLL430_MEMORYMANAGER_H

#include <inttypes.h>
#include <stddef.h>


#include <DLL430_SYMBOL.h>
namespace TI
{
	namespace DLL430
	{
		enum MemoryError { MEMORY_NO_ERROR, MEMORY_READ_ERROR, MEMORY_WRITE_ERROR, MEMORY_LOCKED_ERROR, MEMORY_UNLOCK_ERROR };
		
		/** \brief control a memory cache directly */
		class DLL430_SYMBOL MemoryCacheCtrl
		{
		public:
			/** \brief virtual dtor.
			 *         Else dtors of derived classes are never called
			 *
			 */
			virtual ~MemoryCacheCtrl () {};

			/** \brief enable or disable a memory cache
			 *
			 * \param yes enable cache is true, else disable it
			 */
			virtual void enable (bool yes) = 0;

			/** \brief check if a cache is enabled
			 *
			 * \return true if the cache is enabled, else false
			 */
			virtual bool isEnabled () const = 0;

			/** \brief check if a address is currently in the cache
			 *
			 * \return true if the address has a currently valid cache value, else false
			 */
			virtual bool checkCache (uint32_t address) const = 0;

			/** \brief mark an address as dirty
			 *
			 * A dirty address will be written on the next flush() call. If the address
			 * has not a valid  cache value, fill() will be called to achieve that.
			 *
			 * \param address the first address to mark
			 * \param count number of address to mark, starting with address
			 * \return true if all requested addresses were marked successfully, else false
			 */
			virtual bool markDirty (uint32_t address, size_t count) = 0;

			/** \brief fill a part of a cache
			 *
			 * Filling a cache requires reading from the target device. Addresses that
			 * already have a valid cache value are omitted.
			 *
			 * \param address the first address to fill
			 * \param count number of address to fill, starting with address
			 * \return true if all requested addresses were filled successfully, else false
			 */
			virtual bool fill (uint32_t address, size_t count) = 0;

			/** \brief flush a part of a cache
			 *
			 * Flushing a cache requires writing to the target device. Address that
			 * are not marked dirty are omitted.
			 *
			 * \param address the first address to flush
			 * \param count number of address to flush, starting with address
			 * \return true if all requested addresses were flushed successfully, else false
			 */
			virtual bool flush (uint32_t address, size_t count) = 0;

			/** \brief clear a part of a cache
			 *
			 * Clearing a part of the cache resets that part to the initial state of
			 * being invalid and not dirty. No target device interaction is necessary.
			 *
			 * \param address the first address to clear
			 * \param count number of address to clear, starting with address
			 * \return true if all requested addresses were cleared successfully, else false
			 */
			virtual void clear (uint32_t address, size_t count) = 0;
		};

		/** \brief memory area management */
		class DLL430_SYMBOL MemoryArea //: public MemoryRange, public MemoryAccess
		{
		public:
			/** \brief read from memory
			 *
			 * This registers a read call. The content of buffer is undefined
			 * until you call the sync() method.
			 *
			 * \param address the first address
			 * \param buffer the buffer to write the read data to
			 * \param count the number of elements to read
			 * \return true if the read was registered successfully, else false
			 */

			virtual bool read (uint32_t address, uint32_t* buffer, size_t count) = 0;

			/** \brief write to memory
			 *
			 * This registers a write call. The content of buffer must stay valid
			 * until you call the sync() method.
			 *
			 * \param address the first address
			 * \param buffer the buffer to read the data to write from
			 * \param count the number of elements to write
			 * \return true if the write was registered successfully, else false
			 */
			virtual bool write (uint32_t address, uint32_t* buffer, size_t count) = 0;
			
			/** \brief write to memory
			 *
			 * This registers a write call. The transfer is delayed
			 * until you call the sync() method.
			 *
			 * \param address the first address
			 * \param value the value to write to address
			 * \return true if the write was registered successfully, else false
			 */
			virtual bool write (uint32_t address, uint32_t value) = 0;

			/** \brief This function must be called after using read() or write()
			 *
			 * This function MUST be called to complete a read or write call.
			 * Reads and writes can be combined in any order unless they are inter-dependent.
			 *
			 * \return true if the sync was successfull, else false
			 */
			virtual bool sync () = 0;
			
			/** \brief erase a flash area completely
			 *
			 * This function erases a complete flash area. It will do nothing
			 * if the area is not a flash.
			 *
			 * \param mode for selection of erase mode all/segments
			 * \return true if flash was erased successfully, else false
			 */
			virtual bool erase () = 0;

			/** \brief erase a flash segment
			 *
			 * This function erases a flash segment. It will do nothing
			 * if the area is not a flash. The given start and end address
			 * mark all segments that any of the addresses in this range
			 * is in as to be erased. The addresses to no have to be aligned
			 * to segment size.
			 *
			 * \param start the first address
			 * \param end the last address
			 * \return true if flash segments were erased successfully, else false
			 */
			virtual bool erase (uint32_t start, uint32_t end) = 0;
			
			/** \brief verify memory content
			 *
			 * This function verifies the contents of memory by using the
			 * Pseudo Signature Algorithm (PSA). This is a lot faster then reading
			 * all memory since the PSA is executed on the target device itself.
			 * Note that this will flush all affected cache contents.
			 *
			 * \param address the first address
			 * \param buffer the data to verify against (one byte per buffer element)
			 * \param count the number of buffer elements
			 * \return true if the PSA values matched, else false
			 */
			virtual bool verify(uint32_t address, uint32_t* buffer, size_t count) = 0;			

			/** \brief check of the area is cacheable
			 *
			 * \return true if the area is readOnly, else false
			 */
			virtual bool isReadOnly () = 0;

			/** \brief get the memory start address
			 *
			 * \return the memory start address value
			 */
			virtual uint32_t getStart () const = 0;

			/** \brief get the memory end address
			 *
			 * \return the memory end address value
			 */
			virtual uint32_t getEnd () const = 0;

			/** \brief get the memory size
			 *
			 * \return the memory size value
			 */
			virtual uint32_t getSize () const = 0;

			/** \brief get the memory segment size
			 *
			 * This value only makes sense if the memory is a flash.
			 *
			 * \return the memory segment size value
			 */
			virtual uint32_t getSegmentSize () const = 0;

			/** \brief get number of memory banks 
			 *
			 * This value only makes sense if the memory is a flash.
			 *
			 * \return the memory segment size value
			 */
			virtual uint32_t getBanks () const = 0;

			/** \brief check if a memory is mapped to global address space
			 *
			 * \return true if the memory is mapped, else false
			 */
			virtual bool isMapped () const = 0;

			/** \brief get the name of the area
			 *
			 * \return the memory area name
			 */
			virtual const char* getName () const = 0;

			/** \brief check of the area is cacheable
			 *
			 * \return true if the area is cacheable, else false
			 */
			virtual bool isCacheable () const = 0;

			/** \brief get the memory cache control
			 *
			 * \return pointer to the memory cache control (can be 0)
			 */
			virtual MemoryCacheCtrl* getCacheCtrl () = 0;
		};
		
		/** \brief manages global address space or all memory areas */
		class DLL430_SYMBOL MemoryManager //: public MemoryAccess
		{
		public:
			/** \brief get a memory area by name and sub-index
			 *
			 * Currently standard names are:
			 *  "main" for the main flash,
			 *  "information" for the information flash,
			 *  "system" for the system RAM,
			 *  "boot" for the boot ROM,
			 *  "EEM" for the enhanced emulation module registers,
			 *  "CPU" for the CPU registers,
			 *  "peripheral8bit" for the peripheral registers (8 bit wide) and
			 *  "peripheral16bit" for the peripheral registers (16 bit wide) and
			 *  "SFR" for the special function registers
			 *
			 * \param name the name of the area
			 * \param subIndex addresses multiple areas of the same name (usually 0)
			 * \return pointer to a memory area if the combination of name and subIndex exists, else 0
			 */
			virtual MemoryArea* getMemoryArea (const char* name, size_t subIndex = 0) = 0;
			
			/** \brief get a memory area by index
			 *
			 * \param index the index of the area
			 * \return pointer to a memory area if index is valid, else 0
			 */
			virtual MemoryArea* getMemoryArea (size_t index) = 0;

			/** \brief get the number of memory areas
			 *
			 * \return the number of memory areas
			 */

			virtual size_t count () const = 0;

			/** \brief force real data exchange after write and sync
			 *
			 * \return true on success, else false
			 */
			virtual bool flushAll() = 0;

			/** \brief read from memory
			 *
			 * This registers a read call. The content of buffer is undefined
			 * until you call the sync() method.
			 *
			 * \param address the first address
			 * \param buffer the buffer to write the read data to
			 * \param count the number of elements to read
			 * \return true if the read was registered successfully, else false
			 */
			//virtual bool read (uint32_t address, uint32_t* buffer, size_t count) = 0;

			virtual bool read (uint32_t address, uint32_t* buffer, size_t count) = 0;

			/** \brief write to memory
			 *
			 * This registers a write call. The content of buffer must stay valid
			 * until you call the sync() method.
			 *
			 * \param address the first address
			 * \param buffer the buffer to read the data to write from
			 * \param count the number of elements to write
			 * \return true if the write was registered successfully, else false
			 */
			virtual bool write (uint32_t address, uint32_t* buffer, size_t count) = 0;
			
			/** \brief write to memory
			 *
			 * This registers a write call. The transfer is delayed
			 * until you call the sync() method.
			 *
			 * \param address the first address
			 * \param value the value to write to address
			 * \return true if the write was registered successfully, else false
			 */
			virtual bool write (uint32_t address, uint32_t value) = 0;

			/** \brief This function must be called after using read() or write()
			 *
			 * This function MUST be called to complete a read or write call.
			 * Reads and writes can be combined in any order unless they are inter-dependent.
			 *
			 * \return true if the sync was successfull, else false
			 */
			virtual bool sync () = 0;
			
			/** \brief erase a flash area completely
			 *
			 * This function erases a complete flash area. It will do nothing
			 * if the area is not a flash.
			 *
			 * \return true if flash was erased successfully, else false
			 */
			virtual bool erase () = 0;

			/** \brief erase a flash segment
			 *
			 * This function erases a flash segment. It will do nothing
			 * if the area is not a flash. The given start and end address
			 * mark all segments that any of the addresses in this range
			 * is in as to be erased. The addresses to no have to be aligned
			 * to segment size.
			 *
			 * \param start the first address
			 * \param end the last address
			 * \return true if flash segments were erased successfully, else false
			 */
			virtual bool erase (uint32_t start, uint32_t end) = 0;
			
			/** \brief verify memory content
			 *
			 * This function verifies the contents of memory by using the
			 * Pseudo Signature Algorithm (PSA). This is a lot faster then reading
			 * all memory since the PSA is executed on the target device itself.
			 * Note that this will flush all affected cache contents.
			 *
			 * \param address the first address
			 * \param buffer the data to verify against (one byte per buffer element)
			 * \param count the number of buffer elements
			 * \return true if the PSA values matched, else false
			 */
			virtual bool verify(uint32_t address, uint32_t* buffer, size_t count) = 0;			

			/** \brief check of the area is cacheable
			 *
			 * \return true if the area is readOnly, else false
			 */
			virtual bool isReadOnly () = 0;

			/** \brief	Locks/unlocks desired memory module.
			 *			Memory Module is identified by name.
			 *			Action == true locks module,
			 *			Action == false unlocks module.
			 * \return	true if module isProtected and the state switched actually from 
			 *			locked to unlocked or vice versa, false otherwise.
			 */
			virtual bool lock(const char* name, bool action) = 0;

			/** \brief get last error code
			 *
			 * Read and reset the last error code that occured during a memory operation.
			 *
			 * \return Error code
			 */
			virtual MemoryError getLastError() = 0;

			/** \brief Enable/disable preservation of Ram
			 *
			 * Ram will be saved and restored for operations that can modify Ram content
			 * if set to true.
			 *
			 */
			virtual void setRamPreserveMode(bool enabled) = 0;

			/** \brief Current Ram preservation mode
			 *
			 * Get current setting for Ram preservation
			 *
			 * \return Current state (enabled or not)
			 */
			virtual bool getRamPreserveMode() const = 0;

			/** \brief Check voltage for flash programming
			 *
			 * Check internal and external voltage against minimum voltage
			 *
			 * \return Voltage sufficient
			 */
			virtual bool checkMinFlashVoltage() const = 0;
		};

	};
};

#endif /* DLL430_MEMORYMANAGER_H */
