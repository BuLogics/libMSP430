/*
 * DLL430_SYMBOL.h 
 *
 * Defines symbols for outer world.
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

/** 
 *
 * Also see http://gcc.gnu.org/wiki/Visibility.
 */

#if _MSC_VER > 1000
#pragma once
#endif
#ifndef DLL430_SYMBOL_H
#define DLL430_SYMBOL_H

#if defined(_WIN32) || defined(_WIN64)
#  if defined(DLL430_EXPORT)
#    define DLL430_SYMBOL __declspec(dllexport)
#  else /* DLL430_EXPORT */
#    define DLL430_SYMBOL __declspec(dllimport)
#  endif /* DLL430_EXPORT */

#else
#  if defined(__GNU_C__) && __GNU_C__ >= 4
     /* You must add the -fvisibility=hidden command line option when
      * compiling the library.
      * Using this makes loading the lib a lot faster on some systems
      * and also restricts programs to the API.
      */
#    define DLL430_SYMBOL __attribute__((visibility("default")))
     /* GCC gives all member function the same visibility as the class
      * but you can use the following define to explicitely change
      * a member function to hidden
      */
#    define DLL430_NOSYMBOL __attribute__((visibility("hidden")))
#  endif

   /* work around a windows-specialty of calling conventions */
#  ifndef WINAPI
#    define WINAPI
#  endif
#endif

#ifndef DLL430_SYMBOL
#define DLL430_SYMBOL
#endif

#ifndef DLL430_NOSYMBOL
#define DLL430_NOSYMBOL
#endif

#endif /* DLL430_SYMBOL_H */
