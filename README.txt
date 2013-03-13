1. Prerequesites
================

1.1 Development environment
---------------------------

* You will need to have Microsoft Visual Studio 2003 or later installed.
* You will need to have a IAR EW430 installed if you want to build the firmware.
* A program for unzipping thidparty products, e.g. WinZip.


1.2. Thirdparty dependencies
----------------------------

In order to compile the project within MSVC you have to download the following files into the Thirdparty folder:

* msinttypes-r26.zip from http://code.google.com/p/msinttypes/downloads/list 
* boost_1_44_0.zip from http://sourceforge.net/projects/boost/files/boost/1.44.0/
* boost-jam-3.1.18.zip from http://sourceforge.net/projects/boost/files/boost-jam/3.1.18/


1.3 Installing Thirdparty
--------------------------
* Open the Thirdparty folder within an explorer.
* Unzip the files you downloaded in 1.2 directly into that folder. You should now have a structure similar to the following:
	- MSP430_DLLv3_OS
		+ Bios
		+ DLL430_v3
		+ HalObjectDb
		- ThirdParty
			msinttypes-r26.zip
			boost-jam-3.1.18.zip
			boost_1_44_0.zip
			stdint.h
			inttypes.h
			changelog.txt
			+ boost-jam-3.1.18
			+ boost_1_44_0
* Still within the Thirdparty folder execute the install_thirdparty.bat
* Ensure that the MSP430_DLLv3_OS\ThirdParty\include folder conatins the boost directory - if not, copy the Thirdparty folder to a location near the drive letter, e.g. c:\Thirdparty as most probably a filename was too long

1.4 Build dependencies
----------------------
In order to build the DLL some tools are needed.

* You need a perl installation. perl.exe need to be in your PATH variable.

1.5 Thirdparty build dependencies
---------------------------------
In order to build the funclets with the IAR IDE you need to download the following tools:

* srecord-1.59-win32.zip from  http://sourceforge.net/projects/srecord/files/srecord-win32/1.59/srecord-1.59-win32.zip/download

1.6 Installing Thirdparty build dependencies
--------------------------------------------
* Extract the file srec_cat.exe from record-1.59-win32.zip into 
	- MSP430_DLLv3_OS
		- Bios
            tools
                srec_cat.exe

2. Building
===========

Windows:
* Open Bios/bios_coreOSS.eww in IAR EW430
* Select all subprojects and select "Rebuild All" with right mouse button
* Go to the root of your MSP430_DLLv3_OS folder
* Open the DLL430_v3_OpenSource.sln solution file with MSVC (for versions newer than 2003 the projects will be converted)
* Choose Build -> Build Solution from the menu

Linux:
* Go to your MSP430_DLLv3_OS directory
* Call make (if paths to your boost directory are not set up, you can call "make BOOST_DIR=<path to boost>")
* A libmsp430.so will be created in the current directory

