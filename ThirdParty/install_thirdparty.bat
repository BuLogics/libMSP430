@echo off

:: Configure the downloaded files you want to use here but without the file type ending, e.g. boost-1_34_1 NOT boost-1_34_1.zip

set BOOSTDIR=boost_1_44_0
:: please set this too - you have to change the '_' to '-' after boost
set BOOSTDIRINCLUDEPREFIX=boost-1_44
set BJAMDIR=boost-jam-3.1.18
set INTTYPES=msinttypes-r26
set MSVC_Ver=msvc

:: Don't change settings below this line!

set BOOSTDOWNLOAD=http://sourceforge.net/projects/boost/files/boost/1.44.0
set BJAMDOWNLOAD=http://sourceforge.net/projects/boost/files/boost-jam/3.1.18
set INTTYPESDOWNLOAD=http://code.google.com/p/msinttypes/downloads/list

set LIBDIR=lib
set INCLDUEDIR=include

set BJAMBUILD=build.bat
set BJAMBINDIR=bin.ntx86
set BJAMEXE=bjam.exe

set GENERICBOOSTINCLUDE=%INCLDUEDIR%\boost
::set BOOSTRELEASEPATH=bin.v2\libs\thread\build\%MSVC_Ver%\release\link-static\runtime-link-static\threading-multi

set INTTYPES_FILES=inttypes.h stdint.h
set UNPROCESSEDINTTYPEFILES=changelog.txt

echo After installation there will be two further folders, "lib" and "include".
echo Unzipped files will be deleted in order to have a clean environment!

:: creating lib directory
if not exist %LIBDIR% (
	md %LIBDIR%
)

:: creating include directory
if not exist %INCLDUEDIR% (
	md %INCLDUEDIR%
)

:: installing inttypes
echo.
echo Installing inttypes!
echo.

if not exist %INCLDUEDIR% (
	goto error_no_include
)

for %%i in (%INTTYPES_FILES%) do (
	if not exist %INCLDUEDIR%\%%i (
		if not exist %%i (
			goto error_no_intfiles
		) else (
			move %%i %INCLDUEDIR%
		)
	)
)

:: installing bjam
if not exist %BJAMDIR% (
	goto error_no_bjam_dir
) else (
	pushd %BJAMDIR%
)

if not exist %BJAMBUILD% (
	goto error_no_bjam_bat
) else (
	echo.
	echo Building bjam!
	echo.
	echo.
	@echo on
	call %BJAMBUILD%
	popd
	@echo off
)

:: installing boost	
if not exist %BOOSTDIR% (
	goto error_no_boost_dir
) else (
	pushd %BOOSTDIR%
)

if not exist ../%BJAMDIR%/%BJAMBINDIR%/%BJAMEXE% (
	popd
	goto error_no_bjam_exe
) else (
	echo.
	echo Building and installing boost!
	echo.
	@echo on
	..\%BJAMDIR%\%BJAMBINDIR%\%BJAMEXE% clean
:: prefix=.. means, alib directory in the parent dir will be created
	..\%BJAMDIR%\%BJAMBINDIR%\%BJAMEXE% --toolset=%MSVC_Ver% --prefix=.. link=static --threading=multi runtime-link=static stage install
	popd
	@echo off
	if not exist %INCLDUEDIR% goto error_no_include
	if not exist %INCLDUEDIR%\%BOOSTDIRINCLUDEPREFIX% goto error_no_boost_include
	if not exist %INCLDUEDIR%\%BOOSTDIRINCLUDEPREFIX%\boost goto error_no_boost_boost_include
	move %INCLDUEDIR%\%BOOSTDIRINCLUDEPREFIX%\boost %GENERICBOOSTINCLUDE%
    rmdir /s /q %INCLDUEDIR%\%BOOSTDIRINCLUDEPREFIX%
)

goto finished


:error_no_bjam_dir
echo.
if exist %BJAMDIR%.zip (
	echo *** You have to unzip %BJAMDIR%.zip! ***
) else (
	echo *** Please download %BJAMDIR%.zip from "%BJAMDOWNLOAD%" 
	echo and/or change settings of %0! ***
)
goto finsished_error

:error_no_bjam_bat
echo.
echo *** You're unzipped Version of %BJAMDIR% seems corrupt! ***
echo Unzip %BJAMDIR%.zip again or download bjam from %BJAMDOWNLOAD%! 
goto finsished_error

:error_no_bjam_exe
echo.
echo *** %BJAMDIR%\%BJAMBINDIR%\%BJAMEXE% not found! ***
echo Check path and settings of %0! 
goto finsished_error

:error_no_boost_dir
echo.
if exist %BOOSTDIR%.zip (
	echo *** You have to unzip %BOOSTDIR%.zip! ***
) else (
	echo *** Please download %BOOSTDIR%.zip from "%BOOSTDOWNLOAD%" 
	echo and/or change settings of %0! ***
)
goto finsished_error

:error_no_include
echo.
echo *** Path %INCLDUEDIR% not found! ***
echo Script possibly broken. 
echo This should not happen. 
echo Check paths and configuartion of %0!
goto end

:error_no_boost_include
echo.
echo *** Path %INCLDUEDIR%\%BOOSTDIRINCLUDEPREFIX% not found! ***
echo Script possibly broken. 
echo This should not happen. 
echo Somethings possibly wrong with boost installation routine! 
echo You have to build and configure manually!
goto end

:error_no_boost_boost_include
echo.
echo *** Path %INCLDUEDIR%\%BOOSTDIRINCLUDEPREFIX%\boost not found! ***
echo This shouldn't happen. 
echo Somethings possibly wrong with boost installation routine! 
echo You have to build and configure manually!
goto end

:error_no_intfiles
echo.
if exist %INTTYPES%.zip (
	echo *** You have to unzip %INTTYPES%.zip! ***
) else (
	echo *** Please download %INTTYPES%.zip from "%INTTYPESDOWNLOAD%" 
	echo and/or change settings of %0! ***
)
goto finsished_error

:finsished_error
echo.
echo *** Finished with errors! ***
goto end

:finished
echo.
echo Finished!
goto end

:end
Pause





