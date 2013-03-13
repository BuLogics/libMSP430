:: checks if installing of thirdparty products took place
@echo off

set THIRDPARTY_DIR=Thirdparty
set LIB_DIR=%THIRDPARTY_DIR%\lib
set INCLUDE_DIR=%THIRDPARTY_DIR%\include
set BOOST_INCLUDE_DIR=%INCLUDE_DIR%\boost
set INTTYPE_INCLUDE_FILES=inttypes.h stdint.h

:: changes to directory of this script
pushd "%~dp0"

if not exist %LIB_DIR% (
	echo %LIB_DIR% missing!
	goto error
)

if not exist %INCLUDE_DIR% (
	echo %INCLUDE_DIR% missing!
	goto error
)

if not exist %BOOST_INCLUDE_DIR% (
	echo %BOOST_INCLUDE_DIR% missing!
	goto fatal_error
)

pushd %INCLUDE_DIR%
for %%i in (%INTTYPE_INCLUDE_FILES%) do (
	if not exist %%i (
		echo %%i is missing!
		goto fatal_error
	)
)
popd

:: Very weak check - improve!!
pushd %LIB_DIR%
set FOUND_LIB=NONE
for /r %%i in (*) do (
	set FOUND_LIB=YES
)
if %FOUND_LIB%==NONE (
	echo No library files found! 
	goto :fatal_error
)
popd
popd
exit 0

:error
echo.
echo install_thirdparty.bat executed? See Readme.txt!
echo.
echo Finished with errors
popd
exit 1

:fatal_error
echo.
echo install_thirdparty.bat seems to be broken or configuration of install_thirdparty.bat is strange!
echo.
echo Finished with errors
popd
exit 1