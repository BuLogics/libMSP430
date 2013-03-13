REM uifhalPostBuild.bat
REM DF 09/06/2011

set "PARAM_1=%1%"
set "SRECCAT=%1%\tools\srec_cat.exe"
set "UIFHAL_TXT=%1%\objects\image\UifHal.txt"
set "UIFHAL_TMP_TXT=%1%\objects\image\UifHal.tmp.txt"
set "UIFHAL_H=%1%\src\hal\UifHal.h"
set "MODIFYHAL_PL=%1%\tools\ModifyHal.pl"

REM converts UifHal.txt to SRec C-Array in UifHal.h, adds padding byte if necessary
%SRECCAT% %UIFHAL_TXT% -guess -fill 0xFF -within %UIFHAL_TXT% -guess -range-padding 2 -o %UIFHAL_TMP_TXT% -ti_txt
%SRECCAT% %UIFHAL_TMP_TXT% -guess -o %UIFHAL_H% -ca halImage -ow -c_comp

REM replaces "unsigned short" and "unsigned long" with "uint16_t" and "uint32_t" in UifHal.h
REM and copy it into \include\UifHal.txt, which is used for dll internal HAL update
cd %PARAM_1%
perl %MODIFYHAL_PL%

