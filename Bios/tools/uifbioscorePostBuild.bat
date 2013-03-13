REM uifbioscorePostBuild.bat
REM FB 09/02/2011

set "PARAM_1=%1%"
set "SRECCAT=%1%\tools\srec_cat.exe"
set "UIFBIOSCORE_TXT=%1%\objects\image\UifBiosCore.txt"
set "UIFBIOSCORE_TMP_TXT=%1%\objects\image\UifBiosCore.tmp.txt"
set "COREIMAGE_H=%1%\src\up\core_Image_V3.h"
set "MODIFYCORE_PL=%1%\tools\ModifyCore.pl"

REM converts UifBiosCore.txt to SRec C-Array in core_Image_V3.h, adds padding byte if necessary
%SRECCAT% %UIFBIOSCORE_TXT% -guess -fill 0xFF -within %UIFBIOSCORE_TXT% -guess -range-padding 2 -o %UIFBIOSCORE_TMP_TXT% -ti_txt
%SRECCAT% %UIFBIOSCORE_TMP_TXT% -guess -o %COREIMAGE_H% -ca coreImage -ow -c_comp

REM replaces "unsigned short" and "unsigned long" with "uint16_t" and "uint32_t" in core_Image_V3.h
REM and copy it into \include\Core.txt, which is used for dll internal core update
cd "%1"
perl %MODIFYCORE_PL%
