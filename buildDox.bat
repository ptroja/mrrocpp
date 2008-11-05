@echo off
rem ////////////////////////////////////////////////////////////////////////////
rem /** @file     buildDox.bat
rem  *
rem  *  Build file (for Windows system) for MRROC++ Doxygen documentation.
rem  *
rem  *  @author   janusz.nowacki
rem  *  @date     2007-05-12
rem  *  $URL$
rem  *  $LastChangedRevision$
rem  *  $LastChangedDate$  
rem  *  $LastChangedBy$
rem  *  
rem  */   
rem ////////////////////////////////////////////////////////////////////////////


echo Program: %0
echo Date: %date% 
echo Time: %time% 
echo Building the Doxygen documentation for MRROC++ ...

rem ----------------------------------------------------------------------------
rem                           Settings
rem ----------------------------------------------------------------------------

set CLEAN=cleanDox.bat
set DOX_VIEW=Dox.bat
set PL=PL
set EN=EN
set DOX_DIR=dox
set HTML_DOX=html
set HTML_DOX_MAIN=index.html
set LATEX_DOX=latex
set MAN_DOX=man3
set DOXYGEN=doxygen

rem set DOX_PL=DOXYGEN_PL
rem set DOX_EN=DOXYGEN_EN
rem set BUILD_EN=Y
rem set BUILD_PL=N

set MROK_DIR=%CD%
set DOX_MAINPAGE_TEMPLATE=Mainpage_template.dox
set DOX_MAINPAGE=Mainpage.dox
set SUBWCREV_DIR="C:\Program Files\TortoiseSVN\bin"
set SUBWCREV=%SUBWCREV_DIR%\SubWCRev.exe

rem if "%1" == "" set DOXYFILE_PL=DoxyMrok_pl
rem if "%2" == "" set DOXYFILE_EN=DoxyMrok_en
if "%1" == "" set DOXYFILE=DoxyMrok

rem ----------------------------------------------------------------------------
echo %0 :: Calling clean.bat ...
rem ----------------------------------------------------------------------------
CALL %CLEAN%

rem ----------------------------------------------------------------------------
echo %0 :: Calling SubWCRev on the current project dir ...
rem ----------------------------------------------------------------------------
CALL %SUBWCREV% %MROK_DIR% %DOX_MAINPAGE_TEMPLATE% %DOX_MAINPAGE%

rem ----------------------------------------------------------------------------
echo %0 :: Building the source code documentation ...
rem ----------------------------------------------------------------------------
rem if  "%BUILD_PL%" == "Y" %DOXYGEN% %DOXYFILE_PL%
rem if  "%BUILD_EN%" == "Y" %DOXYGEN% %DOXYFILE_EN%
%DOXYGEN% %DOXYFILE%

rem ----------------------------------------------------------------------------
echo %0 :: View the documentation ...
rem ----------------------------------------------------------------------------
SLEEP 1
CALL %DOX_VIEW%


