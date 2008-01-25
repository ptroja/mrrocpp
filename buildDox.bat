@echo off
rem // *INDENT-OFF*
rem /////////////////////////////////////////////////////////////////////////////////////////////////////////
rem //
rem //                                        TECHNICAL SPECIFICATION
rem //
rem /////////////////////////////////////////////////////////////////////////////////////////////////////////
rem // 
rem // $Id$
rem //
rem /////////////////////////////////////////////////////////////////////////////////////////////////////////
rem //File              : $HeadURL$
rem //Revision          : $LastChangedRevision$
rem //Application       : MRROC++
rem /////////////////////////////////////////////////////////////////////////////////////////////////////////
rem // Creation date    : 2007-01-12
rem // Last modification: $LastChangedDate$
rem // Developer        : $LastChangedBy$
rem //      (elka email): jnowacki@elka.pw.edu.pl
rem //      (nick)      : jarosz 
rem //      (real name) : Janusz Nowacki
rem /////////////////////////////////////////////////////////////////////////////////////////////////////////
rem // File modification history:
rem // 
rem // Date         User              Description
rem // ==========   ==============    =======================================================================
rem // 2007-08-18   janusz.nowacki    1) technical specification added 
rem // ----------   --------------    -----------------------------------------------------------------------
rem // 2007-10-03   janusz.nowacki    1) more technical specification lines
rem //                                2) minor layout changes
rem //                                3) SVN keywords added to file properties
rem // ----------   --------------    -----------------------------------------------------------------------
rem // 2007-10-04   janusz.nowacki    1) retab done in Vim, TAB key set to 2 spaces
rem //                                2) more layout changes
rem //
rem /////////////////////////////////////////////////////////////////////////////////////////////////////////
rem // File description:
rem // 
rem //  Build file (for Windows system) for MRROC++ Doxygen documentation.
rem // 
rem /////////////////////////////////////////////////////////////////////////////////////////////////////////
rem // *INDENT-ON*


echo Program: %0
echo Date: %date% 
echo Time: %time% 
echo Building the Doxygen documentation for MRROC++ ...

rem ----------------------------------------------------------------
rem                           Settings
rem ----------------------------------------------------------------

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
set DOX_PL=DOXYGEN_PL
set DOX_EN=DOXYGEN_EN

set MROK_DIR="F:\home\jn\MGR\JanuszNowacki_mgr_SANDBOX\mrrocpp"
set DOX_MAINPAGE_TEMPLATE=Mainpage_template.dox
set DOX_MAINPAGE=Mainpage.dox
set SUBWCREV_DIR="C:\Program Files\TortoiseSVN\bin"
set SUBWCREV=%SUBWCREV_DIR%\SubWCRev.exe

if "%1" == "" set DOXYFILE_PL=DoxyMrok_pl
if "%2" == "" set DOXYFILE_EN=DoxyMrok_en

rem ################################################################
echo %0 :: Calling clean.bat ...
rem ################################################################
CALL %CLEAN%

rem ################################################################
echo %0 :: Calling SubWCRev on the current project dir ...
rem ################################################################
CALL %SUBWCREV% %MROK_DIR% %DOX_MAINPAGE_TEMPLATE% %DOX_MAINPAGE%

rem ################################################################
echo %0 :: Building the source code documentation ...
rem ################################################################
%DOXYGEN% %DOXYFILE_PL%
%DOXYGEN% %DOXYFILE_EN%

rem ################################################################
echo %0 :: View the documentation ...
rem ################################################################
SLEEP 1
CALL %DOX_VIEW%


