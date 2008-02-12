@echo off
rem ////////////////////////////////////////////////////////////////////////////
rem //
rem //                              TECHNICAL SPECIFICATION
rem //
rem ////////////////////////////////////////////////////////////////////////////
rem // 
rem // $URL: $
rem //
rem ////////////////////////////////////////////////////////////////////////////
rem // Revision  : $LastChangedRevision: $
rem // Project   : MRROC++
rem ////////////////////////////////////////////////////////////////////////////
rem // Created   : 2007-05-12
rem // Developer : janusz.nowacki
rem // Modified  : $LastChangedDate: $
rem // By        : $LastChangedBy: $
rem ////////////////////////////////////////////////////////////////////////////
rem // File description:
rem // 
rem // Script for starting Doxywizard GUI for doxyfiles modifications.
rem // 
rem ////////////////////////////////////////////////////////////////////////////

echo Program: %0
echo Date: %date% 
echo Time: %time% 

rem -------------------------------------------------------------------------
rem                                 Settings
rem -------------------------------------------------------------------------

set DOXYWIZARD=doxywizard
set DOXYFILE_PL=DoxyMrok_pl
set DOXYFILE_EN=DoxyMrok_en

rem #########################################################################
echo %0 :: Start Doxywizard for the Polish documentation config file...
rem #########################################################################
CALL %DOXYWIZARD% %DOXYFILE_PL%

rem #########################################################################
echo %0 :: Start Doxywizard for the English documentation config file...
rem #########################################################################
START %DOXYWIZARD% %DOXYFILE_EN%

