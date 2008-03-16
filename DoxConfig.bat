@echo off
rem ////////////////////////////////////////////////////////////////////////////
rem // 
rem // $URL$
rem //
rem ////////////////////////////////////////////////////////////////////////////
rem // Revision  : $LastChangedRevision$
rem // Project   : MRROC++
rem ////////////////////////////////////////////////////////////////////////////
rem // Created   : 2007-05-12
rem // Developer : janusz.nowacki
rem // Modified  : $LastChangedDate$
rem // By        : $LastChangedBy$
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
rem set DOXYFILE_PL=DoxyMrok_pl
rem set DOXYFILE_EN=DoxyMrok_en
set DOXYFILE=DoxyMrok

rem #########################################################################
echo %0 :: Start Doxywizard ...
rem #########################################################################
CALL %DOXYWIZARD% %DOXYFILE%


