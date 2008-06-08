@echo off
rem ////////////////////////////////////////////////////////////////////////////
rem /** @file     DoxConfig.bat
rem  *
rem  *  Script for starting Doxywizard GUI for doxyfiles modifications.
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


