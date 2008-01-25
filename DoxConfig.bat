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
rem //File              : $HeadURL:  $
rem //Revision          : $LastChangedRevision: $
rem //Application       : MRROC++
rem /////////////////////////////////////////////////////////////////////////////////////////////////////////
rem // Creation date    : 2007-01-12
rem // Last modification: $LastChangedDate:  $
rem // Developer        : $LastChangedBy: $
rem //      (elka email): jnowacki@elka.pw.edu.pl
rem //      (nick)      : jarosz 
rem //      (real name) : Janusz Nowacki
rem /////////////////////////////////////////////////////////////////////////////////////////////////////////
rem // File modification history:
rem // 
rem // Date         User              Description
rem // ==========   ==============    =======================================================================
rem // 2007-10-04   janusz.nowacki    1) technical spec. added
rem //                                2) retab done in Vim, TAB key set to 2 spaces
rem //                                3) layout changes
rem //
rem /////////////////////////////////////////////////////////////////////////////////////////////////////////
rem // File description:
rem // 
rem //  Script for starting Doxywizard GUI for doxyfiles modifications.
rem // 
rem /////////////////////////////////////////////////////////////////////////////////////////////////////////
rem // *INDENT-ON*


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

