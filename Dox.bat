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
rem //  Script for showing a doxygen documentation for MRROC++ in Windows system.
rem // 
rem /////////////////////////////////////////////////////////////////////////////////////////////////////////
rem // *INDENT-ON*


echo Program: %0
echo Date: %date% 
echo Time: %time% 
echo Show the Doxygen documentation for MRROC++ ...

rem ----------------------------------------------------------------
rem                           Settings
rem ----------------------------------------------------------------

set PL=PL
set EN=EN
set DOX_DIR=dox
set HTML_DOX=html
set HTML_DOX_MAIN=index.html


rem ################################################################
echo %0 :: View the documentation ...
rem ################################################################
START %DOX_DIR%\%PL%\%HTML_DOX%\%HTML_DOX_MAIN%
sleep 1
START %DOX_DIR%\%EN%\%HTML_DOX%\%HTML_DOX_MAIN%




