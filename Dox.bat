@echo off
rem ////////////////////////////////////////////////////////////////////////////
rem //
rem //                              TECHNICAL SPECIFICATION
rem //
rem ////////////////////////////////////////////////////////////////////////////
rem // 
rem // $Id$
rem //
rem ////////////////////////////////////////////////////////////////////////////
rem // $URL: $
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
rem // Script for showing a doxygen documentation for MRROC++ in Windows system.
rem // 
rem ////////////////////////////////////////////////////////////////////////////

echo Program: %0
echo Date: %date% 
echo Time: %time% 
echo Show the Doxygen documentation for MRROC++ ...

rem ----------------------------------------------------------------------------
rem                                   Settings
rem ----------------------------------------------------------------------------

set PL=PL
set EN=EN
set DOX_DIR=dox
set HTML_DOX=html
set HTML_DOX_MAIN=index.html


rem ############################################################################
echo %0 :: View the documentation ...
rem ############################################################################
START %DOX_DIR%\%PL%\%HTML_DOX%\%HTML_DOX_MAIN%
sleep 1
START %DOX_DIR%\%EN%\%HTML_DOX%\%HTML_DOX_MAIN%




