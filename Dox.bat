@echo off
rem ////////////////////////////////////////////////////////////////////////////
rem /** @file     Dox.bat
rem  *
rem  *  Script for showing a doxygen documentation for MRROC++ in Windows system.
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
echo Show the Doxygen documentation for MRROC++ ...

rem ----------------------------------------------------------------------------
rem                                   Settings
rem ----------------------------------------------------------------------------

set DOX_DIR=dox
set HTML_DOX=html
set HTML_DOX_MAIN=index.html


rem ############################################################################
echo %0 :: View the documentation ...
rem ############################################################################
if exist "%DOX_DIR%\%HTML_DOX%\%HTML_DOX_MAIN%" START %DOX_DIR%\%HTML_DOX%\%HTML_DOX_MAIN%




