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
rem //  Clean file for dox/PL dir 
rem //  - deleting recursively (in Windows system) MRROC++ Doxygen documentation.
rem // 
rem ////////////////////////////////////////////////////////////////////////////

echo Program: %0
echo Date: %date% 
echo Time: %time% 

set CLEAN=cleanDox.bat

rem ############################################################################
echo dox\PL\cleanDox.bat
rem ############################################################################
	FOR %%i IN (*.*) DO ( 
												IF NOT %%i==%CLEAN% (del %%i) 
											)
	FOR /D %%i IN (*.*) DO (rmdir /S /Q  %%i) 

