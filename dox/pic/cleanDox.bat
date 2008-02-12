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
rem //  Clean file for dox/pic dir 
rem //  - deleting recursively (in Windows system) MRROC++ Doxygen documentation.
rem // 
rem ////////////////////////////////////////////////////////////////////////////

echo Program: %0
echo Date: %date% 
echo Time: %time%
 
set TMP=*.tmp
set SWAP=*~
set LOG=*.log

rem ############################################################################
echo dox\pic\cleanDox.bat
rem ############################################################################
	if not exist %TMP%  (echo No %TMP% found) 	else (for %%i in (%TMP%) do del %%i) 
	if not exist %SWAP% (echo No %SWAP%  found) else (for %%i in (%SWAP%) do del %%i)
	if not exist %LOG%  (echo No %LOG%  found) 	else (for %%i in (%LOG%) do del %%i)

