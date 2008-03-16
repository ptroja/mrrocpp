@echo off
rem ////////////////////////////////////////////////////////////////////////////
rem // 
rem // $URL$
rem //
rem //////////////////////////////////////////////////////////////////////////// 
rem // Revision  : $LastChangedRevision$
rem // Project   : MRROC++
rem ////////////////////////////////////////////////////////////////////////////
rem // Created   : 2008-03-16
rem // Developer : janusz.nowacki
rem // Modified  : $LastChangedDate$
rem // By        : $LastChangedBy$
rem ////////////////////////////////////////////////////////////////////////////
rem // File description:
rem // 
rem //  A script for indenting code in Windows.
rem // 
rem ////////////////////////////////////////////////////////////////////////////

echo Program: %0
echo Date: %date% 
echo Time: %time% 
echo Indenting MRROC++ ...


rem ----------------------------------------------------------------------------
rem                           Settings
rem ----------------------------------------------------------------------------

rem set INDENT="C:\Program Files\astyle\bin\AStyle.exe"
set INDENT="C:\cygwin\bin\indent.exe"
set FILE=%1

rem ----------------------------------------------------------------------------
rem                       Indent options: 
rem ----------------------------------------------------------------------------
rem set BLANK_LINES=--blank-lines-after-declarations 
rem                 --blank-lines-after-procedures 
rem                 --leave-optional-blank-lines
rem set BREAKING_LONG_LINES=
rem set DECLARATIONS=--blank-lines-after-commas 
rem                  --break-function-decl-args 
rem                  --braces-after-struct-decl-line 
rem                  --declaration-indentation1 
rem                  --dont-break-procedure-type
rem set STATEMENTS=--no-space-after-function-call-names 
rem                --no-space-after-parentheses
rem set COMMENTS=--dont-format-comments
rem set INDENTATIONS=
rem set MISC=--verbose"
rem set COMMON_STYLES=
rem set INVOKING_INDENT=
rem ----------------------------------------------------------------------------
set INDENT_STYLE=-gnu
set INDENT_OPTIONS=--dont-format-comments
rem set INDENT_OPTIONS=--style=linux 
rem                    --indent=tab=4 
rem                    --brackets=linux 
rem                    --indent-classes 
rem                    --indent-switches
rem ----------------------------------------------------------------------------

if "%1" == "" (goto indent_dir) else (goto indent_file)
goto :EOF  


rem ----------------------------------------------------------------------------
:indent_dir
echo %0 :: No file selected ... Indenting whole directory recursive
	
FOR %%i IN (*.h, *.cc, *.cpp) DO (CALL  %INDENT% %%i %INDENT_OPTIONS%)

sleep 2
exit
rem ----------------------------------------------------------------------------


rem ----------------------------------------------------------------------------
:indent_file
echo Indenting %1 ...
CALL %INDENT% %1 %INDENT_OPTIONS%
rem ----------------------------------------------------------------------------

