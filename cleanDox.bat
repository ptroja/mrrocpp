@echo off
rem ////////////////////////////////////////////////////////////////////////////
rem /** @file     cleanDox.bat
rem  *
rem  *  Clean file - deleting (in Windows system) MRROC++ Doxygen documentation.
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
echo Cleaning the Doxygen documentation for MRROC++ ...

rem ----------------------------------------------------------------------------
rem                           Settings
rem ----------------------------------------------------------------------------
set CLEAN=cleanDox.bat
set DOX_DIR=dox
set TMP=*.tmp
set SWAP=*~
set LOG=*.log

set MAINPAGE=Mainpage.dox


rem ############################################################################
echo Cleaning the documentation directories ...
rem ############################################################################

  if not exist %TMP%  (echo No %TMP% found) else (for %%i in (%TMP%) do del %%i) 
  
  if not exist %SWAP% (echo No %SWAP%  found) else (for %%i in (%SWAP%) do del %%i)
  
  if not exist %LOG%  (echo No %LOG%  found) else (for %%i in (%LOG%) do del %%i)
  
  if not exist %MAINPAGE%  (echo No %MAINPAGE%  found) else (for %%i in (%MAINPAGE%) do del %%i)

  cd %DOX_DIR%
  CALL %CLEAN% 
  cd ..

