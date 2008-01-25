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
rem //File              : $HeadURL$
rem //Revision          : $LastChangedRevision$
rem //Application       : MRROC++
rem /////////////////////////////////////////////////////////////////////////////////////////////////////////
rem // Creation date    : 2007-01-12
rem // Last modification: $LastChangedDate$
rem // Developer        : $LastChangedBy$
rem //      (elka email): jnowacki@elka.pw.edu.pl
rem //      (nick)      : jarosz 
rem //      (real name) : Janusz Nowacki
rem /////////////////////////////////////////////////////////////////////////////////////////////////////////
rem // File modification history:
rem // 
rem // Date         User              Description
rem // ==========   ==============    =======================================================================
rem // 2007-08-18   janusz.nowacki    1) technical specification added 
rem // ----------   --------------    -----------------------------------------------------------------------
rem // 2007-10-03   janusz.nowacki    1) layout changes
rem //                                2) deletion of Mainpage.dox added, which gets 
rem //                                   created by SubWCRev
rem //                                3) SVN keywords added to file properties
rem // ----------   --------------    -----------------------------------------------------------------------
rem // 2007-10-04   janusz.nowacki    1) retab done in Vim, TAB key set to 2 spaces
rem //                                2) more layout changes
rem //                                  
rem /////////////////////////////////////////////////////////////////////////////////////////////////////////
rem // File description:
rem // 
rem //  Clean file - deleting (in Windows system) MRROC++ Doxygen documentation.
rem // 
rem /////////////////////////////////////////////////////////////////////////////////////////////////////////
rem // *INDENT-ON*


echo Program: %0
echo Date: %date% 
echo Time: %time% 
echo Cleaning the Doxygen documentation for MRROC++ ...

rem ----------------------------------------------------------------
rem                             Settings
rem ----------------------------------------------------------------

set CLEAN=cleanDox.bat
set DOX_DIR=dox
set TMP=*.tmp
set SWAP=*~
set LOG=*.log

set MAINPAGE=Mainpage.dox


rem ################################################################
echo Cleaning the documentation directories ...
rem ################################################################

  if not exist %TMP%  (echo No %TMP% found) else (for %%i in (%TMP%) do del %%i) 
  
  if not exist %SWAP% (echo No %SWAP%  found) else (for %%i in (%SWAP%) do del %%i)
  
  if not exist %LOG%  (echo No %LOG%  found) else (for %%i in (%LOG%) do del %%i)
  
  if not exist %MAINPAGE%  (echo No %MAINPAGE%  found) else (for %%i in (%MAINPAGE%) do del %%i)

  cd %DOX_DIR%
  CALL %CLEAN% 
  cd ..

