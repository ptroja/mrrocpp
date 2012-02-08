---------------------------------------------------------------
 Example Projects for the Type II Reflexxes Motion Library
---------------------------------------------------------------


***************************************************************
 1. Directory Contents
***************************************************************

 - include: Folder for all header files of the Reflexxes API
   and the Type II Reflexxes Motion Library
 - src: Folder for the source code files of the Type II
   Reflexxes Motion Library and the six sample applications
 - Linux: Folder with example makefiles for Linux
 - MacOS: Folder with example makefiles for Mac OS X
 - Windows: Folder with example project files for Microsoft
   Windows (Visual Studio 2008 Express)


***************************************************************
 2. Getting Started
***************************************************************
Change to the directory 'Linux' or 'MacOS', respectively, and
enter

make clean all

to check whether all files compile correctly on your system.
Under Microsoft Windows, you can open the Visual Studio 
solution file 'ReflexxesTypeII_ExampleProject.sln' and 
re-complie all projects to perform the check on your system.
If everything compiles without error messages, you can take a
look at one of the simple sample applications in the 'src'
folder to learn about the simple and clean Reflexxes API and
to use it for your own applications. In case of problems or
issues with this procedure, please contact us at
support@reflexxes.com.


***************************************************************
 3. Documentation
***************************************************************

http://www.reflexxes.com/software/typeiirml/v1.2


***************************************************************
 A. Appendix - Entire Folder Structure
***************************************************************

    * ReflexxesTypeII
        o include Folder for all header files of the Reflexxes API and the source code of the Type II Reflexxes Motion Library
        o Linux Folder with example makefiles for Linux
                + RMLPositionSampleApplications Folder for the makefile of RMLPositionSampleApplication1.cpp, RMLPositionSampleApplication2.cpp, and RMLPositionSampleApplication3.cpp
                + RMLVelocitySampleApplications Folder for the makefile of RMLVelocitySampleApplication4.cpp, RMLVelocitySampleApplication5.cpp, and RMLVelocitySampleApplication6.cpp
                + TypeIIRML Folder for the makefile of the Type II Reflexxes Motion Library                
                + x64 Binary files for 64-bit environments
                      # debug Files with debug information (non-optimized)
                            * bin Executable files of all sample applications
                            * lib This folder contains the Reflexxes Type II Motion Library.
                            * obj Object files
                      # release Files without debug information (fully optimized)
                            * bin Executable files of all sample applications
                            * lib This folder contains the Reflexxes Type II Motion Library.
                            * obj Object files
                + x86 Binary files for 32-bit environments
                      # debug Files with debug information (non-optimized)
                            * bin Executable files of all sample applications
                            * lib This folder contains the Reflexxes Type II Motion Library.
                            * obj Object files
                      # release Files without debug information (fully optimized)
                            * bin Executable files of all sample applications
                            * lib This folder contains the Reflexxes Type II Motion Library.
                            * obj Object files
        o MacOS Folder with example makefiles for Mac OS X
                + RMLPositionSampleApplications Folder for the makefile of RMLPositionSampleApplication1.cpp, RMLPositionSampleApplication2.cpp, and RMLPositionSampleApplication3.cpp
                + RMLVelocitySampleApplications Folder for the makefile of RMLVelocitySampleApplication4.cpp, RMLVelocitySampleApplication5.cpp, and RMLVelocitySampleApplication6.cpp
                + TypeIIRML Folder for the makefile of the Type II Reflexxes Motion Library
                + x86 Binary files for 32-bit environments
                      # debug Files with debug information (non-optimized)
                            * bin Executable files of all sample applications
                            * lib This folder contains the Reflexxes Type II Motion Library.
                            * obj Object files
                      # release Files without debug information (fully optimized)
                            * bin Executable files of all sample applications
                            * lib This folder contains the Reflexxes Type II Motion Library.
                            * obj Object files
        o src Folder for the source code files of the six sample applications
                + RMLPositionSampleApplications Source code of RMLPositionSampleApplication1.cpp, RMLPositionSampleApplication2.cpp, and RMLPositionSampleApplication3.cpp
                + RMLVelocitySampleApplications Source code of RMLVelocitySampleApplication4.cpp, RMLVelocitySampleApplication5.cpp, and RMLVelocitySampleApplication6.cpp
                + TypeIIRML Source code of the Type II Reflexxes Motion Library
        o Windows Folder with example project files for Microsoft Windows (Visual Studio 2008 Express)
                + Debug Binary files with debug information (non-optimized)
                + Release Binary files without debug information (fully optimized)
                + RMLPositionSampleApplication1 Folder for the project file of RMLPositionSampleApplication1.cpp
                + RMLPositionSampleApplication2 Folder for the project file of RMLPositionSampleApplication2.cpp
                + RMLPositionSampleApplication3 Folder for the project file of RMLPositionSampleApplication3.cpp
                + RMLVelocitySampleApplication4 Folder for the project file of RMLVelocitySampleApplication4.cpp
                + RMLVelocitySampleApplication5 Folder for the project file of RMLVelocitySampleApplication5.cpp
                + RMLVelocitySampleApplication6 Folder for the project file of RMLVelocitySampleApplication6.cpp
                + TypeIIRML Folder for the project file of the Type II Reflexxes Motion Library

---------------------------------------------------------------
Copyright (C) 2012 Reflexxes GmbH