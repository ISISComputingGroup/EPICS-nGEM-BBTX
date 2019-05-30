setlocal
call %~dp0..\..\iocBoot\iocnGEMTest\dllPath.bat
call %~dp0..\..\bin\%EPICS_HOST_ARCH%\nGEMFileReader.exe %*
