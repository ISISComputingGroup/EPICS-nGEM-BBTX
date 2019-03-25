REM @echo off
set "PATH=C:\windows\system32;C:\windows;C:\windows\System32\Wbem"
set "BASE=%1"
set "DIR=%2"
REM these are set by ISIS NDX instrument
set "INSTNAME=%3"
set "INSTRUN=%4"

set "INSTDIR=%INSTNAME%%INSTRUN%"
set "ARCHIVE=C:\test"

REM wait 30 seconds
timeout /t 30 /nobreak >NUL

robocopy "%BASE%\%DIR%" "%ARCHIVE%\%INSTDIR%\%DIR%" *.* /MOV /E
