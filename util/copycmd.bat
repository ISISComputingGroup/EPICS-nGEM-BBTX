REM @echo off
REM reset PATH to default as EPICS path is long and can cause issues
setlocal
set "PATH=C:\windows\system32;C:\windows;C:\windows\System32\Wbem"
set "BASE=%1"
set "DIR=%2"
REM these are set by ISIS NDX instrument
set "INSTNAME=%3"
set "INSTRUN=%4"
set "ARCHIVE=%5"

set "INSTDIR=%INSTNAME%%INSTRUN%"
REM example archive directory, change locally

if not "%ARCHIVE%" == "" (
    REM wait 30 seconds but avoid timeout command as does not work non interactively
    ping -n 30 127.0.0.1 >NUL
    robocopy "%BASE%\%DIR%" "%ARCHIVE%\%INSTDIR%\%DIR%" *.* /MOV /E
)
