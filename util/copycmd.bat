REM @echo off
set "PATH=C:\windows\system32;C:\windows;C:\windows\System32\Wbem"
set "BASE=%1"
set "DIR=%2"
set "ARCHIVE=C:\test"
robocopy "%BASE%\%DIR%" "%ARCHIVE%\%DIR%" *.* /MOV /E
