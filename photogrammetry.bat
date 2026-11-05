@echo off
setlocal
set "ROOT=%~dp0"
set "JAVA_BIN=java"
for /d %%J in ("%ROOT%tools\jdk-25*") do if exist "%%~J\bin\java.exe" set "JAVA_BIN=%%~J\bin\java.exe"
if defined JAVA_HOME set "JAVA_BIN=%JAVA_HOME%\bin\java.exe"
if exist "%ROOT%runtime\bin\java.exe" set "JAVA_BIN=%ROOT%runtime\bin\java.exe"
if defined POSE_JAVA_HOME set "JAVA_BIN=%POSE_JAVA_HOME%\bin\java.exe"
if not defined PHOTOGRAMMETRY_HEAP set "PHOTOGRAMMETRY_HEAP=8g"
"%JAVA_BIN%" --enable-native-access=ALL-UNNAMED -Xmx%PHOTOGRAMMETRY_HEAP% -cp "%ROOT%target\classes;%ROOT%lib\*" photogrammetry %*
exit /b %ERRORLEVEL%
