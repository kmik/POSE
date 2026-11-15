@echo off
rem POSE desktop window (photogrammetry gui), without a console window.
rem Set POSE_GUI_CONSOLE=1 to keep a console (for error messages).
setlocal
set "ROOT=%~dp0"
set "JAVA_BIN=java"
for /d %%J in ("%ROOT%..\sgm_java\tools\jdk-25*") do if exist "%%~J\bin\java.exe" set "JAVA_BIN=%%~J\bin\java.exe"
for /d %%J in ("%ROOT%tools\jdk-25*") do if exist "%%~J\bin\java.exe" set "JAVA_BIN=%%~J\bin\java.exe"
if defined JAVA_HOME set "JAVA_BIN=%JAVA_HOME%\bin\java.exe"
if exist "%ROOT%runtime\bin\java.exe" set "JAVA_BIN=%ROOT%runtime\bin\java.exe"
if defined POSE_JAVA_HOME set "JAVA_BIN=%POSE_JAVA_HOME%\bin\java.exe"
if not defined PHOTOGRAMMETRY_HEAP set "PHOTOGRAMMETRY_HEAP=8g"
if defined POSE_GUI_CONSOLE goto console
set "JAVAW_BIN=%JAVA_BIN:java.exe=javaw.exe%"
if "%JAVA_BIN%"=="java" set "JAVAW_BIN=javaw"
start "POSE" "%JAVAW_BIN%" --enable-native-access=ALL-UNNAMED -Xmx%PHOTOGRAMMETRY_HEAP% -cp "%ROOT%target\classes;%ROOT%lib\*" photogrammetry gui %*
exit /b 0
:console
"%JAVA_BIN%" --enable-native-access=ALL-UNNAMED -Xmx%PHOTOGRAMMETRY_HEAP% -cp "%ROOT%target\classes;%ROOT%lib\*" photogrammetry gui %*
exit /b %ERRORLEVEL%
