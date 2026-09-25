@echo off
setlocal
set "URL=https://temesotejam.github.io/realsense-browser-slam/d435-profile-sweep.html?backend=directshow"
set "PROFILE=%TEMP%\chrome-realsense-directshow"

set "CHROME=%ProgramFiles%\Google\Chrome\Application\chrome.exe"
if exist "%CHROME%" goto found
set "CHROME=%ProgramFiles(x86)%\Google\Chrome\Application\chrome.exe"
if exist "%CHROME%" goto found
set "CHROME=%LocalAppData%\Google\Chrome\Application\chrome.exe"
if exist "%CHROME%" goto found

echo Chrome executable was not found.
echo Please launch Chrome manually with:
echo   chrome.exe --force-directshow --user-data-dir="%PROFILE%" "%URL%"
pause
exit /b 1

:found
echo Launching isolated Chrome with DirectShow forced...
echo Chrome: %CHROME%
echo Profile: %PROFILE%
start "" "%CHROME%" --force-directshow --user-data-dir="%PROFILE%" "%URL%"
exit /b 0
