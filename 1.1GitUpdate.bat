@echo off
REM Navigate to the directory where the script is located (your repo)
cd /d "%~dp0"

REM Run Git commands
git add .
git commit -m "Generic update"
git push

REM Pause so you can see any output/errors
pause