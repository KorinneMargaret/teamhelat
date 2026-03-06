@echo off
REM Navigate to the directory where the script is located (your repo)
cd /d "%~dp0"

REM Pull latest changes from remote
git pull

REM Pause so you can see any output/errors
pause