@echo off
title 6-DOF Robot Arm Digital Twin
echo Kaynnistetaan 6-DOF Robottikasivarren Digital Twin & OLP -sovellus...
cd /d "%~dp0digital_twin"
python main.py
if %ERRORLEVEL% NEQ 0 (
    echo.
    echo Sovelluksen ajamisessa tapahtui virhe.
    pause
)
