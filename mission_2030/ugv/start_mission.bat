@echo off
setlocal enabledelayedexpansion

:: ============================================================
::  UGV MISSION LAUNCHER (WINDOWS)
:: ============================================================

set "SCRIPT_DIR=%~dp0"
set "REPO_ROOT=%SCRIPT_DIR%..\.."
for %%i in ("%REPO_ROOT%") do set "REPO_ROOT=%%~fi"
set "MISSION_ROOT=%REPO_ROOT%\mission_2030"
set "VENV_PATH=%REPO_ROOT%\.venv"

echo =======================================================
echo    OPERATION TOUCHDOWN - UGV LAUNCHER (Windows)
echo =======================================================
echo Repo root: %REPO_ROOT%

:: ------ Create/Activate Virtual Environment ------
if not exist "%VENV_PATH%" (
    echo Virtual environment not found. Creating one at %VENV_PATH%...
    python -m venv "%VENV_PATH%"
    if errorlevel 1 (
        echo ERROR: Python not found or failed to create venv.
        pause
        exit /b 1
    )
    echo Installing dependencies from pyproject.toml...
    "%VENV_PATH%\Scripts\pip" install --upgrade pip
    "%VENV_PATH%\Scripts\pip" install -e "%REPO_ROOT%"
) else (
    echo Using existing virtual environment at %VENV_PATH%
)

:: Activate the venv
call "%VENV_PATH%\Scripts\activate"

:: ------ Export PYTHONPATH ------
set "PYTHONPATH=%REPO_ROOT%"

:menu
cls
echo =======================================================
echo    OPERATION TOUCHDOWN - UGV LAUNCHER (Windows)
echo =======================================================
echo.
echo Select what to run:
echo.
echo --- COMPETITION MISSION ---
echo [1]  UGV Runner - waits for UAV destination, drives, avoids obstacles
echo.
echo --- HARDWARE TESTS (pair with matching drone test) ---
echo [t3]  Test 03 - Receive UAV command -> drive 5 ft
echo [t4]  Test 04 - Receive UAV destination vector -> drive to ArUco
echo [t5]  Test 05 - Forward drive (pair with drone test 05)
echo [t6]  Test 06 - Turn left  (pair with drone test 06)
echo [t7]  Test 07 - Turn right (pair with drone test 07)
echo.
echo [q]  Quit
echo.
set /p CHOICE="Your choice: "

if "%CHOICE%"=="1" set "SCRIPT=%MISSION_ROOT%\ugv\ugv_runner.py"
if "%CHOICE%"=="t3" set "SCRIPT=%MISSION_ROOT%\dennis_test\groundvehicle\03_move_ugv_5ft.py"
if "%CHOICE%"=="t4" set "SCRIPT=%MISSION_ROOT%\dennis_test\groundvehicle\04_ugv_to_aruco.py"
if "%CHOICE%"=="t5" set "SCRIPT=%MISSION_ROOT%\dennis_test\groundvehicle\05_fly_side_by_side_forward.py"
if "%CHOICE%"=="t6" set "SCRIPT=%MISSION_ROOT%\dennis_test\groundvehicle\06_fly_side_by_side_left.py"
if "%CHOICE%"=="t7" set "SCRIPT=%MISSION_ROOT%\dennis_test\groundvehicle\07_fly_side_by_side_right.py"
if "%CHOICE%"=="q" exit /b 0

if defined SCRIPT (
    echo.
    echo Running: python %SCRIPT%
    python "%SCRIPT%"
    echo.
    echo Script finished.
    pause
    goto menu
) else (
    echo Invalid choice.
    pause
    goto menu
)
