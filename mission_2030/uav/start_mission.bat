@echo off
setlocal enabledelayedexpansion

:: ============================================================
::  UAV MISSION LAUNCHER (WINDOWS)
:: ============================================================

set "SCRIPT_DIR=%~dp0"
set "REPO_ROOT=%SCRIPT_DIR%..\.."
for %%i in ("%REPO_ROOT%") do set "REPO_ROOT=%%~fi"
set "MISSION_ROOT=%REPO_ROOT%\mission_2030"
set "VENV_PATH=%REPO_ROOT%\.venv"

echo =======================================================
echo    OPERATION TOUCHDOWN - UAV LAUNCHER (Windows)
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
)

:: Activate the venv
call "%VENV_PATH%\Scripts\activate"

:: ------ Verify Dependencies ------
set "SENTINEL=%VENV_PATH%\venv_ready"
set "ENV_OK=true"

if not exist "%SENTINEL%" set "ENV_OK=false"
if "%ENV_OK%"=="true" (
    python -c "import past, numpy" >nul 2>&1
    if errorlevel 1 (
        echo Environment health check failed (missing modules). Repairing...
        set "ENV_OK=false"
    )
)

if "%ENV_OK%"=="false" (
    echo Environment incomplete or corrupted. Installing dependencies...
    python -m pip install --upgrade pip
    python -m pip install -e "%REPO_ROOT%"
    if errorlevel 1 (
        echo ERROR: Installation failed.
        pause
        exit /b 1
    )
    echo. > "%SENTINEL%"
    echo Environment ready!
) else (
    echo Using existing virtual environment at %VENV_PATH%
)

:: ------ Export PYTHONPATH ------
set "PYTHONPATH=%REPO_ROOT%"

:menu
cls
echo =======================================================
echo    OPERATION TOUCHDOWN - UAV LAUNCHER (Windows)
echo =======================================================
echo.
echo Select what to run:
echo.
echo --- COMPETITION MISSIONS ---
echo [1]  Mission 1 - Takeoff, land on moving UGV, 30s ride
echo [2]  Mission 2 - Scan field, send coords to UGV, land on UGV
echo [3]  Mission 3 - Mission 2 + obstacle avoidance relay
echo.
echo --- HARDWARE TESTS ---
echo [t1]  Test 01 - Basic flight: arm / hover / land
echo [t2]  Test 02 - ZED camera + ArUco detection
echo [t3]  Test 03 - ESP32 V2V link: send command to UGV
echo [t4]  Test 04 - ZED point cloud to UGV destination
echo [t5]  Test 05 - Forward pacing with UGV
echo [t6]  Test 06 - Left sway pacing with UGV
echo [t7]  Test 07 - Right sway pacing with UGV
echo [t8]  Test 08 - Proportional centering hover (high, 1.3m)
echo [t9]  Test 09 - Proportional centering hover (low, 0.5m)
echo [t10] Test 10 - Full precision landing (LANDING_TARGET stream)
echo [t11] Test 11 - Command UGV to Arm and Avoid Obstacles
echo.
echo [q]  Quit
echo.
set /p CHOICE="Your choice: "

if "%CHOICE%"=="1" set "SCRIPT=%MISSION_ROOT%\uav\mission1_runner.py"
if "%CHOICE%"=="2" set "SCRIPT=%MISSION_ROOT%\uav\mission2_runner.py"
if "%CHOICE%"=="3" set "SCRIPT=%MISSION_ROOT%\uav\mission3_runner.py"
if "%CHOICE%"=="t1" set "SCRIPT=%MISSION_ROOT%\dennis_test\drone\01_basic_flight.py"
if "%CHOICE%"=="t2" set "SCRIPT=%MISSION_ROOT%\dennis_test\drone\02_aruco_land.py"
if "%CHOICE%"=="t3" set "SCRIPT=%MISSION_ROOT%\dennis_test\drone\03_move_ugv_5ft.py"
if "%CHOICE%"=="t4" set "SCRIPT=%MISSION_ROOT%\dennis_test\drone\04_ugv_to_aruco.py"
if "%CHOICE%"=="t5" set "SCRIPT=%MISSION_ROOT%\dennis_test\drone\05_fly_side_by_side_forward.py"
if "%CHOICE%"=="t6" set "SCRIPT=%MISSION_ROOT%\dennis_test\drone\06_fly_side_by_side_left.py"
if "%CHOICE%"=="t7" set "SCRIPT=%MISSION_ROOT%\dennis_test\drone\07_fly_side_by_side_right.py"
if "%CHOICE%"=="t8" set "SCRIPT=%MISSION_ROOT%\dennis_test\drone\08_center_hover_high.py"
if "%CHOICE%"=="t9" set "SCRIPT=%MISSION_ROOT%\dennis_test\drone\09_center_hover_low.py"
if "%CHOICE%"=="t10" set "SCRIPT=%MISSION_ROOT%\dennis_test\drone\10_precision_land.py"
if "%CHOICE%"=="t11" set "SCRIPT=%MISSION_ROOT%\dennis_test\drone\11_arm_ugv_avoid.py"
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
