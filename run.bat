@echo off
setlocal enabledelayedexpansion
REM run.bat - TIMDR-Robot: instalacja + uruchomienie dashboardu lokalnie.
REM Wymaga Pythona 3.8+.
REM
REM Ta struktura (venv + pip install + pre-flight port check + pause na
REM koncu) jest identyczna jak w fusion-tools/run.bat i
REM SYNOPTYK-ARCTIC/run.bat z tego samego zestawu repo - ten sam,
REM sprawdzony wzorzec, w tym pauza na koncu, ktora pozwala zobaczyc
REM traceback zamiast znikajacego okna konsoli przy podwojnym kliknieciu.

cd /d "%~dp0"

echo [1/5] Sprawdzanie Pythona...
where python >nul 2>nul
if errorlevel 1 (
    echo [BLAD] Nie znaleziono Pythona w PATH. Wymaga Pythona 3.8+.
    pause
    exit /b 1
)

echo [2/5] Sprawdzanie/tworzenie srodowiska .venv...
if not exist ".venv\Scripts\python.exe" (
    python -m venv .venv
    if errorlevel 1 (
        echo [BLAD] Nie udalo sie utworzyc .venv.
        pause
        exit /b 1
    )
)

echo [3/5] Instalacja zaleznosci...
".venv\Scripts\python.exe" -m pip install -q --upgrade pip
".venv\Scripts\python.exe" -m pip install -q -r requirements.txt
if errorlevel 1 (
    echo [BLAD] Instalacja zaleznosci nie powiodla sie - patrz traceback powyzej.
    pause
    exit /b 1
)

echo [4/5] Sprawdzanie portu 8000...
netstat -ano | findstr ":8000" | findstr "LISTENING" >nul 2>nul
if not errorlevel 1 (
    echo [UWAGA] Port 8000 jest juz zajety - prawdopodobnie poprzednia
    echo         sesja serwera nie zostala poprawnie zamknieta. Zamknij
    echo         proces uzywajacy portu 8000 i uruchom run.bat ponownie.
    pause
    exit /b 1
)

echo [5/5] Uruchamianie dashboardu na http://127.0.0.1:8000 ...
start "" http://127.0.0.1:8000
".venv\Scripts\python.exe" -m uvicorn api:app --host 127.0.0.1 --port 8000
set UVICORN_EXIT=%errorlevel%
echo.
if not "%UVICORN_EXIT%"=="0" (
    echo [BLAD] Serwer zakonczyl sie z kodem %UVICORN_EXIT% - patrz traceback powyzej.
) else (
    echo Serwer zatrzymany.
)
pause
