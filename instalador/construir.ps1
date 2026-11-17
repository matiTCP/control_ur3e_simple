# Construye el instalador de «Control UR3e»:
#   1) entorno de Python aparte con MediaPipe + PyInstaller (fuera de OneDrive, se reutiliza),
#   2) PyInstaller -> carpeta con "Control UR3e.exe" y todo lo que necesita (no hace falta Python),
#   3) Inno Setup -> salida\Instalar_Control_UR3e_<versión>.exe
#
# Uso (desde la raíz del repositorio):
#   powershell -ExecutionPolicy Bypass -File instalador\construir.ps1 [-Version 1.0.0]
# Requisitos: Python 3.12+ (python en el PATH) e Inno Setup 6 (winget install JRSoftware.InnoSetup).
# Si falta modelos\hand_landmarker.task, se descarga el modelo oficial de MediaPipe.

param([string]$Version = "1.0.0")
$ErrorActionPreference = "Stop"
$aqui = $PSScriptRoot
$repo = Join-Path $aqui ".." | Resolve-Path
$trabajo = Join-Path $env:LOCALAPPDATA "ControlUR3e-build"      # venv y archivos temporales (pesan ~1 GB)
$venv = Join-Path $trabajo "venv"
$py = Join-Path $venv "Scripts\python.exe"

# 1) Entorno de compilación
if (-not (Test-Path $py)) {
    Write-Host "Creando entorno de Python en $venv ..."
    python -m venv $venv
    & $py -m pip install --upgrade pip
}
& $py -m pip install --upgrade "mediapipe>=1.2" pyinstaller
if ($LASTEXITCODE) { throw "pip falló" }

# 2) PyInstaller (modo carpeta: arranca más rápido que un único .exe)
$modelo = Join-Path $repo "modelos\hand_landmarker.task"
if (-not (Test-Path $modelo)) {
    Write-Host "Descargando el modelo de MediaPipe en $modelo ..."
    New-Item -ItemType Directory -Force (Split-Path $modelo) | Out-Null
    Invoke-WebRequest -Uri "https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task" -OutFile $modelo
}
$dist = Join-Path $trabajo "dist"
& $py -m PyInstaller --noconfirm --clean --windowed `
    --name "Control UR3e" `
    --icon (Join-Path $aqui "icono.ico") `
    --add-data "$(Join-Path $aqui 'icono.ico');." `
    --add-data "$modelo;modelos" `
    --collect-all mediapipe `
    --copy-metadata opencv-contrib-python `
    --distpath $dist --workpath (Join-Path $trabajo "build") --specpath $trabajo `
    (Join-Path $repo "control_ur3e.py")
if ($LASTEXITCODE) { throw "PyInstaller falló" }

# 3) Inno Setup
$iscc = @("$env:LOCALAPPDATA\Programs\Inno Setup 6\ISCC.exe",
          "${env:ProgramFiles(x86)}\Inno Setup 6\ISCC.exe",
          "$env:ProgramFiles\Inno Setup 6\ISCC.exe") | Where-Object { Test-Path $_ } | Select-Object -First 1
if (-not $iscc) { throw "No encuentro Inno Setup 6 (winget install JRSoftware.InnoSetup)" }
& $iscc "/DAppVersion=$Version" "/DDistDir=$(Join-Path $dist 'Control UR3e')" (Join-Path $aqui "control_ur3e.iss")
if ($LASTEXITCODE) { throw "Inno Setup falló" }
Write-Host "`nListo: $(Join-Path $aqui "salida\Instalar_Control_UR3e_$Version.exe")"
