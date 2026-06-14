# Preview or build the URLab documentation site locally.
#
# Usage:
#   Scripts/docs.ps1            # live preview at http://localhost:8000 (auto-reload)
#   Scripts/docs.ps1 build      # static strict build into ./site
#
# On first run this creates an isolated .venv-docs and installs the docs
# dependencies (mkdocs-material, matching CI). It is kept separate from any
# global mkdocs so a broken global install can't interfere.
param([string]$Action = "serve")
$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
$venv = Join-Path $root ".venv-docs"
$py = Join-Path $venv "Scripts/python.exe"

if (-not (Test-Path $py)) {
    Write-Host "Creating docs venv at $venv ..." -ForegroundColor Cyan
    python -m venv $venv
    & $py -m pip install --quiet --upgrade pip
    & $py -m pip install --quiet -r (Join-Path $root "requirements-docs.txt")
}

Push-Location $root
try {
    if ($Action -eq "build") {
        & $py -m mkdocs build --strict
    }
    else {
        Write-Host "Serving docs at http://localhost:8000 (Ctrl+C to stop)" -ForegroundColor Green
        & $py -m mkdocs serve
    }
}
finally { Pop-Location }
