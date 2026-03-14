#!/usr/bin/env bash
set -euo pipefail

# Root of the project (folder containing this script).
ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

echo "[WSV] Project root: $ROOT"

if [ ! -d "$ROOT/.venv" ] || [ ! -f "$ROOT/.venv/bin/activate" ]; then
  echo "[WSV] Creating (or repairing) Python virtual environment..."
  rm -rf "$ROOT/.venv"
  python3 -m venv "$ROOT/.venv"
fi

echo "[WSV] Activating virtual environment and installing Python dependencies..."
source "$ROOT/.venv/bin/activate"
pip install --upgrade pip >/dev/null
pip install pybind11 fastapi "uvicorn[standard]" >/dev/null

echo "[WSV] Configuring and building C++ core + pyawscr module..."

# Use pybind11 from the active virtualenv to configure CMake.
PYBIND11_DIR="$(python -m pybind11 --cmakedir 2>/dev/null || true)"
if [ -n "$PYBIND11_DIR" ]; then
  cmake -S "$ROOT" -B "$ROOT/build" -Dpybind11_DIR="$PYBIND11_DIR"
else
  cmake -S "$ROOT" -B "$ROOT/build"
fi
cmake --build "$ROOT/build" -j"$(nproc)"

export PYTHONPATH="$ROOT/build:$ROOT"

echo "[WSV] Starting FastAPI + simulation backend on port 8000..."
(
  cd "$ROOT"
  uvicorn bridge.server:app --host 0.0.0.0 --port 8000
) &
BACKEND_PID=$!

trap 'echo "[WSV] Stopping backend (PID $BACKEND_PID)"; kill $BACKEND_PID 2>/dev/null || true' INT TERM

echo "[WSV] Preparing web dashboard..."
cd "$ROOT/web"
if [ ! -d "node_modules" ]; then
  echo "[WSV] Installing npm dependencies..."
  npm install
fi

echo "[WSV] Starting Vite dev server on http://localhost:5173 ..."
echo "[WSV] Open the URL in your browser, draw a polygon on the map, then press 'Start Mission'."
npm run dev

