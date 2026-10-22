#!/bin/bash
# Start the D435 inspector on macOS (Apple Silicon).
#
#   ./run-mac.sh [options]     options go to d435_inspect.py: --check, --list, --no-fw-update, ...
#
# 1. First run: creates .venv in this folder and installs requirements.txt into it. Nothing is
#    installed or cached outside the folder.
# 2. Reports which USB-C port the camera is on (usb_ports.py).
# 3. Starts the inspector with sudo: on macOS librealsense reaches the camera through libusb, which
#    needs root. -B stops Python writing root-owned .pyc files, so the folder stays deletable.
cd "$(dirname "$0")" || exit 1
[ "$(uname -s)" = Darwin ] || { echo "This launcher is for macOS; on Linux use ./run-linux.sh" >&2; exit 1; }
[ "$(uname -m)" = arm64 ] || { echo "pyrealsense2-macosx only exists for Apple Silicon Macs." >&2; exit 1; }
PY=.venv/bin/python
export UV_NO_CACHE=1 UV_PYTHON_INSTALL_DIR="$PWD/.uv/python"   # anything uv fetches stays in here

create_venv() {
    echo "Creating .venv (first run)..."
    rm -rf .venv
    if command -v uv >/dev/null; then
        uv venv --seed --python ">=3.9,<3.15" .venv
        return
    fi
    local p
    for p in python3.12 python3.13 python3.11 python3.10 python3; do
        if command -v "$p" >/dev/null; then
            "$p" -m venv .venv
            return
        fi
    done
    echo "No python3 found: install Python 3.9+ (python.org or 'brew install python')." >&2
    return 1
}

install_requirements() {
    echo "Installing Python packages into .venv..."
    if "$PY" -m pip --version >/dev/null 2>&1; then
        "$PY" -m pip install --disable-pip-version-check --no-cache-dir -r requirements.txt
    else   # a venv made by uv without --seed has no pip
        uv pip install --python "$PY" -r requirements.txt
    fi && cksum < requirements.txt > .venv/.requirements
}

"$PY" -c '' 2>/dev/null || create_venv || exit 1
[ "$(cat .venv/.requirements 2>/dev/null)" = "$(cksum < requirements.txt)" ] || install_requirements || exit 1
if ! "$PY" -B -c 'import numpy, cv2, pyrealsense2'; then
    echo "The Python packages did not load (error above). Delete .venv and run again." >&2
    exit 1
fi

for arg in "$@"; do   # help needs neither the USB check nor root
    case "$arg" in
        -h|--help) exec "$PY" -B d435_inspect.py --launcher "$0" --help ;;
    esac
done

"$PY" -B usb_ports.py
exec sudo "$PY" -B d435_inspect.py --launcher "$0" "$@"
