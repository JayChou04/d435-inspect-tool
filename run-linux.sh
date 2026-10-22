#!/bin/bash
# Start the D435 inspector on Linux (x86_64, or aarch64 boards such as Jetson).
#
#   ./run-linux.sh [options]   options go to d435_inspect.py: --check, --list, --no-fw-update, ...
#
# 1. First run: creates .venv in this folder and installs requirements.txt into it. If no system
#    Python has a pyrealsense2 wheel, uv (downloaded into .uv/) fetches one into .uv/. Nothing is
#    installed or cached outside the folder.
# 2. Reports which USB port the camera is on and whether its /dev nodes are accessible (usb_ports.py).
# 3. Starts the inspector as this user when the RealSense udev rules are installed and the camera is
#    accessible, otherwise with sudo. -B stops Python writing root-owned .pyc files.
cd "$(dirname "$0")" || exit 1
[ "$(uname -s)" = Linux ] || { echo "This launcher is for Linux; on macOS use ./run-mac.sh" >&2; exit 1; }
PY=.venv/bin/python
PLATFORM="$(uname -s)-$(uname -m)"
case "$(uname -m)" in   # Python versions with a pyrealsense2 2.56.5 wheel, preferred first
    x86_64)        VERSIONS="3.12 3.11 3.10 3.13 3.9"; UV_TARGET=x86_64-unknown-linux-gnu ;;
    aarch64|arm64) VERSIONS="3.10 3.9";                UV_TARGET=aarch64-unknown-linux-gnu ;;
    *) echo "pyrealsense2 has no Linux wheel for $(uname -m)." >&2; exit 1 ;;
esac
export UV_NO_CACHE=1 UV_PYTHON_INSTALL_DIR="$PWD/.uv/python"   # anything uv fetches stays in here

system_python() {   # a system interpreter of a supported version that can create a venv
    local v p
    for v in $VERSIONS; do
        for p in "python$v" python3; do
            p=$(command -v "$p") || continue
            [ "$("$p" -c 'import sys; print("%d.%d" % sys.version_info[:2])')" = "$v" ] || continue
            if "$p" -c 'import ensurepip, venv' 2>/dev/null; then
                echo "$p"
                return 0
            fi
        done
    done
    return 1
}

uv_bin() {   # uv from PATH, or a copy downloaded once into .uv/bin
    if command -v uv; then
        return 0
    fi
    if [ ! -x .uv/bin/uv ]; then
        local url="https://github.com/astral-sh/uv/releases/latest/download/uv-$UV_TARGET.tar.gz"
        echo "No usable Python ($VERSIONS) found; downloading uv into .uv/ to fetch one." >&2
        mkdir -p .uv/bin
        { curl -fsSL "$url" 2>/dev/null || wget -qO- "$url"; } | tar -xz -C .uv/bin --strip-components=1 \
            || return 1
    fi
    echo "$PWD/.uv/bin/uv"
}

create_venv() {
    local py uv
    echo "Creating .venv (first run on this machine)..."
    rm -rf .venv
    if py=$(system_python); then
        "$py" -m venv .venv || return 1
    else
        uv=$(uv_bin) || { echo "Could not get uv; install python3-venv or uv and try again." >&2; return 1; }
        "$uv" venv --seed --python "${VERSIONS%% *}" .venv || return 1
    fi
    echo "$PLATFORM" > .venv/.platform
}

install_requirements() {
    echo "Installing Python packages into .venv..."
    "$PY" -m pip install --disable-pip-version-check --no-cache-dir -r requirements.txt \
        && cksum < requirements.txt > .venv/.requirements
}

# A .venv copied from another machine (macOS, other CPU) cannot be reused.
if ! "$PY" -c '' 2>/dev/null || [ "$(cat .venv/.platform 2>/dev/null)" != "$PLATFORM" ]; then
    create_venv || exit 1
fi
[ "$(cat .venv/.requirements 2>/dev/null)" = "$(cksum < requirements.txt)" ] || install_requirements || exit 1
if ! "$PY" -B -c 'import numpy, cv2, pyrealsense2'; then
    cat >&2 <<'EOF'
The Python packages did not load (error above). OpenCV usually needs these system libraries:
  Debian/Ubuntu:  sudo apt install libgl1 libglib2.0-0
  Fedora/RHEL:    sudo dnf install mesa-libGL glib2
EOF
    exit 1
fi

for arg in "$@"; do   # help needs neither the USB check nor root
    case "$arg" in
        -h|--help) exec "$PY" -B d435_inspect.py --launcher "$0" --help ;;
    esac
done

"$PY" -B usb_ports.py
if [ $? -eq 0 ] || [ "$(id -u)" -eq 0 ]; then   # usb_ports.py exits 10 when root is needed
    exec "$PY" -B d435_inspect.py --launcher "$0" "$@"
fi
# Under sudo the GUI still needs the user's display and its X authority cookie.
if [ -z "${XAUTHORITY:-}" ] && [ -f "$HOME/.Xauthority" ]; then
    export XAUTHORITY="$HOME/.Xauthority"
fi
export QT_X11_NO_MITSHM=1   # root cannot share memory with the user's X server
exec sudo --preserve-env=DISPLAY,XAUTHORITY,WAYLAND_DISPLAY,QT_X11_NO_MITSHM "$PY" -B d435_inspect.py --launcher "$0" "$@"
