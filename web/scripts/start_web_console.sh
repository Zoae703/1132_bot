#!/bin/bash
# ============================================================================
# 1132_bot Web Console — startup script
#
# Usage:
#   ./scripts/start_web_console.sh                       # Real hardware (default)
#   ./scripts/start_web_console.sh --simulate            # Simulation mode
#   ./scripts/start_web_console.sh --serial /dev/ttyS1   # Custom serial port
#   ./scripts/start_web_console.sh --port 9000           # Custom web port
# ============================================================================

set -euo pipefail

cd "$(dirname "$0")/.."

# ---- Defaults ----
MODE="from validated config"
CONFIG="${ROV_CONFIG:-opi_console/config.yaml}"
ARGS=()

require_value() {
    if [[ $# -lt 2 || -z "${2:-}" ]]; then
        echo "ERROR: $1 requires a value" >&2
        exit 2
    fi
}

# ---- Parse args ----
while [[ $# -gt 0 ]]; do
    case "$1" in
        --simulate)
            ARGS+=("--simulate")
            MODE="SIMULATION"
            shift
            ;;
        --hardware)
            ARGS+=("--hardware")
            MODE="REAL HARDWARE"
            shift
            ;;
        --serial)
            require_value "$@"
            ARGS+=("--serial" "$2")
            shift 2
            ;;
        --baud)
            require_value "$@"
            ARGS+=("--baud" "$2")
            shift 2
            ;;
        --host)
            require_value "$@"
            ARGS+=("--web-host" "$2")
            shift 2
            ;;
        --port)
            require_value "$@"
            ARGS+=("--web-port" "$2")
            shift 2
            ;;
        --config)
            require_value "$@"
            CONFIG="$2"
            shift 2
            ;;
        --debug)
            ARGS+=("--log-level" "DEBUG")
            shift
            ;;
        --help|-h)
            echo "Usage: $0 [--simulate|--hardware] [--serial DEV] [--baud N] [--port N] [--debug]"
            echo ""
            echo "Options:"
            echo "  --simulate    Run with simulated STM32 (no hardware needed)"
            echo "  --hardware    Force real STM32 mode"
            echo "  --serial DEV  Override serial port from config"
            echo "  --baud N      Override baud rate (firmware requires 115200)"
            echo "  --port N      Override web server port"
            echo "  --host HOST   Override web server host"
            echo "  --config FILE YAML config file (default: opi_console/config.yaml)"
            echo "  --debug       Enable DEBUG logging"
            echo ""
            echo "Environment: ROV_CONFIG, ROV_SIMULATE, ROV_SERIAL_PORT,"
            echo "             ROV_SERIAL_BAUD, ROV_WEB_HOST, ROV_WEB_PORT, ROV_LOG_LEVEL"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage."
            exit 1
            ;;
    esac
done

# ---- Print banner ----
echo "╔══════════════════════════════════════════════════════════╗"
echo "║         1132_bot Underwater ROV Debug Console           ║"
echo "╠══════════════════════════════════════════════════════════╣"
printf "║  MODE: %-48s ║\n" "$MODE"
printf "║  Config: %-46s ║\n" "$CONFIG"
echo "╚══════════════════════════════════════════════════════════╝"
echo ""

# ---- Check Python ----
if ! command -v python3 &> /dev/null; then
    echo "ERROR: python3 not found"
    exit 1
fi

# ---- Activate virtual environment if present ----
if [ -f "venv/bin/activate" ]; then
    source venv/bin/activate
    echo "Activated virtual environment"
fi

# ---- Install dependencies if needed ----
if [ -f "requirements.txt" ]; then
    python3 -c "import fastapi, uvicorn, yaml, serial, serial_asyncio" 2>/dev/null || {
        echo "Installing dependencies..."
        if ! python3 -m pip --version >/dev/null 2>&1; then
            echo "ERROR: Python pip is not available."
            echo ""
            echo "On Arch Linux / Orange Pi, install pip and virtualenv first:"
            echo "  sudo pacman -Syu --needed python-pip python-virtualenv python-platformdirs"
            echo "  python3 -m virtualenv venv"
            echo "  source venv/bin/activate"
            echo "  python3 -m pip install -r requirements.txt"
            exit 1
        fi
        python3 -m pip install -r requirements.txt
    }
fi

# ---- Launch ----
export PYTHONPATH="$(pwd):$(pwd)/protocol/shared${PYTHONPATH:+:${PYTHONPATH}}"

echo "Starting console..."
python3 -m opi_console.main \
    --config "$CONFIG" \
    "${ARGS[@]}"
