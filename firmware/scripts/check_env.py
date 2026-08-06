#!/usr/bin/env python3
"""STM32 development environment check script.

Checks that all required tools for building, flashing, and debugging
an STM32 project are available via system PATH.

Returns 0 if all essential tools are found, non-zero otherwise.

Usage:
    python3 scripts/check_env.py        # Linux
    python scripts/check_env.py         # Windows
"""

import shutil
import subprocess
import sys
from pathlib import Path

REQUIRED_TOOLS = [
    ("arm-none-eabi-gcc", "ARM GCC Compiler", True, ["--version"]),
    ("arm-none-eabi-g++", "ARM G++ Compiler", True, ["--version"]),
    ("arm-none-eabi-objcopy", "ARM objcopy", True, ["--version"]),
    ("arm-none-eabi-size", "ARM size", True, ["--version"]),
    ("cmake", "CMake", True, ["--version"]),
    ("ninja", "Ninja", True, ["--version"]),
    ("openocd", "OpenOCD", True, ["--version"]),
    ("git", "Git", True, ["--version"]),
]

OPTIONAL_TOOLS = [
    ("STM32CubeMX", "STM32CubeMX", False, None),
]


def run_version(cmd, args):
    try:
        result = subprocess.run(
            [cmd] + args, capture_output=True, text=True, timeout=10
        )
        output = result.stdout + result.stderr
        for line in output.strip().splitlines():
            if line.strip():
                return line.strip()
        return "(no output)"
    except Exception as e:
        return f"(error: {e})"


def check_tool(cmd, display, required, version_args):
    path = shutil.which(cmd)
    if path:
        version_str = ""
        if version_args:
            version_str = run_version(path, version_args)
            print(f"  [OK] {display}")
            print(f"       Path   : {path}")
            print(f"       Version: {version_str}")
        else:
            print(f"  [OK] {display}")
            print(f"       Path   : {path}")
        return True
    else:
        if required:
            print(f"  [MISSING] {display} — REQUIRED but not found in PATH")
        else:
            print(f"  [WARN] {display} — not found in PATH (optional)")
        return not required


def check_gdb():
    arm_gdb = shutil.which("arm-none-eabi-gdb")
    multiarch = shutil.which("gdb-multiarch")
    if arm_gdb:
        version = run_version(arm_gdb, ["--version"])
        print(f"  [OK] ARM GDB (arm-none-eabi-gdb)")
        print(f"       Path   : {arm_gdb}")
        print(f"       Version: {version}")
        return True
    elif multiarch:
        version = run_version(multiarch, ["--version"])
        print(f"  [OK] GDB (gdb-multiarch) — fallback, arm-none-eabi-gdb not found")
        print(f"       Path   : {multiarch}")
        print(f"       Version: {version}")
        return True
    else:
        print("  [MISSING] ARM GDB — neither arm-none-eabi-gdb nor gdb-multiarch found")
        return False


def check_multiple_openocd():
    import os
    paths = os.environ.get("PATH", "").split(os.pathsep)
    found = []
    for p in paths:
        candidate = os.path.join(p, "openocd")
        if os.path.isfile(candidate) and os.access(candidate, os.X_OK):
            found.append(candidate)
    if len(found) > 1:
        # /bin is symlink to /usr/bin on modern Linux — filter duplicates
        unique = []
        for f in found:
            real = os.path.realpath(f)
            if real not in unique:
                unique.append(real)
                if len(unique) > 1:
                    print(f"\n  [WARN] Multiple OpenOCD found in PATH:")
                    for u in unique:
                        print(f"         {u}")
                    print(f"         The first one will be used: {found[0]}")
                    return
        if len(unique) > 1:
            print(f"\n  [WARN] Multiple OpenOCD found in PATH:")
            for u in unique:
                print(f"         {u}")
            print(f"         The first one will be used: {found[0]}")


def check_ioc():
    project_dir = Path(__file__).resolve().parent.parent
    ioc_files = list(project_dir.glob("*.ioc"))
    if ioc_files:
        for f in ioc_files:
            print(f"  [OK] .ioc file: {f.name}")
    else:
        print("  [WARN] No .ioc file found in project root")


def check_cubemx():
    import os
    path = shutil.which("STM32CubeMX")
    if not path:
        candidates = [
            "/opt/st/stm32cubemx/STM32CubeMX",
            os.path.expanduser("~/STMicroelectronics/STM32CubeMX/STM32CubeMX"),
        ]
        for c in candidates:
            if os.path.isfile(c):
                path = c
                break
    if path:
        print(f"  [OK] STM32CubeMX: {path}")
    else:
        print("  [WARN] STM32CubeMX not installed — .ioc cannot be edited on this machine")


def main():
    print("=" * 60)
    print("  STM32 CMake Template — Environment Check")
    print("=" * 60)
    errors = 0

    print("\n[1] Required Build Tools")
    print("-" * 40)
    for cmd, display, required, vargs in REQUIRED_TOOLS:
        if not check_tool(cmd, display, required, vargs):
            errors += 1

    print("\n[2] Debugger")
    print("-" * 40)
    if not check_gdb():
        errors += 1

    print("\n[3] OpenOCD Details")
    print("-" * 40)
    check_multiple_openocd()

    print("\n[4] Project Files")
    print("-" * 40)
    check_ioc()

    print("\n[5] Optional Tools")
    print("-" * 40)
    check_cubemx()

    print("\n" + "=" * 60)
    if errors == 0:
        print("  RESULT: All essential tools found. Ready to build.")
        print("=" * 60)
        return 0
    else:
        print(f"  RESULT: {errors} essential tool(s) missing.")
        print("=" * 60)
        return 1


if __name__ == "__main__":
    sys.exit(main())
