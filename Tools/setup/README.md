# PX4 Python build environment

PX4 firmware is compiled by CMake/Ninja and the C/C++ toolchain. Python is
used by the build-time generators and configuration tools, including
kconfiglib, menuconfig, uORB/DDS generation, and parameter metadata tools.

The repository-local .venv contains those Python tools. It is separate from
the ROS/Torch environment used by the companion uav_ws repository.

## First setup

Install operating-system and compiler dependencies once. On Ubuntu, this also
installs python3-venv:

    bash Tools/setup/ubuntu.sh --no-nuttx

On macOS, use the existing Homebrew setup:

    bash Tools/setup/macos.sh --sim-tools

Then create or update the PX4 Python environment:

    bash Tools/setup/setup-venv

The script installs Tools/setup/requirements.txt into
PX4-Autopilot/.venv and verifies kconfiglib and menuconfig.

## Build

Activate the PX4 environment in every shell used for a PX4 build:

    source .venv/bin/activate
    make px4_sitl gz_x500_vision

For a hardware target, use the same environment before the board target, for
example make cuav_7-nano_default -j2.

If the build directory was previously configured with another Python
interpreter, remove only that PX4 build directory once and configure again:

    rm -rf build/px4_sitl_default

The .venv and build/ directories are local artifacts and are ignored by Git.
Commit the setup scripts and dependency files instead.
