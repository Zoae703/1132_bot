#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "$0")/.." && pwd)"
output="${TMPDIR:-/tmp}/stm32_protocol_host_test"
handler_output="${TMPDIR:-/tmp}/stm32_protocol_handler_host_test"
thruster_config_output="${TMPDIR:-/tmp}/thruster_config_host_test"
pca9685_output="${TMPDIR:-/tmp}/pca9685_host_test"
ms5837_output="${TMPDIR:-/tmp}/ms5837_host_test"
actuator_output="${TMPDIR:-/tmp}/actuator_startup_test"

g++ -std=c++17 -Wall -Wextra -Werror \
  -I"$repo_root/Modules/Protocol/include" \
  -I"$repo_root/protocol/shared" \
  -x c++ \
  "$repo_root/tests/stm32_protocol_host_test.cpp" \
  "$repo_root/Modules/Protocol/src/binary_protocol.cpp" \
  "$repo_root/protocol/shared/protocol_crc.c" \
  -o "$output"

"$output"

g++ -std=c++17 -Wall -Wextra -Werror \
  -I"$repo_root/tests/host_stubs" \
  -I"$repo_root/Modules/Protocol/include" \
  -I"$repo_root/Modules/DataBus/include" \
  -I"$repo_root/protocol/shared" \
  -x c++ \
  "$repo_root/tests/stm32_protocol_handler_host_test.cpp" \
  "$repo_root/Modules/Protocol/src/binary_protocol.cpp" \
  "$repo_root/Modules/Protocol/src/protocol_handler.cpp" \
  "$repo_root/Modules/DataBus/src/robot_data.cpp" \
  "$repo_root/protocol/shared/protocol_crc.c" \
  -o "$handler_output"

"$handler_output"

g++ -std=c++17 -Wall -Wextra -Werror \
  -I"$repo_root/tests/host_stubs" \
  -I"$repo_root/Modules/MotorControl/include" \
  -I"$repo_root/Modules/PIDController/include" \
  -I"$repo_root/Modules/DataBus/include" \
  "$repo_root/tests/thruster_config_host_test.cpp" \
  "$repo_root/Modules/MotorControl/src/motor_control.cpp" \
  "$repo_root/Modules/PIDController/src/PID.cpp" \
  "$repo_root/Modules/DataBus/src/robot_data.cpp" \
  -o "$thruster_config_output"

"$thruster_config_output"

g++ -std=c++17 -Wall -Wextra -Werror \
  -I"$repo_root/tests/host_stubs" \
  -I"$repo_root/Modules/PCA9685Driver" \
  -I"$repo_root/Modules/PCA9685Driver/include" \
  -I"$repo_root/Modules/DataBus" \
  -I"$repo_root/Modules/DataBus/include" \
  "$repo_root/tests/pca9685_host_test.cpp" \
  "$repo_root/Modules/PCA9685Driver/src/PCA9685.cpp" \
  "$repo_root/Modules/DataBus/src/robot_data.cpp" \
  -o "$pca9685_output"

"$pca9685_output"

g++ -std=c++17 -Wall -Wextra -Werror \
  -I"$repo_root/tests/host_stubs" \
  -I"$repo_root/Modules/MS5837Sensor" \
  -I"$repo_root/Modules/MS5837Sensor/include" \
  -I"$repo_root/Modules/DataBus" \
  -I"$repo_root/Modules/DataBus/include" \
  "$repo_root/tests/ms5837_host_test.cpp" \
  "$repo_root/Modules/MS5837Sensor/src/MS5837.cpp" \
  "$repo_root/Modules/DataBus/src/robot_data.cpp" \
  -o "$ms5837_output"

"$ms5837_output"

g++ -std=c++17 -Wall -Wextra -Werror \
  -I"$repo_root/tests/host_stubs" \
  -I"$repo_root/Modules/DataBus" \
  -I"$repo_root/Modules/DataBus/include" \
  -x c++ \
  "$repo_root/tests/actuator_startup_test.cpp" \
  "$repo_root/Modules/DataBus/src/robot_data.cpp" \
  -o "$actuator_output"

"$actuator_output"
