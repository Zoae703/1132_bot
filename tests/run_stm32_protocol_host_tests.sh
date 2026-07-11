#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "$0")/.." && pwd)"
output="${TMPDIR:-/tmp}/stm32_protocol_host_test"
handler_output="${TMPDIR:-/tmp}/stm32_protocol_handler_host_test"

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
  "$repo_root/protocol/shared/protocol_crc.c" \
  -o "$handler_output"

"$handler_output"
