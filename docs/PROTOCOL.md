# UART Binary Protocol v2

USART6 is reserved for machine communication in the normal FreeRTOS firmware.
It must not carry unframed text. Standalone bring-up builds are separate modes
and do not run the binary stack.

## Frame format

All multi-byte values are little-endian.

| Field | Bytes | Value |
| --- | ---: | --- |
| Magic | 2 | `AA 55` |
| Version | 1 | `02` |
| Message type | 1 | See `protocol/shared/protocol.h` |
| Sequence | 2 | Unsigned, wraps at 65535 |
| Payload length | 2 | 0 to 240 |
| Payload | N | Message-specific packed bytes |
| CRC16 | 2 | CRC-16/CCITT-FALSE over header and payload |

The fixed header is 8 bytes; total framing overhead including CRC is 10 bytes.
The C packed structures and Python `struct` formats are kept byte-for-byte
compatible and are protected by size tests/static assertions.

Version 2 is intentionally not wire-compatible with the earlier provisional
version because NACK and STATUS_REPORT changed. Orange Pi and STM32 firmware
must therefore be upgraded together. A frame with any other version is rejected
and the parser continues searching for the next `AA 55` sync marker.

## Command validation

Every command has an exact payload length. Empty commands must carry zero bytes;
fixed-size commands must match the corresponding packed structure exactly.
Length is checked before any structure access. Channels, enums, finite floating
point values, PWM limits, and timeouts are then checked independently.

Invalid commands receive a sequence-aware NACK. Unknown message types are not
silently treated as valid commands.

For `SET_PWM`, protocol v2 accepts channels `0..7`, the configured test range
`1450..1550us`, and a duration of `200..2000ms`. Only MANUAL_TEST may accept the
command, and a different channel is rejected as busy until all outputs have
been neutralized.

## ACK and NACK

ACK payload (`<H`, 2 bytes):

| Field | Bytes |
| --- | ---: |
| Acknowledged request sequence | 2 |

NACK payload (`<HBB`, 4 bytes):

| Field | Bytes |
| --- | ---: |
| Rejected request sequence | 2 |
| Original message type | 1 |
| Rejection reason | 1 |

The host resolves pending requests only by rejected/acknowledged sequence. A
NACK for an unknown sequence is diagnostic information and cannot fail a
different request of the same type.

Additional v2 rejection reasons are:

- `INVALID_PAYLOAD_LENGTH = 10`
- `UNSUPPORTED_MESSAGE = 11`
- `INVALID_VALUE = 12`

## Status report

STATUS_REPORT is 24 bytes (`<BB8hHHBB`): safety state, flags, eight confirmed
PWM values, protocol error count, missed-heartbeat count, neutral reason, and
active manual-test channel (`0xFF` means none).

Neutral reasons are `NONE`, `COMMAND`, `PWM_COMMAND_TIMEOUT`, `COMM_LOST`,
`DISARM`, `EMERGENCY_STOP`, `LAST_CLIENT_DISCONNECTED`, and `FAULT`.

An ACK means that a command was accepted. Only a later STATUS_REPORT is proof of
the current safety state and confirmed PWM output.

## Streaming and recovery

UART callback boundaries have no protocol meaning. ISR bytes are copied into a
FreeRTOS stream buffer; CommunicationTask owns parsing and dispatch. The parser
supports partial frames, multiple frames per read, leading noise, CRC failure,
and byte-by-byte resynchronization. CRC or length failure never disables future
frame parsing.

The transmit path is also single-owner: producers enqueue framed messages and
CommunicationTask is the only normal-runtime caller that writes USART6. Control
responses use the high-priority queue; routine telemetry cannot starve ACK,
NACK, heartbeat ACK, or safety events.
