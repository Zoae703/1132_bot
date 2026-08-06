/**
 * @file    protocol.h
 * @brief   Binary communication protocol v2 — shared between STM32 and Orange Pi.
 *
 * Frame format (little-endian):
 *   [0xAA 0x55] [version:1] [type:1] [seq:2] [payload_len:2] [payload:N] [crc16:2]
 *
 * Header: 8 bytes.  Total framing overhead including CRC: 10 bytes.
 * Max payload: 240 bytes.
 * CRC16: CCITT (polynomial 0x1021, initial value 0xFFFF).
 *
 * This file is the SINGLE SOURCE OF TRUTH for message IDs and payload structures.
 * The Python mirror at protocol/shared/protocol.py MUST stay in sync.
 */

#ifndef PROTOCOL_SHARED_PROTOCOL_H
#define PROTOCOL_SHARED_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ------------------------------------------------------------------ */
/*  Frame constants                                                    */
/* ------------------------------------------------------------------ */

#define PROTO_MAGIC_0         0xAAU
#define PROTO_MAGIC_1         0x55U
#define PROTO_VERSION         0x02U
#define PROTO_FRAME_OVERHEAD  8U
#define PROTO_CRC_SIZE        2U
#define PROTO_MAX_PAYLOAD     240U
#define PROTO_BUF_SIZE        (PROTO_MAX_PAYLOAD + PROTO_FRAME_OVERHEAD + PROTO_CRC_SIZE)

/* ------------------------------------------------------------------ */
/*  Message types                                                      */
/* ------------------------------------------------------------------ */

typedef enum {
    /* 0x00 – 0x0F : Session / ACK */
    ProtoMsg_NOP              = 0x00,
    ProtoMsg_ACK              = 0x01,
    ProtoMsg_NACK             = 0x02,

    /* 0x10 – 0x1F : Arm / Safety */
    ProtoMsg_ARM              = 0x10,
    ProtoMsg_DISARM           = 0x11,
    ProtoMsg_EMERGENCY_STOP   = 0x12,
    ProtoMsg_RESET_ESTOP      = 0x13,
    ProtoMsg_ENTER_MANUAL     = 0x14,
    ProtoMsg_EXIT_MANUAL      = 0x15,

    /* 0x20 – 0x2F : PWM / Motor direct control */
    ProtoMsg_SET_PWM          = 0x20,
    ProtoMsg_SET_ALL_NEUTRAL  = 0x21,

    /* 0x30 – 0x3F : Float / Motion / PID commands */
    ProtoMsg_FLOAT_ON         = 0x30,
    ProtoMsg_FLOAT_OFF        = 0x31,
    ProtoMsg_ANGLE_ON         = 0x32,
    ProtoMsg_ANGLE_OFF        = 0x33,
    ProtoMsg_SET_DEPTH        = 0x34,
    ProtoMsg_SET_YAW          = 0x35,
    ProtoMsg_SET_MOTION       = 0x36,
    ProtoMsg_SET_BODY_COMMAND = 0x37,
    ProtoMsg_BODY_CONTROL_ON  = 0x38,
    ProtoMsg_BODY_CONTROL_OFF = 0x39,
    ProtoMsg_SET_MOTION_TUNING = 0x3A,
    ProtoMsg_SET_DEPTH_PID_TUNING = 0x3B,

    /* 0x40 – 0x4F : Query / Request */
    ProtoMsg_REQUEST_STATUS   = 0x40,
    ProtoMsg_REQUEST_SENSORS  = 0x41,
    ProtoMsg_REQUEST_MOTION_TUNING = 0x42,
    ProtoMsg_REQUEST_DEPTH_PID_TUNING = 0x43,
    ProtoMsg_REQUEST_DEPTH_CONTROL = 0x44,

    /* 0x80 – 0x8F : Telemetry reports (STM32 → OPi) */
    ProtoMsg_STATUS_REPORT    = 0x80,
    ProtoMsg_SENSOR_REPORT    = 0x81,
    ProtoMsg_MOTION_TUNING_REPORT = 0x82,
    ProtoMsg_DEPTH_PID_TUNING_REPORT = 0x83,
    ProtoMsg_DEPTH_CONTROL_REPORT = 0x84,

    /* 0x90 – 0x9F : Async events (STM32 → OPi) */
    ProtoMsg_LOG_MESSAGE      = 0x90,
    ProtoMsg_SAFETY_EVENT     = 0x91,

    /* 0xF0 – 0xFF : Heartbeat */
    ProtoMsg_HEARTBEAT        = 0xF0,
    ProtoMsg_HEARTBEAT_ACK    = 0xF1,
} ProtoMsgType;

/* ------------------------------------------------------------------ */
/*  Safety / robot state (reported in STATUS_REPORT and SAFETY_EVENT) */
/* ------------------------------------------------------------------ */

typedef enum {
    ProtoState_DISARMED       = 0,
    ProtoState_ARMED_IDLE     = 1,
    ProtoState_ARMED_ACTIVE   = 2,
    ProtoState_MANUAL_TEST    = 3,
    ProtoState_COMM_LOST      = 4,
    ProtoState_EMERGENCY_STOP = 5,
    ProtoState_FAULT          = 6,
} ProtoSafetyState;

/* ------------------------------------------------------------------ */
/*  NACK reason codes                                                  */
/* ------------------------------------------------------------------ */

typedef enum {
    ProtoNack_Unknown         = 0,
    ProtoNack_BadCRC          = 1,
    ProtoNack_BadState        = 2,
    ProtoNack_BadChannel      = 3,
    ProtoNack_BadPwmValue     = 4,
    ProtoNack_ChannelBusy     = 5,
    ProtoNack_EstopLocked     = 6,
    ProtoNack_NotArmed        = 7,
    ProtoNack_Timeout         = 8,
    ProtoNack_InternalError   = 9,
    ProtoNack_InvalidPayloadLength = 10,
    ProtoNack_UnsupportedMessage   = 11,
    ProtoNack_InvalidValue         = 12,
} ProtoNackReason;

/* ------------------------------------------------------------------ */
/*  Why outputs were most recently forced to neutral                  */
/* ------------------------------------------------------------------ */

typedef enum {
    ProtoNeutral_NONE                     = 0,
    ProtoNeutral_COMMAND                  = 1,
    ProtoNeutral_PWM_COMMAND_TIMEOUT      = 2,
    ProtoNeutral_COMM_LOST                = 3,
    ProtoNeutral_DISARM                   = 4,
    ProtoNeutral_EMERGENCY_STOP           = 5,
    ProtoNeutral_LAST_CLIENT_DISCONNECTED = 6,
    ProtoNeutral_FAULT                    = 7,
    ProtoNeutral_DEPTH_SENSOR             = 8,
    ProtoNeutral_STARTUP                  = 9,
} ProtoNeutralReason;

/* ------------------------------------------------------------------ */
/*  Safety event types                                                 */
/* ------------------------------------------------------------------ */

typedef enum {
    ProtoEvent_CommLost       = 0,
    ProtoEvent_CommRestored   = 1,
    ProtoEvent_EstopTriggered = 2,
    ProtoEvent_Fault          = 3,
    ProtoEvent_StateChange    = 4,
} ProtoSafetyEventType;

/* ------------------------------------------------------------------ */
/*  Payload structures  (packed, little-endian)                        */
/* ------------------------------------------------------------------ */

#pragma pack(push, 1)

/** SET_PWM payload: 6 bytes */
typedef struct {
    uint8_t  channel;         /* 0–7 */
    uint8_t  reserved;
    int16_t  pwm_us;          /* pulse width in microseconds */
    uint16_t timeout_ms;      /* per-command timeout (0 = use system default) */
} ProtoSetPwm;

/** SET_DEPTH payload: 4 bytes */
typedef struct {
    float target_depth_cm;
} ProtoSetDepth;

/** SET_YAW payload: 4 bytes */
typedef struct {
    float target_yaw_deg;
} ProtoSetYaw;

/** SET_MOTION payload: 1 byte */
typedef struct {
    uint8_t motion_state;     /* 0=STOP, 1=FLOAT, 2=FRONT, 3=BACK, 4=LEFT, 5=RIGHT, 6=CW, 7=CCW */
} ProtoSetMotion;

/** SET_BODY_COMMAND payload: 24 bytes, all axes normalized to [-1, +1] */
typedef struct {
    float surge;
    float sway;
    float heave;
    float roll;
    float pitch;
    float yaw;
} ProtoSetBodyCommand;

/**
 * SET_MOTION_TUNING / MOTION_TUNING_REPORT payload: 56 bytes.
 * Axis order is surge, sway, heave, roll, pitch, yaw.
 */
typedef struct {
    float axis_gain[6];
    float axis_max_output[6];
    float global_multiplier;
    uint16_t pwm_slew_rate_us_per_s;
    uint16_t command_timeout_ms;
} ProtoMotionTuning;

/**
 * SET_DEPTH_PID_TUNING / DEPTH_PID_TUNING_REPORT payload: 28 bytes.
 * Errors are expressed in centimetres and all limits/outputs in PWM us.
 */
typedef struct {
    float kp;
    float ki;
    float kd;
    float p_limit_us;
    float i_limit_us;
    float d_limit_us;
    float output_limit_us;
} ProtoDepthPidTuning;

/** STATUS_REPORT payload: 24 bytes */
typedef struct {
    uint8_t  safety_state;    /* ProtoSafetyState */
    uint8_t  flags;           /* bit0=control, bit1=float, bit2=angle, bit3=manual, bit4=estop, bit5=body, bit6=horizontal saturated, bit7=vertical saturated */
    int16_t  pwm[8];          /* current PWM values, microseconds */
    uint16_t error_count;     /* cumulative protocol errors */
    uint16_t heartbeat_missed;
    uint8_t  neutral_reason;  /* ProtoNeutralReason */
    uint8_t  active_channel;  /* 0..7, or 0xFF when no channel is active */
} ProtoStatusReport;

/** SENSOR_REPORT payload: 72 bytes (18 floats) */
typedef struct {
    float depth_m;
    float pressure_mbar;
    float water_temp_c;
    float yaw;                /* rad */
    float pitch;              /* rad */
    float roll;               /* rad */
    float accel[3];           /* m/s²  — 12 bytes */
    float gyro[3];            /* rad/s — 12 bytes */
    float mag[3];             /* µT    — 12 bytes */
    float yaw_v;              /* rad/s */
    float pitch_v;            /* rad/s */
    float roll_v;             /* rad/s */
} ProtoSensorReport;

/**
 * DEPTH_CONTROL_REPORT payload: 40 bytes.
 * flags:
 * bit0=enabled, bit1=sensor ready, bit2=sample fresh/valid,
 * bit3=PID output saturated, bit4=vertical mixer saturated,
 * bit5=actuator output ready.
 */
typedef struct {
    float requested_target_cm;
    float active_setpoint_cm;
    float measured_depth_cm;
    float error_cm;
    float p_term_us;
    float i_term_us;
    float d_term_us;
    float output_us;
    uint32_t sample_age_ms;
    uint8_t flags;
    uint8_t fault_reason;
    uint16_t reserved;
} ProtoDepthControlReport;

/** ACK payload: 2 bytes (echoes the sequence number of the acknowledged command) */
typedef struct {
    uint16_t ack_seq;
} ProtoAck;

/** NACK payload: 4 bytes */
typedef struct {
    uint16_t rejected_sequence; /* sequence of the rejected request */
    uint8_t original_type;    /* ProtoMsgType of the rejected command */
    uint8_t reason;           /* ProtoNackReason */
} ProtoNack;

/** SAFETY_EVENT payload: 4 bytes */
typedef struct {
    uint8_t event_type;       /* ProtoSafetyEventType */
    uint8_t reason_code;
    uint16_t reserved;
} ProtoSafetyEvent;

/** HEARTBEAT payload: 2 bytes */
typedef struct {
    uint16_t heartbeat_timeout_ms;  /* requested timeout from host */
} ProtoHeartbeat;

/** HEARTBEAT_ACK payload: 6 bytes */
typedef struct {
    uint8_t  safety_state;
    uint8_t  reserved;
    uint16_t uptime_s;
    uint16_t error_count;
} ProtoHeartbeatAck;

#pragma pack(pop)

/* Protocol ABI checks.  Keep these next to the wire structures so a
 * compiler/packing change fails the build instead of changing the wire. */
#if defined(__cplusplus)
static_assert(sizeof(ProtoSetPwm) == 6U, "ProtoSetPwm wire size");
static_assert(sizeof(ProtoSetDepth) == 4U, "ProtoSetDepth wire size");
static_assert(sizeof(ProtoSetYaw) == 4U, "ProtoSetYaw wire size");
static_assert(sizeof(ProtoSetMotion) == 1U, "ProtoSetMotion wire size");
static_assert(sizeof(ProtoSetBodyCommand) == 24U,
              "ProtoSetBodyCommand wire size");
static_assert(sizeof(ProtoMotionTuning) == 56U,
              "ProtoMotionTuning wire size");
static_assert(sizeof(ProtoDepthPidTuning) == 28U,
              "ProtoDepthPidTuning wire size");
static_assert(sizeof(ProtoStatusReport) == 24U, "ProtoStatusReport wire size");
static_assert(sizeof(ProtoSensorReport) == 72U, "ProtoSensorReport wire size");
static_assert(sizeof(ProtoDepthControlReport) == 40U,
              "ProtoDepthControlReport wire size");
static_assert(sizeof(ProtoAck) == 2U, "ProtoAck wire size");
static_assert(sizeof(ProtoNack) == 4U, "ProtoNack wire size");
static_assert(sizeof(ProtoSafetyEvent) == 4U, "ProtoSafetyEvent wire size");
static_assert(sizeof(ProtoHeartbeat) == 2U, "ProtoHeartbeat wire size");
static_assert(sizeof(ProtoHeartbeatAck) == 6U, "ProtoHeartbeatAck wire size");
#else
_Static_assert(sizeof(ProtoSetPwm) == 6U, "ProtoSetPwm wire size");
_Static_assert(sizeof(ProtoSetDepth) == 4U, "ProtoSetDepth wire size");
_Static_assert(sizeof(ProtoSetYaw) == 4U, "ProtoSetYaw wire size");
_Static_assert(sizeof(ProtoSetMotion) == 1U, "ProtoSetMotion wire size");
_Static_assert(sizeof(ProtoSetBodyCommand) == 24U,
               "ProtoSetBodyCommand wire size");
_Static_assert(sizeof(ProtoMotionTuning) == 56U,
               "ProtoMotionTuning wire size");
_Static_assert(sizeof(ProtoDepthPidTuning) == 28U,
               "ProtoDepthPidTuning wire size");
_Static_assert(sizeof(ProtoStatusReport) == 24U, "ProtoStatusReport wire size");
_Static_assert(sizeof(ProtoSensorReport) == 72U, "ProtoSensorReport wire size");
_Static_assert(sizeof(ProtoDepthControlReport) == 40U,
               "ProtoDepthControlReport wire size");
_Static_assert(sizeof(ProtoAck) == 2U, "ProtoAck wire size");
_Static_assert(sizeof(ProtoNack) == 4U, "ProtoNack wire size");
_Static_assert(sizeof(ProtoSafetyEvent) == 4U, "ProtoSafetyEvent wire size");
_Static_assert(sizeof(ProtoHeartbeat) == 2U, "ProtoHeartbeat wire size");
_Static_assert(sizeof(ProtoHeartbeatAck) == 6U, "ProtoHeartbeatAck wire size");
#endif

/* ------------------------------------------------------------------ */
/*  CRC16                                                              */
/* ------------------------------------------------------------------ */

uint16_t proto_crc16(const uint8_t *data, uint16_t len);

/* ------------------------------------------------------------------ */
/*  Frame header (for internal parser use)                              */
/* ------------------------------------------------------------------ */

typedef struct {
    uint8_t  magic0;
    uint8_t  magic1;
    uint8_t  version;
    uint8_t  type;
    uint16_t seq;
    uint16_t payload_len;
} ProtoFrameHeader;

#if defined(__cplusplus)
static_assert(sizeof(ProtoFrameHeader) == 8U, "ProtoFrameHeader wire size");
#else
_Static_assert(sizeof(ProtoFrameHeader) == 8U, "ProtoFrameHeader wire size");
#endif

/* ------------------------------------------------------------------ */
/*  Helper: build a complete frame into buf (returns total frame len)  */
/* ------------------------------------------------------------------ */

/**
 * @brief  Encode a protocol frame.
 * @param  buf        Output buffer (must be at least PROTO_BUF_SIZE bytes).
 * @param  buf_size   Size of output buffer.
 * @param  type       Message type.
 * @param  seq        Sequence number.
 * @param  payload    Payload bytes (may be NULL if payload_len == 0).
 * @param  payload_len Payload length.
 * @return Total frame length on success, 0 on error (buffer too small).
 */
uint16_t proto_encode_frame(uint8_t *buf, uint16_t buf_size,
                            uint8_t type, uint16_t seq,
                            const uint8_t *payload, uint16_t payload_len);

#ifdef __cplusplus
}
#endif

#endif /* PROTOCOL_SHARED_PROTOCOL_H */
