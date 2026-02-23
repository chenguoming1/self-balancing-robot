#ifndef BT_COMMAND_H
#define BT_COMMAND_H

#include <stdint.h>
#include <stdbool.h>
#include "ring_buffer.h"

/* ── Simple ASCII protocol ───────────────────────────────────────────────
   Command format:  <CMD><VALUE>\n
   Examples:
     F100\n  → forward, speed override 100
     B100\n  → backward
     S\n     → stop
     P2.5\n  → set Kp = 2.5
     I0.1\n  → set Ki = 0.1
     D0.05\n → set Kd = 0.05
     A1.5\n  → set balance setpoint angle = 1.5°
   ─────────────────────────────────────────────────────────────────────── */

#define BT_CMD_BUF_LEN  32u

typedef enum {
    BT_CMD_NONE = 0,
    BT_CMD_FORWARD,
    BT_CMD_BACKWARD,
    BT_CMD_STOP,
    BT_CMD_SET_KP,
    BT_CMD_SET_KI,
    BT_CMD_SET_KD,
    BT_CMD_SET_ANGLE_OFFSET,
} BT_CmdType_t;

typedef struct {
    BT_CmdType_t type;
    float        value;
} BT_Command_t;

/** Feed bytes from ring buffer, parse complete commands.
 *  Returns true when a full command is ready in *cmd. */
bool BT_Parse(RingBuffer_t *rb, BT_Command_t *cmd);

#endif /* BT_COMMAND_H */
