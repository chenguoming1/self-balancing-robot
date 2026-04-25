#ifndef BT_COMMAND_H
#define BT_COMMAND_H

#include <stdint.h>
#include <stdbool.h>
#include "ring_buffer.h"

/* ── Simple ASCII protocol ───────────────────────────────────────────────
   Command format:  <CMD><VALUE>\n
   Examples:
     F20\n   → forward throttle target 20
     B20\n   → backward throttle target 20
     T-5\n   → signed throttle target -5
     R80\n   → steering trim 80
     S\n     → stop throttle + steering
     P65\n   → set stabilisation Kp
     I220\n  → set stabilisation Ki
     D2.2\n  → set stabilisation Kd
     A0.7\n  → set balance trim / neutral angle offset
     V0.08\n → set velocity Kp
     W0.25\n → set velocity Ki
     X0.0\n  → set velocity Kd
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
    BT_CMD_SET_THROTTLE,
    BT_CMD_SET_STEERING,
    BT_CMD_SET_VEL_KP,
    BT_CMD_SET_VEL_KI,
    BT_CMD_SET_VEL_KD,
} BT_CmdType_t;

typedef struct {
    BT_CmdType_t type;
    float        value;
} BT_Command_t;

/** Feed bytes from ring buffer, parse complete commands.
 *  Returns true when a full command is ready in *cmd. */
bool BT_Parse(RingBuffer_t *rb, BT_Command_t *cmd);

#endif /* BT_COMMAND_H */
