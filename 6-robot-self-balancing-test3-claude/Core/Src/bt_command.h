#ifndef BT_COMMAND_H
#define BT_COMMAND_H

#include <stdint.h>
#include <stdbool.h>
#include "ring_buffer.h"

/* ASCII protocol: <CMD><VALUE>\n
   P<v>  – pid_stb Kp             I<v>  – pid_stb Ki
   D<v>  – pid_stb Kd             V<v>  – pid_vel Kp
   W<v>  – pid_vel Ki             F<v>  – throttle forward
   B<v>  – throttle backward      L<v>  – steer left
   R<v>  – steer right            S     – stop
   E     – enable                 X     – disable               */

#define BT_CMD_BUF_LEN  32u

typedef enum {
    BT_CMD_NONE = 0,
    BT_CMD_FORWARD,
    BT_CMD_BACKWARD,
    BT_CMD_STEER_LEFT,
    BT_CMD_STEER_RIGHT,
    BT_CMD_STOP,
    BT_CMD_ENABLE,
    BT_CMD_DISABLE,
    BT_CMD_SET_KP,
    BT_CMD_SET_KI,
    BT_CMD_SET_KD,
    BT_CMD_SET_VEL_KP,
    BT_CMD_SET_VEL_KI,
} BT_CmdType_t;

typedef struct {
    BT_CmdType_t type;
    float        value;
} BT_Command_t;

bool BT_Parse(RingBuffer_t *rb, BT_Command_t *cmd);

#endif /* BT_COMMAND_H */
