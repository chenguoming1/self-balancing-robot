#include "bt_command.h"
#include <string.h>
#include <stdlib.h>

bool BT_Parse(RingBuffer_t *rb, BT_Command_t *cmd)
{
    static char    line[BT_CMD_BUF_LEN];
    static uint8_t idx = 0;

    uint8_t byte;
    while (rb_pop(rb, &byte)) {
        if (byte == '\r') continue;

        if (byte == '\n') {
            line[idx] = '\0';
            idx = 0;
            if (line[0] == '\0') return false;

            char  type_char = line[0];
            float val       = (line[1] != '\0') ? (float)atof(&line[1]) : 0.0f;

            switch (type_char) {
                case 'F': cmd->type = BT_CMD_FORWARD;     break;
                case 'B': cmd->type = BT_CMD_BACKWARD;    break;
                case 'L': cmd->type = BT_CMD_STEER_LEFT;  break;
                case 'R': cmd->type = BT_CMD_STEER_RIGHT; break;
                case 'S': cmd->type = BT_CMD_STOP;        break;
                case 'E': cmd->type = BT_CMD_ENABLE;      break;
                case 'X': cmd->type = BT_CMD_DISABLE;     break;
                case 'P': cmd->type = BT_CMD_SET_KP;      break;
                case 'I': cmd->type = BT_CMD_SET_KI;      break;
                case 'D': cmd->type = BT_CMD_SET_KD;      break;
                case 'V': cmd->type = BT_CMD_SET_VEL_KP;  break;
                case 'W': cmd->type = BT_CMD_SET_VEL_KI;  break;
                default:  cmd->type = BT_CMD_NONE;        break;
            }
            cmd->value = val;
            return (cmd->type != BT_CMD_NONE);
        }

        if (idx < BT_CMD_BUF_LEN - 1u) {
            line[idx++] = (char)byte;
        } else {
            idx = 0;
        }
    }
    return false;
}
