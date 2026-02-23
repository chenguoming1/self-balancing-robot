#include "bt_command.h"
#include <string.h>
#include <stdlib.h>   // atof

bool BT_Parse(RingBuffer_t *rb, BT_Command_t *cmd)
{
    static char   line[BT_CMD_BUF_LEN];
    static uint8_t idx = 0;

    uint8_t byte;
    while (rb_pop(rb, &byte)) {
        if (byte == '\r') continue;   // ignore carriage return

        if (byte == '\n') {
            // Line complete → parse
            line[idx] = '\0';
            idx = 0;

            if (line[0] == '\0') return false;

            char   type_char = line[0];
            float  val       = (line[1] != '\0') ? (float)atof(&line[1]) : 0.0f;

            switch (type_char) {
                case 'F': cmd->type = BT_CMD_FORWARD;       break;
                case 'B': cmd->type = BT_CMD_BACKWARD;      break;
                case 'S': cmd->type = BT_CMD_STOP;          break;
                case 'P': cmd->type = BT_CMD_SET_KP;        break;
                case 'I': cmd->type = BT_CMD_SET_KI;        break;
                case 'D': cmd->type = BT_CMD_SET_KD;        break;
                case 'A': cmd->type = BT_CMD_SET_ANGLE_OFFSET; break;
                default:  cmd->type = BT_CMD_NONE;          break;
            }
            cmd->value = val;
            return (cmd->type != BT_CMD_NONE);
        }

        // Buffer byte
        if (idx < BT_CMD_BUF_LEN - 1u) {
            line[idx++] = (char)byte;
        } else {
            idx = 0;  // overflow → discard line
        }
    }
    return false;
}
