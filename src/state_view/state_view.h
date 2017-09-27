// #include "state_view.h"

extern int uart_ui;

void view_main_task(void);
void view_msg(U8 msg);
void view_LT_sensor(s16 p8,s16 p9,s16 p10,s16 p11,s16 p12,s16 p13);
void view_CMD(U32 cmd_id,U8 cmd_action);
void view_key(U32 key_val);
void view_bump(U32 bump);
