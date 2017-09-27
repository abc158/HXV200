//------------------------------------------------------------------------------
//  Copyright (C) 2014-2017, Amicro, Inc.
//  All rights reserved.
//------------------------------------------------------------------------------
#include <ui-config.h>
#include "am_date_base.h"
#include "ui-commands.h"
#include "am_config_macro.h"
#include "syscall_api.h"
#include "ui-song-player.h"
#include "act.h"

#include "state_view/state_view.h"

/*命令结构的长度*/
#define CMD_LEN  7

U8 uart_send_buf[70];                      /*存放与系统通信的数据的数组*/
schedule_time_info tmp_schedule_info;

/*配置主板当前的时间*/
static tm set_current_day;

schedule_time_info obj_schedule_info_msg_t;

extern u8 last_state;
extern u8 direction_key_up_flag;
extern u8 direction_key_pause;
/****************************************************************
*Function   :  act_command_q
*Description:  发送命令  
*Input      :  CONTROL_CMD_E ui_cmd 
*              CMD_STATE_E cmd_state
*              void *msg :数据存放的地址
*              U16 msg_len:数据长度
*Output     :  无
*Return     :  无
*Others     :  
******************************************************************/
void act_command_q(U32 ui_cmd, U8 cmd_state, void *msg, U16 msg_len)
{
  command   cmd; 
  cmd.cmd       =ui_cmd;
  cmd.cmd_type  = CONTROL_CMD;
  cmd.cmd_state = cmd_state;
  cmd.len = msg_len;
  
  view_CMD(ui_cmd,cmd_state);
  
  memcpy(uart_send_buf,&cmd,CMD_LEN);
  if(msg_len >0)
  {
    memcpy(uart_send_buf + CMD_LEN, (U8 *)msg, msg_len);    
  }
  act_ctrl_cmd(uart_send_buf);     
}

/*pack for a kind of data*/
U32 send_schedule_cmd(void)
{ 
  schedule_time_info *msg = &obj_schedule_info_msg_t;
  U16 msg_len = sizeof(schedule_time_info);
  act_command_q((U32)CMD_SCHDULE, (U8)CMD_RUN, (void *)msg, msg_len);
  return (U32)CMD_SCHDULE;
}

U32 send_time_cmd(void)
{ 
  tm *msg = &set_current_day;
  U16 msg_len = sizeof(tm);
  act_command_q((U32)CMD_CLOCK, (U8)CMD_RUN, (void *)msg, msg_len);
  return (U32)CMD_CLOCK;
}

U32 send_sleep_time_cmd(void)
{ 
  tm *msg = &set_current_day;
  printf("week=%d,hour=%d,min=%d\r\n",set_current_day.week,set_current_day.hour,set_current_day.min);
  U16 msg_len = sizeof(tm);
  act_command_q((U32)CMD_TIMING, (U8)CMD_RUN, (void *)msg, msg_len);
  return (U32)CMD_TIMING;
}

U32 send_standby_cmd(void)
{
  act_command_q((U32)CMD_POWER, (U8)CMD_RUN, (void *)NULL, 0);
  return (U32)CMD_POWER;    
}

/*get state info */
tm *get_current_time(void)
{
  return NULL;
}

schedule_time_info *get_schedule_info(void)
{
  return &(obj_schedule_info_msg_t);
}

schedule_time_info *get_tmp_schedule_info(void)
{

  return NULL;
}

void make_schedule_time(tm *t, bool en)
{
  U8 bit = (0x1<< t->week);
  tmp_schedule_info.SCH_ENABLE.BYTE &= ~bit;
  bit = (en << t->week);
  tmp_schedule_info.SCH_ENABLE.BYTE |= bit;
  memcpy(&(tmp_schedule_info.t[t->week]),t,sizeof(tm));
}

void set_schedule_time(void)
{
  memcpy(&(obj_schedule_info_msg_t),&(tmp_schedule_info),sizeof(schedule_time_info));    
}

void set_current_time(tm *t)
{
  memcpy(&set_current_day, t, sizeof(tm));
}


u8 send_drive_cmd = 0;

/****************************************************************
*Function   :  send_forward_cmd
*Description:  发送向前命令  
*Input      :  CMD_STATE_E mode
*Output     :  无
*Return     :  无
*Others     :  
******************************************************************/
void send_forward_cmd(CMD_STATE_E mode)
{ 
  s16 msg[1];
  
  if (mode == CMD_RUN)
  {
  	if (send_drive_cmd == 0)
  	{
	  if((last_state == ROBOT_STATE_DOCK) || (last_state == ROBOT_STATE_PAUSE))
	  {
	  		if(direction_key_pause != 1)
                state_save_and_recover(0);	  
	  }
	  else
	  {
			act_command_q(CMD_PAUSE, CMD_STOP, NULL, 0);
			direction_key_pause =1;
	  		sleep(10); 	  
	  }
      msg[0] = -1;//前进的距离，单位是cm，-1表示一直向前
      act_command_q(CMD_FORWARD, CMD_RUN, (void *)&msg[0], 2);
      send_drive_cmd = 1;
	  direction_key_up_flag =0;
    }
  }
  else
  {	
	act_command_q(CMD_FORWARD, CMD_STOP, NULL, 0);
	direction_key_up_flag =1;
	send_drive_cmd = 0;
  }
}

/****************************************************************
*Function   :  send_forward_cmd
*Description:  发送向后命令  
*Input      :  CMD_STATE_E mode
*Output     :  无
*Return     :  无
*Others     :  
******************************************************************/
void send_backward_cmd(CMD_STATE_E mode)
{ 
  s16 msg[1];
  
  if (mode == CMD_RUN)
  {
  	if (send_drive_cmd == 0)
  	{
	  if((last_state == ROBOT_STATE_DOCK) || (last_state == ROBOT_STATE_PAUSE))
	  {
	  		if(direction_key_pause != 1)
                state_save_and_recover(0);	  
	  }
	  else
	  {
			act_command_q(CMD_PAUSE, CMD_STOP, NULL, 0);
			direction_key_pause =1;
	  		sleep(10); 	  
	  } 
      msg[0] = -1;//后退的距离，单位是cm，-1表示一直向前
      act_command_q(CMD_BACK, CMD_RUN, (void *)&msg[0], 2);
      send_drive_cmd = 1;
	  direction_key_up_flag =0;
    }
  }
  else
  {
	act_command_q(CMD_BACK, CMD_STOP, NULL, 0);
	direction_key_up_flag =1;
  	send_drive_cmd = 0;
  }
}

/****************************************************************
*Function   :  send_left_cmd
*Description:  发送向左命令  
*Input      :  CMD_STATE_E mode
*Output     :  无
*Return     :  无
*Others     :  
******************************************************************/
void send_left_cmd(CMD_STATE_E mode)
{ 
  s16 msg[1];
  
  if (mode == CMD_RUN)
  {
  	if (send_drive_cmd == 0)
  	{
	  if((last_state == ROBOT_STATE_DOCK) || (last_state == ROBOT_STATE_PAUSE))
	  {
	  		if(direction_key_pause != 1)
                state_save_and_recover(0);	  
	  }
	  else
	  {
			act_command_q(CMD_PAUSE, CMD_STOP, NULL, 0);
			direction_key_pause =1;
	  		sleep(10); 	  
	  }
      msg[0] = -1;//转动的角度，单位是度，-1表示一直转动
      act_command_q(CMD_LEFT, CMD_RUN, (void *)&msg[0], 2);
      send_drive_cmd = 1;
	  direction_key_up_flag =0;
    }
  }
  else
  {
	act_command_q(CMD_LEFT, CMD_STOP, NULL, 0);	
	direction_key_up_flag =1;	  
  	send_drive_cmd = 0;
  }
}

/****************************************************************
*Function   :  send_right_cmd
*Description:  发送向右命令  
*Input      :  CMD_STATE_E mode
*Output     :  无
*Return     :  无
*Others     :  
******************************************************************/
void send_right_cmd(CMD_STATE_E mode)
{ 
  s16 msg[1];
  
  if (mode == CMD_RUN)
  {
  	if (send_drive_cmd == 0)
  	{
	  if((last_state == ROBOT_STATE_DOCK) || (last_state == ROBOT_STATE_PAUSE))
	  {
	  		if(direction_key_pause != 1)
                state_save_and_recover(0);	  
	  }
	  else
	  {
			act_command_q(CMD_PAUSE, CMD_STOP, NULL, 0);
			direction_key_pause =1;
	  		sleep(10); 	  
	  }
      msg[0] = -1;//转动的角度，单位是度，-1表示一直转动
      act_command_q(CMD_RIGHT, CMD_RUN, (void *)&msg[0], 2);
      send_drive_cmd = 1;
	  direction_key_up_flag =0;
    }
  }
  else
  {
	act_command_q(CMD_RIGHT, CMD_STOP, NULL, 0);
	direction_key_up_flag =1;	
  	send_drive_cmd = 0;
  }
}





